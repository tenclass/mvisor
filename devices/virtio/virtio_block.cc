/* 
 * MVisor VirtIO Block Device
 * Copyright (C) 2021 Terrence <terrence@tenclass.com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "virtio_pci.h"
#include <cstring>
#include <cmath>
#include "linuz/virtio_blk.h"
#include "logger.h"
#include "disk_image.h"
#include "qcow2.h"
#include "machine.h"

#define DEFAULT_QUEUE_SIZE 256

class VirtioBlock : public VirtioPci {
 private:
  virtio_blk_config block_config_;
  DiskImage* image_ = nullptr;

 public:
  VirtioBlock() {
    pci_header_.class_code = 0x010000;
    pci_header_.device_id = 0x1001;
    pci_header_.subsys_id = 0x0002;
    
    AddPciBar(1, 0x1000, kIoResourceTypeMmio);
    AddMsiXCapability(1, 9, 0, 0x1000);

    device_features_ |= (1UL << VIRTIO_BLK_F_SEG_MAX) |
      // (1UL << VIRTIO_BLK_F_GEOMETRY) |
      (1UL << VIRTIO_BLK_F_BLK_SIZE) |
      (1UL << VIRTIO_BLK_F_FLUSH) |
      // (1UL << VIRTIO_BLK_F_TOPOLOGY) |
      (1UL << VIRTIO_BLK_F_WCE) |
      (1UL << VIRTIO_BLK_F_MQ);
    bzero(&block_config_, sizeof(block_config_));
  }

  virtual void Disconnect() {
    if (image_) {
      VirtioPci::Disconnect();
      delete image_;
      image_ = nullptr;
    }
  }

  virtual void Connect() {
    /* Connect to backend image */
    bool readonly = has_key("readonly") && std::get<bool>(key_values_["readonly"]);
    bool snapshot = has_key("snapshot") && std::get<bool>(key_values_["snapshot"]);
    if (has_key("image")) {
      std::string path = std::get<std::string>(key_values_["image"]);
      image_ = DiskImage::Create(this, path, readonly, snapshot);

      /* Qcow2 supports disacard & write zeros */
      if (path.find(".qcow2") != std::string::npos) {
        device_features_ |=  (1UL << VIRTIO_BLK_F_DISCARD) | (1UL << VIRTIO_BLK_F_WRITE_ZEROES);
      }
    }
    if (image_) {
      VirtioPci::Connect();

      InitializeGeometry();
      if (readonly) {
        device_features_ |= VIRTIO_BLK_F_RO;
      }
    }
  }

  virtual bool LoadState(MigrationReader* reader) {
    if (!VirtioPci::LoadState(reader)) {
      return false;
    }

    // Reset image file for network migration
    if (dynamic_cast<MigrationNetworkReader*>(reader)) {
      auto image = dynamic_cast<Qcow2Image*>(image_);
      if (image) {
        image->Reset();
      }
    }
    return true;
  }

  void InitializeGeometry() {
    auto information = image_->information();
    block_config_.capacity = information.total_blocks;
    block_config_.blk_size = information.block_size;

    block_config_.num_queues = 8; // msix table size = num_queues + 1
    block_config_.seg_max = DEFAULT_QUEUE_SIZE - 2;
    block_config_.wce = 1; // write back (enable cache)
    block_config_.max_discard_sectors = __INT_MAX__ / block_config_.blk_size;
    block_config_.max_discard_seg = 1;
    block_config_.discard_sector_alignment = 1;
    block_config_.max_write_zeroes_sectors = block_config_.max_discard_sectors;
    block_config_.max_write_zeroes_seg = block_config_.max_discard_seg;
    block_config_.write_zeroes_may_unmap = 1;
  }

  void Reset() {
    /* Reset all queues */
    VirtioPci::Reset();
  
    for (int i = 0; i < block_config_.num_queues; ++i) {
      AddQueue(DEFAULT_QUEUE_SIZE, std::bind(&VirtioBlock::OnOutput, this, i));
    }
  }

  void ReadDeviceConfig(uint64_t offset, uint8_t* data, uint32_t size) {
    MV_ASSERT(offset + size <= sizeof(block_config_));
    memcpy(data, (uint8_t*)&block_config_ + offset, size);
  }

  void OnOutput(int queue_index) {
    auto &vq = queues_[queue_index];

    while (auto element = PopQueue(vq)) {
      HandleCommand(vq, element, [=, &vq]() {
        PushQueue(vq, element);
        NotifyQueue(vq);
      });
    }
  }

  void BlockIoAsync(VirtElement* element, size_t position, bool is_write, IoCallback callback) {
    auto vector(element->vector);
    for (auto &iov : vector) {
      void* buffer = iov.iov_base;
      size_t length = iov.iov_len;

      auto io_complete = [element, position, length, is_write, callback](auto ret) {
        if (!is_write && ret != (ssize_t)length) {
          MV_PANIC("failed IO ret=%lx pos=%lx length=%lx", ret, position, length);
        }
        if (!is_write) {
          element->length += length;
        }
        element->vector.pop_back();
        if (element->vector.empty()) {
          callback(ret == (ssize_t)length ? VIRTIO_BLK_S_OK : VIRTIO_BLK_S_IOERR);
        }
      };
      if (debug_) {
        MV_LOG("%s pos=0x%lx len=0x%lx ", is_write ? "write" : "read", position, length);
      }

      ImageIoRequest r = {
        .type = is_write ? kImageIoWrite : kImageIoRead,
        .position = position,
        .length = length
      };
      r.vector.emplace_back(iovec {
        .iov_base = buffer,
        .iov_len = length
      });
      image_->QueueIoRequest(r, std::move(io_complete));
      position += length;
    }
  }

  void HandleCommand(VirtQueue& vq, VirtElement* element, VoidCallback callback) {
    MV_UNUSED(vq);

    auto &vector = element->vector;
    /* Read block header */
    virtio_blk_outhdr* request = (virtio_blk_outhdr*)vector.front().iov_base;
    vector.pop_front();

    /* Get status header at the end of vector, currently only 1 byte */
    uint8_t* status = (uint8_t*)vector.back().iov_base;
    MV_ASSERT(vector.back().iov_len == 1);
    vector.pop_back();

    /* Set the vring data length to bytes returned */
    element->length = sizeof(*status);

    switch (request->type)
    {
    case VIRTIO_BLK_T_IN:
    case VIRTIO_BLK_T_OUT: {
      size_t position = request->sector * block_config_.blk_size;
      bool is_write = request->type == VIRTIO_BLK_T_OUT;
      BlockIoAsync(element, position, is_write, [callback = std::move(callback), status](auto ret) {
        *status = ret;
        callback();
      });
      break;
    }
    case VIRTIO_BLK_T_FLUSH: {
      ImageIoRequest r = {
        .type = kImageIoFlush
      };
      image_->QueueIoRequest(r, [callback = std::move(callback), status](ssize_t ret) {
        *status = ret == 0 ? VIRTIO_BLK_S_OK : VIRTIO_BLK_S_IOERR;
        callback();
      });
      break;
    }
    case VIRTIO_BLK_T_GET_ID: {
      auto &iov = vector.front();
      void* buffer = iov.iov_base;
      MV_ASSERT(iov.iov_len >= 20);

      strcpy((char*)buffer, "virtio-block");
      element->length += iov.iov_len;
      *status = VIRTIO_BLK_S_OK;
      callback();
      break;
    }
    case VIRTIO_BLK_T_WRITE_ZEROES:
    case VIRTIO_BLK_T_DISCARD: {
      auto &iov = vector.front();
      auto discard = (virtio_blk_discard_write_zeroes*)iov.iov_base;
      MV_ASSERT(iov.iov_len == sizeof(*discard));
      size_t position = discard->sector * block_config_.blk_size;
      size_t length = discard->num_sectors * block_config_.blk_size;

      ImageIoRequest r = {
        .type = request->type == VIRTIO_BLK_T_WRITE_ZEROES ? kImageIoWriteZeros : kImageIoDiscard,
        .position = position,
        .length = length
      };
      image_->QueueIoRequest(r, [status, callback = std::move(callback), length](auto ret) {
        *status = ret == (ssize_t)length ? VIRTIO_BLK_S_OK : VIRTIO_BLK_S_IOERR;
        callback();
      });
      break;
    }
    default:
      MV_PANIC("unhandled command type=0x%x", request->type);
      break;
    }
  }
};

DECLARE_DEVICE(VirtioBlock);
