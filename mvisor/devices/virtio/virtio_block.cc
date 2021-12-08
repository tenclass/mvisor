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
#include <linux/virtio_blk.h>
#include "logger.h"
#include "disk_image.h"
#include "machine.h"

class VirtioBlock : public VirtioPci {
 private:
  virtio_blk_config block_config_;
  DiskImage* image_ = nullptr;

 public:
  VirtioBlock() {
    devfn_ = PCI_MAKE_DEVFN(6, 0);
    pci_header_.class_code = 0x010000;
    pci_header_.device_id = 0x1001;
    pci_header_.subsys_id = 0x0002;
    
    device_features_ |= (1UL << VIRTIO_BLK_F_SEG_MAX) |
      (1UL << VIRTIO_BLK_F_GEOMETRY) |
      (1UL << VIRTIO_BLK_F_BLK_SIZE) |
      (1UL << VIRTIO_BLK_F_FLUSH) |
      (1UL << VIRTIO_BLK_F_TOPOLOGY) |
      (1UL << VIRTIO_BLK_F_WCE) |
      (1UL << VIRTIO_BLK_F_MQ) |
      (1UL << VIRTIO_BLK_F_DISCARD) |
      (1UL << VIRTIO_BLK_F_WRITE_ZEROES);
    bzero(&block_config_, sizeof(block_config_));
  }

  virtual ~VirtioBlock() {
  }

  void Connect() {
    VirtioPci::Connect();

    /* Connect to backend image */
    for (auto object : children_) {
      auto image = dynamic_cast<DiskImage*>(object);
      if (image) {
        image->Connect();
        image_ = image;
        break;
      }
    }
    if (image_) {
      InitializeGeometry();
    }
  }

  void InitializeGeometry() {
    auto information = image_->information();
    block_config_.capacity = information.total_blocks;
    block_config_.blk_size = information.block_size;
  }

  void Reset() {
    /* Reset all queues */
    VirtioPci::Reset();
  
    for (int i = 0; i < 1; ++i) {
      AddQueue(256, std::bind(&VirtioBlock::OnOutput, this, i));
    }
  }

  void ReadDeviceConfig(uint64_t offset, uint8_t* data, uint32_t size) {
    MV_ASSERT(offset + size <= sizeof(block_config_));
    memcpy(data, (uint8_t*)&block_config_ + offset, size);
    MV_LOG("read device config at 0x%lx size=%x", offset, size);
  }

  void OnOutput(int queue_index) {
    auto &vq = queues_[queue_index];
    VirtElement element;
  
    while (PopQueue(vq, element)) {
      HandleCommand(element);
      PushQueue(vq, element);
      NotifyQueue(vq);
      // printf("virtqueue %d push used_idx=%d id=%d len=0x%x in_num=%lu out_num=%lu in_size=%lx out_size=0x%lx\n",
      //   queue_index, vq.used_ring->index, element.id, element.length, element.write_vector.size(), element.read_vector.size(),
      //   element.write_size, element.read_size);
    }
  }

  void HandleCommand(VirtElement& element) {
    virtio_blk_outhdr* request = (virtio_blk_outhdr*)element.read_vector[0].iov_base;
    auto &last_write_iov = element.write_vector[element.write_vector.size() - 1];
    MV_ASSERT(last_write_iov.iov_len == 1);
    uint8_t* status = (uint8_t*)last_write_iov.iov_base;
    element.length = element.write_size;

    MV_LOG("request type=%x sector=%lx read_size=%lu:%lu write_size=%lu:%lu",
      request->type, request->sector,
      element.read_vector.size(), element.read_size, element.write_vector.size(), element.write_size);
    switch (request->type)
    {
    case VIRTIO_BLK_T_IN: {
      MV_ASSERT(element.read_vector.size() == 1);
      size_t position = request->sector * block_config_.blk_size;
      for (size_t index = 0; index < element.write_vector.size() - 1; index++) {
        void* buffer = element.write_vector[index].iov_base;
        size_t length = element.write_vector[index].iov_len;
        size_t bytes = (size_t )image_->Read(buffer, position, length);
        if (bytes != length) {
          MV_PANIC("failed read bytes=%lx pos=%lx length=%lx", bytes, position, length);
        }
        position += length;
      }
      *status = 0;
      break;
    }
    case VIRTIO_BLK_T_OUT: {
      MV_ASSERT(element.write_vector.size() == 1);
      size_t position = request->sector * block_config_.blk_size;
      for (size_t index = 1; index < element.read_vector.size(); index++) {
        void* buffer = element.read_vector[index].iov_base;
        size_t length = element.read_vector[index].iov_len;
        size_t bytes = (size_t )image_->Write(buffer, position, length);
        if (bytes != length) {
          MV_PANIC("failed write bytes=%lx pos=%lx length=%lx", bytes, position, length);
        }
        position += length;
      }
      *status = 0;
      break;
    }
    case VIRTIO_BLK_T_FLUSH:
      MV_ASSERT(element.write_vector.size() == 1);
      image_->Flush();
      *status = 0;
      break;
    case VIRTIO_BLK_T_GET_ID: {
      void* buffer = element.write_vector[0].iov_base;
      MV_ASSERT(element.write_vector[0].iov_len >= 20);
      strcpy((char*)buffer, "virtio-block");
      *status = 0;
      break;
    }
    default:
      MV_PANIC("unhandled command type=0x%x", request->type);
      break;
    }
  }
};

DECLARE_DEVICE(VirtioBlock);
