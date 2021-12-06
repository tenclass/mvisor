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

class VirtioBlock : public VirtioPci {
 private:
  virtio_blk_config block_config_;
  DiskImage* image_ = nullptr;

 public:
  VirtioBlock() {
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
    if (image_) {
      delete image_;
    }
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
  }

  void Reset() {
    VirtioPci::Reset();
  
    // CreateQueuesForPorts();
  }
};

DECLARE_DEVICE(VirtioBlock);
