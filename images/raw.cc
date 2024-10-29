/* 
 * MVisor
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

#include "disk_image.h"

#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <filesystem>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <linux/fs.h>


#include "logger.h"
#include "device_manager.h"

class RawImage : public DiskImage {
 private:
  int fd_ = -1;
  size_t block_size_ = 512;
  size_t total_blocks_ = 0;

  ImageInformation information() {
    return ImageInformation {
      .block_size = block_size_,
      .total_blocks = total_blocks_
    };
  }

  virtual ~RawImage() {
    if (fd_ != -1) {
      FlushAll();
      safe_close(&fd_);
    }

    if (snapshot_) {
      remove(filepath_.c_str());
    }
  }

  void Initialize() {
    int oflags = readonly_ ? O_RDONLY : O_RDWR;
    if (snapshot_) {
      char temp[] = "/tmp/snapshot_XXXXXX.img";
      close(mkstemps(temp, 4));
      std::filesystem::copy_file(filepath_, temp, std::filesystem::copy_options::overwrite_existing);
      filepath_ = temp;
    }
    fd_ = open(filepath_.c_str(), oflags);
    if (fd_ < 0)
      MV_PANIC("disk file not found: %s", filepath_.c_str());
    long long size_blockdevice;
    if(ioctl(fd_,BLKGETSIZE64,&size_blockdevice)!=-1) {
      total_blocks_ = size_blockdevice / block_size_;
      block_size_ = 512;
      total_blocks_ = size_blockdevice / block_size_;
    } else {
      MV_PANIC("ioctl failed");
    }
  }

  long HandleIoRequest(const ImageIoRequest& request) {
    long ret = -1;

    switch (request.type)
    {
    case kImageIoRead:
    case kImageIoWrite: {
      size_t rw_total = 0, pos = request.position;
      for (auto &iov : request.vector) {
        if (request.type == kImageIoRead) {
          ret = pread(fd_, iov.iov_base, iov.iov_len, pos);
        } else {
          ret = pwrite(fd_, iov.iov_base, iov.iov_len, pos);
        }
        if (ret <= 0) {
          return ret;
        }
        rw_total += ret;
        pos += ret;
      }
      ret = rw_total;
      break;
    }
    case kImageIoFlush:
      ret = FlushAll();
      break;
    default:
      MV_ERROR("unhandled io request %d", request.type);
      break;
    }
    return ret;
  }

  ssize_t FlushAll() {
    if (readonly_) {
      return 0;
    } else {
      return fsync(fd_);
    }
  }

};

DECLARE_DISK_IMAGE(RawImage);
