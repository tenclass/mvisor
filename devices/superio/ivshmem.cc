/* 
 * MVisor
 * Copyright (C) 2025 Andy <550896603@qq.com>
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

#include "logger.h"
#include "pci_device.h"
#include "ivshmem.pb.h"
#include <fcntl.h>
#include <sys/mman.h>

class Ivshmem : public PciDevice {
 private:
  size_t shmem_size_ = 512 * 1024 * 1024;
  void* shmem_base_ = nullptr;
  int shmem_fd_ = -1;

 public:
  Ivshmem() {
    pci_header_.vendor_id = 0x1af4;
    pci_header_.device_id = 0x1110;
    pci_header_.revision_id = 1;
    pci_header_.class_code = 0x0500;

    SetupPciBar(0, 256, kIoResourceTypeMmio);
    SetupPciBar(2, shmem_size_, kIoResourceTypeRam);
  }

  ~Ivshmem() {
  }

  void Connect() {
    if (has_key("shmem_size")) {
      auto size = std::get<uint64_t>(key_values_["shmem_size"]);

      if (size < 512) {
        MV_PANIC("shmem_size must be at least 512MB");
        return;
      }

      if (size & (size - 1)) {
        MV_PANIC("shmem_size must be power of 2");
        return;
      }
      shmem_size_ = size << 20;
    }

    if (has_key("shmem_path")) {
      auto path = std::get<std::string>(key_values_["shmem_path"]);
      
      shmem_fd_ = shm_open(path.c_str(), O_RDWR, 0666);
      if (shmem_fd_ < 0) {
        MV_PANIC("Failed to open shared memory file %s", path.c_str());
        return;
      }

      shmem_base_ = mmap(nullptr, shmem_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shmem_fd_, 0);
    } else {
      shmem_base_ = mmap(nullptr, shmem_size_, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
    }

    if (shmem_base_ == MAP_FAILED) {
      MV_PANIC("Failed to map shared memory for Ivshmem");
      return;
    }
    MV_ASSERT(madvise(shmem_base_, shmem_size_, MADV_DONTDUMP) == 0);

    pci_bars_[2].size = shmem_size_;
    pci_bars_[2].host_memory = shmem_base_;

    PciDevice::Connect();
  }

  void Disconnect() {
    if (shmem_base_) {
      munmap(shmem_base_, shmem_size_);
      shmem_base_ = nullptr;
    }

    if (shmem_fd_ != -1) {
      close(shmem_fd_);
      shmem_fd_ = -1;
    }

    PciDevice::Disconnect();
  }

  // https://www.qemu.org/docs/master/specs/ivshmem-spec.html
  void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    if (resource->base == pci_bars_[0].address) {
      switch (offset) {
        case 0x8:
          MV_ASSERT(size == 4);
          memset(data, 0, size);
          break;
        default:
          break;
      }
    }
  }

  bool SaveState(MigrationWriter* writer) {
    IvshmemState state;
    state.set_shmem((char*)shmem_base_, shmem_size_);

    writer->WriteProtobuf("ivshmem", state);
    return Device::SaveState(writer);
  }

  bool LoadState(MigrationReader* reader) {
    if (!Device::LoadState(reader)) {
      return false;
    }

    IvshmemState state;
    if (!reader->ReadProtobuf("ivshmem", state)) {
      MV_PANIC("Failed to load ivshmem state");
      return false;
    }

    MV_ASSERT(state.shmem().size() == shmem_size_);
    memcpy(shmem_base_, state.shmem().data(), shmem_size_);

    return true;
  }
};

DECLARE_DEVICE(Ivshmem);
