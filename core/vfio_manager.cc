#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <sys/ioctl.h>
#include <linux/kvm.h>

#include "vfio_manager.h"
#include "logger.h"
#include "machine.h"


VfioManager::VfioManager(Machine* machine) {
  machine_ = machine;
}

VfioManager::~VfioManager() {
  for (auto& [_, group] : iommu_groups_) {
    FreeIommuGroup(group);
  }
  iommu_groups_.clear();

  if (memory_listener_) {
    machine_->memory_manager()->UnregisterMemoryListener(&memory_listener_);
  }
  if (dirty_memory_listener_) {
    machine_->memory_manager()->UnregisterDirtyMemoryListener(&dirty_memory_listener_);
  }
  safe_close(&container_fd_);
  safe_close(&kvm_device_fd_);
}

void VfioManager::FreeIommuGroup(IommuGroup* group) {
  /* Remove group from KVM device */
  if (kvm_device_fd_ != -1) {
    kvm_device_attr attr = {
      .flags = 0,
      .group = KVM_DEV_VFIO_GROUP,
      .attr = KVM_DEV_VFIO_GROUP_DEL,
      .addr = (uint64_t)&group->group_fd
    };
    if (ioctl(kvm_device_fd_, KVM_SET_DEVICE_ATTR, &attr) < 0) {
      MV_PANIC("failed to remove group %d from KVM VFIO device %d", group->group_fd, kvm_device_fd_);
    }
  }

  safe_close(&group->group_fd);
  delete group;
}

IommuGroup* VfioManager::GetIommuGroup(std::string sysfs_path) {
  /* Get VFIO group id from device path */
  char path[1024];
  auto len = readlink((sysfs_path + "/iommu_group").c_str(), path, 1024);
  if (len < 0) {
    MV_PANIC("failed to read iommu_group");
  }
  path[len] = 0;

  int group_id;
  if (sscanf(basename(path), "%d", &group_id) != 1) {
    MV_PANIC("failed to get group id from %s", path);
  }

  /* Check if group already exists */
  auto it = iommu_groups_.find(group_id);
  if (it != iommu_groups_.end()) {
    return it->second;
  }
  
  /* Open group */
  sprintf(path, "/dev/vfio/%d", group_id);
  int group_fd = open(path, O_RDWR);
  if (group_fd < 0) {
    MV_PANIC("failed to open %s", path);
  }

  /* Check if it is OK */
  vfio_group_status status = { .argsz = sizeof(status), .flags = 0 };
  MV_ASSERT(ioctl(group_fd, VFIO_GROUP_GET_STATUS, &status) == 0);
  if (!(status.flags & VFIO_GROUP_FLAGS_VIABLE)) {
    MV_PANIC("VFIO group %d is not viable", group_id);
  }

  /* Add group to container */
  if (container_fd_ == -1) {
    /* Create container */
    container_fd_ = open("/dev/vfio/vfio", O_RDWR);
    if (container_fd_ < 0) {
      MV_PANIC("failed to open /dev/vfio/vfio");
    }

    /* Here use type1 iommu */
    MV_ASSERT(ioctl(container_fd_, VFIO_GET_API_VERSION) == VFIO_API_VERSION);
    MV_ASSERT(ioctl(container_fd_, VFIO_CHECK_EXTENSION, VFIO_TYPE1v2_IOMMU));
    MV_ASSERT(ioctl(group_fd, VFIO_GROUP_SET_CONTAINER, &container_fd_) == 0);
    MV_ASSERT(ioctl(container_fd_, VFIO_SET_IOMMU, VFIO_TYPE1v2_IOMMU) == 0);

    SetupDmaMaps();
  } else {
    MV_ASSERT(ioctl(group_fd, VFIO_GROUP_SET_CONTAINER, &container_fd_) == 0);
  }

  /* Add group to KVM device
   * Currently disabled this feature because it makes the vGPU snapshots CPU 100% while restoring
   */
  // if (kvm_device_fd_ == -1) {
  //   kvm_create_device create = {
  //     .type = KVM_DEV_TYPE_VFIO,
  //     .fd = 0,
  //     .flags = 0
  //   };
  //   if (ioctl(machine_->vm_fd_, KVM_CREATE_DEVICE, &create) < 0) {
  //     MV_PANIC("failed to create KVM VFIO device");
  //   }
  //   kvm_device_fd_ = create.fd;
  // }

  // kvm_device_attr attr = {
  //   .flags = 0,
  //   .group = KVM_DEV_VFIO_GROUP,
  //   .attr = KVM_DEV_VFIO_GROUP_ADD,
  //   .addr = (uint64_t)&group_fd
  // };
  // if (ioctl(kvm_device_fd_, KVM_SET_DEVICE_ATTR, &attr) < 0) {
  //   MV_PANIC("failed to add group %d to KVM VFIO device %d", group_fd, kvm_device_fd_);
  // }

  /* Create IommuGroup */
  auto group = new IommuGroup();
  group->group_id = group_id;
  group->group_fd = group_fd;
  iommu_groups_[group_id] = group;
  return group;
}


int VfioManager::AttachDevice(std::string sysfs_path) {
  auto group = GetIommuGroup(sysfs_path);

  int device_fd = ioctl(group->group_fd, VFIO_GROUP_GET_DEVICE_FD, basename(sysfs_path.c_str()));
  if (device_fd < 0) {
    MV_PANIC("failed to get device fd for %s", sysfs_path.c_str());
  }

  group->device_fds.insert(device_fd);
  return device_fd;
}


void VfioManager::DetachDevice(int device_fd) {
  for (auto& [group_id, group] : iommu_groups_) {
    if (group->device_fds.erase(device_fd)) {
      if (group->device_fds.empty()) {
        FreeIommuGroup(group);
        iommu_groups_.erase(group_id);
      }
      return;
    }
  }
  MV_ERROR("device fd %d not found", device_fd);
}

void VfioManager::MapDmaPages(const MemorySlot& slot) {
  vfio_iommu_type1_dma_map dma_map = {
    .argsz = sizeof(dma_map),
    .flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE,
    .vaddr = slot.hva,
    .iova = slot.begin,
    .size = slot.size
  };

  if (ioctl(container_fd_, VFIO_IOMMU_MAP_DMA, &dma_map) < 0) {
    MV_PANIC("failed to map vaddr=0x%lx size=0x%lx", dma_map.iova, dma_map.size);
  }
}

void VfioManager::UnmapDmaPages(const MemorySlot& slot) {
  vfio_iommu_type1_dma_unmap dma_ummap = {
    .argsz = sizeof(dma_ummap),
    .flags = 0,
    .iova = slot.begin,
    .size = slot.size
  };

  if (ioctl(container_fd_, VFIO_IOMMU_UNMAP_DMA, &dma_ummap) < 0) {
    MV_PANIC("failed to unmap vaddr=0x%lx size=0x%lx", dma_ummap.iova, dma_ummap.size);
  }
}

void VfioManager::SetupDmaMaps() {
  auto mm = machine_->memory_manager();

  /* Map all current slots */
  for (const auto& slot : mm->GetMemoryFlatView()) {
    if (slot.is_system()) {
      MapDmaPages(slot);
    }
  }

  /* Add memory listener to keep DMA maps synchronized */
  memory_listener_ = mm->RegisterMemoryListener([this](auto slot, bool unmap) {
    if (slot.is_system()) {
      if (unmap) {
        UnmapDmaPages(slot);
      } else {
        MapDmaPages(slot);
      }
    }
  });

  /* Add dirty memory listener to track memory in DMA */
  dirty_memory_listener_ = mm->RegisterDirtyMemoryListener([this](const DirtyMemoryCommand command) {
    std::vector<struct DirtyMemoryBitmap> bitmaps;
    switch (command) {
      case kGetDirtyMemoryBitmap: {
        auto dirty_buffer_size = sizeof(vfio_iommu_type1_dirty_bitmap) + sizeof(vfio_iommu_type1_dirty_bitmap_get);
        uint8_t dirty_buffer[dirty_buffer_size];

        auto dirty_bitmap = (struct vfio_iommu_type1_dirty_bitmap*)dirty_buffer;
        auto dirty_bitmap_get = (struct vfio_iommu_type1_dirty_bitmap_get*)dirty_bitmap->data;

        dirty_bitmap->argsz = dirty_buffer_size;
        dirty_bitmap->flags = VFIO_IOMMU_DIRTY_PAGES_FLAG_GET_BITMAP;

        auto slots = machine_->memory_manager()->GetMemoryFlatView();
        for (const auto& slot : slots) {
          if (!slot.is_system()) {
            continue;
          }

          size_t bitmap_size = ALIGN(slot.size / PAGE_SIZE, 64) / 8;
          std::string bitmap(bitmap_size, '\0');

          // get dirty memory bitmap for this slot from vfio
          dirty_bitmap_get->iova = slot.begin;
          dirty_bitmap_get->size = slot.size;
          dirty_bitmap_get->bitmap = {
            .pgsize = PAGE_SIZE,
            .size = bitmap_size,
            .data = (__u64*)bitmap.data()
          };
          MV_ASSERT(ioctl(container_fd_, VFIO_IOMMU_DIRTY_PAGES, dirty_bitmap) == 0);

          // save it to return for network migration
          bitmaps.push_back({
            .region = {
              .hva = slot.hva,
              .begin = slot.begin,
              .end = slot.end
            },
            .data = std::move(bitmap)
          });
        }
        break;
      }
      case kStopTrackingDirtyMemory:
      case kStartTrackingDirtyMemory: {
        struct vfio_iommu_type1_dirty_bitmap dirty_bitmap = {
          .argsz = sizeof(dirty_bitmap),
          .flags = (uint32_t)(command == kStartTrackingDirtyMemory ? 
            VFIO_IOMMU_DIRTY_PAGES_FLAG_START : VFIO_IOMMU_DIRTY_PAGES_FLAG_STOP)
        };
        MV_ASSERT(ioctl(container_fd_, VFIO_IOMMU_DIRTY_PAGES, &dirty_bitmap) == 0);
        break;
      }
      default:
        MV_PANIC("unknown command %d", command);
        break;
    }
    return bitmaps;
  });
}
