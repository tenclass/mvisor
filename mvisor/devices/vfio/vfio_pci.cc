/* 
 * MVisor VFIO for vGPU
 * Copyright (C) 2022 Terrence <terrence@tenclass.com>
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

#include "vfio_pci.h"
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/prctl.h>
#include <sys/mman.h>
#include <sys/eventfd.h>
#include "device_manager.h"
#include "logger.h"

VfioPci::VfioPci() {
  for (auto &interrupt : interrupts_) {
    interrupt.event_fd = -1;
    interrupt.gsi = -1;
  }
}

VfioPci::~VfioPci() {

}

void VfioPci::Connect() {
  PciDevice::Connect();

  if (!has_key("sysfs")) {
    MV_PANIC("Please specify 'sysfs' for vfio-pci, like '/sys/bus/mdev/devices/xxx'");
  }
  sysfs_path_ = std::get<std::string>(key_values_["sysfs"]);

  SetupVfioGroup();
  SetupVfioContainer();
  SetupVfioDevice();
  SetupPciConfiguration();
  SetupGfxPlane();
  SetupDmaMaps();
  SetupMigraionInfo();
}

void VfioPci::Disconnect() {
  auto machine = manager_->machine();
  if (memory_listener_) {
    machine->memory_manager()->UnregisterMemoryListener(&memory_listener_);
  }
  if (state_change_listener_) {
    machine->UnregisterStateChangeListener(&state_change_listener_);
  }

  for (auto &interrupt : interrupts_) {
    if (interrupt.gsi > 0) {
      manager_->UpdateMsiRoute(interrupt.gsi, 0, 0, -1);
    }
    if (interrupt.event_fd > 0) {
      // If we use IRQFD, we don't use polling to handle interrupts
      // manager_->io()->StopPolling(interrupt.event_fd);
      safe_close(&interrupt.event_fd);
    }
  }

  manager_->UnregisterVfioGroup(group_fd_);

  safe_close(&device_fd_);
  safe_close(&container_fd_);
  safe_close(&group_fd_);
  PciDevice::Disconnect();
}

void VfioPci::Reset() {
  /* disable IO / MMIO / bus master and INTX */
  uint16_t command = pci_header_.command & ~(PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER | PCI_COMMAND_INTX_DISABLE);
  WritePciConfigSpace(offsetof(PciConfigHeader, command), (uint8_t*)&command, 2);

  /* reset vfio device */
  if (device_fd_ > 0 && (device_info_.flags & VFIO_DEVICE_FLAGS_RESET)) {
    if (ioctl(device_fd_, VFIO_DEVICE_RESET) < 0) {
      MV_PANIC("failed to reset device %s", name_);
    }
  }

  PciDevice::Reset();
}

void VfioPci::SetupPciConfiguration() {
  /* Read PCI configuration from device */
  auto &config_region = regions_[VFIO_PCI_CONFIG_REGION_INDEX];
  size_t config_size = PCI_DEVICE_CONFIG_SIZE;
  MV_ASSERT(config_region.size >= config_size);
  auto ret = pread(device_fd_, pci_header_.data, config_size, config_region.offset);
  if (ret < (int)config_size) {
    MV_PANIC("failed to read device config space, ret=%d", ret);
  }
  /* Multifunction is not supported yet */
  pci_header_.header_type &= ~PCI_MULTI_FUNCTION;
  MV_ASSERT(pci_header_.header_type == PCI_HEADER_TYPE_NORMAL);

  /* Setup bars */
  for (uint8_t i = 0; i < VFIO_PCI_ROM_REGION_INDEX; i++) {
    auto &bar_region = regions_[i];
    if (!bar_region.size)
      continue;
    auto bar = pci_header_.bars[i];
    if (bar & PCI_BASE_ADDRESS_SPACE_IO) {
      AddPciBar(i, bar_region.size, kIoResourceTypePio);
    } else {
      /* 64bit bar is not supported yet */
      if (bar & PCI_BASE_ADDRESS_MEM_TYPE_64) {
        bar &= ~PCI_BASE_ADDRESS_MEM_TYPE_64;
      }
      AddPciBar(i, bar_region.size, kIoResourceTypeMmio);
    }
  }

  /* Setup capabilites */
  if (pci_header_.status & PCI_STATUS_CAP_LIST) {
    uint pos = pci_header_.capability & ~3;
    while (pos) {
      auto cap = (PciCapabilityHeader*)(pci_header_.data + pos);
      switch (cap->type)
      {
      case PCI_CAP_ID_MSI: {
        uint16_t control = *(uint16_t*)&cap[1];
        MV_ASSERT(!(control & PCI_MSI_FLAGS_MASKBIT));
        msi_config_.is_64bit = control & PCI_MSI_FLAGS_64BIT;
        msi_config_.offset = pos;
        msi_config_.is_msix = false;
        if (msi_config_.is_64bit) {
          msi_config_.msi64 = (MsiCapability64*)cap;
          msi_config_.length = sizeof(MsiCapability64);
          msi_config_.enabled = msi_config_.msi64->control & PCI_MSI_FLAGS_ENABLE;
        } else {
          msi_config_.msi32 = (MsiCapability32*)cap;
          msi_config_.length = sizeof(MsiCapability32);
          msi_config_.enabled = msi_config_.msi32->control & PCI_MSI_FLAGS_ENABLE;
        }
        break;
      }
      case PCI_CAP_ID_MSIX: {
        uint16_t control = *(uint16_t*)&cap[1];
        MV_ASSERT(!(control & PCI_MSIX_FLAGS_MASKALL));
        msi_config_.is_64bit = true;
        msi_config_.is_msix = true;
        msi_config_.offset = pos;
        msi_config_.msix = (MsiXCapability*)cap;
        msi_config_.length = sizeof(MsiXCapability);
        msi_config_.enabled = msi_config_.msix->control & PCI_MSIX_FLAGS_ENABLE;
        msi_config_.msix_table_size = (control & PCI_MSIX_FLAGS_QSIZE) + 1;
        msi_config_.msix_space_size = sizeof(msi_config_.msix_table);
        msi_config_.msix_space_offset = msi_config_.msix->table_offset & PCI_MSIX_TABLE_OFFSET;
        msi_config_.msix_bar = msi_config_.msix->table_offset & PCI_MSIX_TABLE_BIR;
        MV_ASSERT(msi_config_.msix_bar < sizeof(pci_header_.bars) / sizeof(uint32_t));
        break;
      }
      case PCI_CAP_ID_VNDR:
        /* ignore vendor specific data */
        break;
      case PCI_CAP_ID_EXP:
        is_pcie_ = true;
        MV_LOG("unsupported PCI Express, dump config space");
        DumpHex(pci_header_.data, PCI_DEVICE_CONFIG_SIZE);
        break;
      default:
        MV_LOG("unhandled capability=0x%x", cap->type);
        break;
      }
      pos = cap->next;
    }
  }
}

void VfioPci::SetupGfxPlane() {
  vfio_device_gfx_plane_info gfx_plane_info = {
    .argsz = sizeof(gfx_plane_info),
    .flags = VFIO_GFX_PLANE_TYPE_PROBE | VFIO_GFX_PLANE_TYPE_REGION
  };
  auto ret = ioctl(device_fd_, VFIO_DEVICE_QUERY_GFX_PLANE, &gfx_plane_info);
  if (ret == 0) {
    /* Register GFX plane */
  }
}

void VfioPci::MapDmaPages(const MemorySlot* slot) {
  vfio_iommu_type1_dma_map dma_map = {
    .argsz = sizeof(dma_map),
    .flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE,
    .vaddr = slot->hva,
    .iova = slot->begin,
    .size = slot->end - slot->begin
  };
  if (ioctl(container_fd_, VFIO_IOMMU_MAP_DMA, &dma_map) < 0) {
    MV_PANIC("failed to map vaddr=0x%lx size=0x%lx", dma_map.iova, dma_map.size);
  }
}

void VfioPci::UnmapDmaPages(const MemorySlot* slot) {
  vfio_iommu_type1_dma_unmap dma_ummap = {
    .argsz = sizeof(dma_ummap),
    .iova = slot->begin,
    .size = slot->end - slot->begin
  };
  if (ioctl(container_fd_, VFIO_IOMMU_UNMAP_DMA, &dma_ummap) < 0) {
    MV_PANIC("failed to unmap vaddr=0x%lx size=0x%lx", dma_ummap.iova, dma_ummap.size);
  }
}

void VfioPci::SetupDmaMaps() {
  auto mm = manager_->machine()->memory_manager();

  /* Map all current slots */
  for (auto slot : mm->GetMemoryFlatView()) {
    if (slot->type == kMemoryTypeRam) {
      MapDmaPages(slot);
    }
  }

  /* Add memory listener to keep DMA maps synchronized */
  memory_listener_ = mm->RegisterMemoryListener([this](auto slot, bool unmap) {
    if (slot->type == kMemoryTypeRam) {
      if (unmap) {
        UnmapDmaPages(slot);
      } else {
        MapDmaPages(slot);
      }
    }
  });
}

void VfioPci::SetupVfioGroup() {
  /* Get VFIO group id from device path */
  char path[1024];
  auto len = readlink((sysfs_path_ + "/iommu_group").c_str(), path, 1024);
  if (len < 0) {
    MV_PANIC("failed to read iommu_group");
  }
  path[len] = 0;
  if (sscanf(basename(path), "%d", &group_id_) != 1) {
    MV_PANIC("failed to get group id from %s", path);
  }
  
  /* Open group */
  sprintf(path, "/dev/vfio/%d", group_id_);
  group_fd_ = open(path, O_RDWR);
  if (group_fd_ < 0) {
    MV_PANIC("failed to open %s", path);
  }

  /* Check if it is OK */
  vfio_group_status status = { .argsz = sizeof(status) };
  MV_ASSERT(ioctl(group_fd_, VFIO_GROUP_GET_STATUS, &status) == 0);
  if (!(status.flags & VFIO_GROUP_FLAGS_VIABLE)) {
    MV_PANIC("VFIO group %d is not viable", group_id_);
  }
}

void VfioPci::SetupVfioContainer() {
  /* Create container */
  container_fd_ = open("/dev/vfio/vfio", O_RDWR);
  if (container_fd_ < 0) {
    MV_PANIC("failed to open /dev/vfio/vfio");
  }

  /* Here use type1 iommu */
  MV_ASSERT(ioctl(container_fd_, VFIO_GET_API_VERSION) == VFIO_API_VERSION);
  MV_ASSERT(ioctl(container_fd_, VFIO_CHECK_EXTENSION, VFIO_TYPE1v2_IOMMU));
  MV_ASSERT(ioctl(group_fd_, VFIO_GROUP_SET_CONTAINER, &container_fd_) == 0);
  MV_ASSERT(ioctl(container_fd_, VFIO_SET_IOMMU, VFIO_TYPE1v2_IOMMU) == 0);
  
  /* Get IOMMU type1 info */
  size_t argsz = sizeof(vfio_iommu_type1_info);
  auto info = (vfio_iommu_type1_info*)malloc(argsz);
  info->argsz = argsz;
  MV_ASSERT(ioctl(container_fd_, VFIO_IOMMU_GET_INFO, info) == 0);
  if (info->argsz != argsz) {
    info = (vfio_iommu_type1_info*)realloc(info, info->argsz);
    MV_ASSERT(ioctl(container_fd_, VFIO_IOMMU_GET_INFO, info) == 0);
  }

  /* Enumerate capabilities, currently migration capability */
  if (info->flags & VFIO_IOMMU_INFO_CAPS && info->cap_offset) {
    uint8_t* ptr = (uint8_t*)info;
    auto cap_header = (vfio_info_cap_header*)(ptr + info->cap_offset);
    while (true) {
      if (cap_header->id == VFIO_IOMMU_TYPE1_INFO_CAP_MIGRATION) {
        auto cap_migration = (vfio_iommu_type1_info_cap_migration*)cap_header;
        /* page size should support 4KB */
        MV_ASSERT(cap_migration->pgsize_bitmap & PAGE_SIZE);
      }
      if (cap_header->next) {
        cap_header = (vfio_info_cap_header*)(ptr + cap_header->next);
      } else {
        break;
      }
    }
  }
  free(info);
  
  /* Add iommu group to KVM */
  manager_->RegisterVfioGroup(group_fd_);
}

void VfioPci::SetupVfioDevice() {
  device_name_ = basename(sysfs_path_.c_str());
  device_fd_ = ioctl(group_fd_, VFIO_GROUP_GET_DEVICE_FD, device_name_.c_str());
 
  /* get device info */
  device_info_.argsz = sizeof(device_info_);
  MV_ASSERT(ioctl(device_fd_, VFIO_DEVICE_GET_INFO, &device_info_) == 0);
  
  MV_ASSERT(device_info_.flags & VFIO_DEVICE_FLAGS_RESET);
  MV_ASSERT(device_info_.flags & VFIO_DEVICE_FLAGS_PCI);
  MV_ASSERT(device_info_.num_regions > VFIO_PCI_CONFIG_REGION_INDEX);
  MV_ASSERT(device_info_.num_irqs > VFIO_PCI_MSIX_IRQ_INDEX);

  /* find vfio regions */
  bzero(&regions_, sizeof(regions_));
  for (uint index = VFIO_PCI_BAR0_REGION_INDEX; index < device_info_.num_regions; index++) {
    auto argsz = sizeof(vfio_region_info);
    auto region_info = (vfio_region_info*)malloc(argsz);
    bzero(region_info, argsz);
    region_info->argsz = argsz;
    region_info->index = index;

    MV_ASSERT(ioctl(device_fd_, VFIO_DEVICE_GET_REGION_INFO, region_info) == 0);
    if (region_info->argsz != argsz) {
      region_info = (vfio_region_info*)realloc(region_info, region_info->argsz);
      MV_ASSERT(ioctl(device_fd_, VFIO_DEVICE_GET_REGION_INFO, region_info) == 0);
    }
    if (!region_info->size) {
      free(region_info);
      continue;
    }
    if (region_info->index >= MAX_VFIO_REGIONS) {
      free(region_info);
      continue;
    }
    
    auto &region = regions_[region_info->index];
    region.index = region_info->index;
    region.flags = region_info->flags;
    region.offset = region_info->offset;
    region.size = region_info->size;

    if ((region_info->flags & VFIO_REGION_INFO_FLAG_CAPS) && region_info->cap_offset) {
      auto ptr = (uint8_t*)region_info;
      auto cap_header = (vfio_info_cap_header*)(ptr + region_info->cap_offset);
      while (true) {
        if (cap_header->id == VFIO_REGION_INFO_CAP_SPARSE_MMAP) {
          auto cap_mmap = (vfio_region_info_cap_sparse_mmap*)cap_header;
          for (uint j = 0; j < cap_mmap->nr_areas; j++) {
            region.mmap_areas.push_back({
              .offset = cap_mmap->areas[j].offset,
              .size = cap_mmap->areas[j].size
            });
          }
        } else if (cap_header->id == VFIO_REGION_INFO_CAP_TYPE) {
          auto cap_type = (vfio_region_info_cap_type*)cap_header;
          region.type = cap_type->type;
          region.subtype = cap_type->subtype;
        }
        if (cap_header->next) {
          cap_header = (vfio_info_cap_header*)(ptr + cap_header->next);
        } else {
          break;
        }
      }
    }
    if (debug_) {
      MV_LOG("region index=%u flags=0x%x size=0x%lx type=%d subtype=%d sparse=%lu",
        region.index, region.flags, region.size, region.type, region.subtype, region.mmap_areas.size());
    }
    free(region_info);
  }

  /* find vfio interrupts */
  for (auto &interrupt : interrupts_) {
    interrupt.event_fd = -1;
  }
  for (uint index = 0; index < device_info_.num_irqs; index++) {
    vfio_irq_info irq_info = { .argsz = sizeof(irq_info) };
    irq_info.index = index;
    auto ret = ioctl(device_fd_, VFIO_DEVICE_GET_IRQ_INFO, &irq_info);
    if (debug_) {
      MV_LOG("irq index=%d size=%u flags=%x count=%d ret=%d", index,
        irq_info.argsz, irq_info.flags, irq_info.count, ret);
    }
    /* FIXME: currently my mdev only uses one IRQ */
    if (index == VFIO_PCI_MSI_IRQ_INDEX) {
      MV_ASSERT(irq_info.flags & VFIO_IRQ_INFO_EVENTFD);
      MV_ASSERT(irq_info.count == 1 || irq_info.count == 3);
    } else if (index == VFIO_PCI_MSIX_IRQ_INDEX) {
      MV_ASSERT(irq_info.flags & VFIO_IRQ_INFO_EVENTFD);
      MV_ASSERT(irq_info.count == 1 || irq_info.count == 3);
    }
  }
}

void VfioPci::SetupMigraionInfo() {
  bzero(&migration_, sizeof(migration_));
  auto index = FindRegion(VFIO_REGION_TYPE_MIGRATION, VFIO_REGION_SUBTYPE_MIGRATION);
  if (index < 0) {
    return;
  }
  migration_.enabled = true;
  migration_.region = &regions_[index];
  MV_ASSERT(migration_.region->mmap_areas.size() == 1);

  auto machine = manager_->machine();
  state_change_listener_ = machine->RegisterStateChangeListener([=]() {
    if (machine->IsPaused()) {
      SetMigrationDeviceState(VFIO_DEVICE_STATE_STOP);
    } else {
      SetMigrationDeviceState(VFIO_DEVICE_STATE_RUNNING);
    }
  });
}

void VfioPci::MapBarRegion(uint8_t index) {
  auto &bar = pci_bars_[index];
  auto &region = regions_[index];
  int protect = 0;
  if (region.flags & VFIO_REGION_INFO_FLAG_READ)
    protect |= PROT_READ;
  if (region.flags & VFIO_REGION_INFO_FLAG_WRITE)
    protect |= PROT_WRITE;
  if (region.mmap_areas.empty()) {
    bar.host_memory = mmap(nullptr, region.size, protect, MAP_SHARED, device_fd_, region.offset);
    AddIoResource(kIoResourceTypeRam, bar.address, bar.size, "VFIO BAR RAM", bar.host_memory);
  } else {
    /* The MMIO region is overlapped by the mmap areas */
    AddIoResource(kIoResourceTypeMmio, bar.address, bar.size, "VFIO BAR MMIO");
    for (auto &area : region.mmap_areas) {
      area.mmap = mmap(nullptr, area.size, protect, MAP_SHARED, device_fd_, region.offset + area.offset);
      if (area.mmap == MAP_FAILED) {
        MV_PANIC("failed to map region %d, area offset=0x%lx size=0x%lx", index, area.offset, area.size);
      }
      AddIoResource(kIoResourceTypeRam, bar.address + area.offset, area.size, "VFIO BAR RAM", area.mmap);
    }
  }
}

void VfioPci::UnmapBarRegion(uint8_t index) {
  auto &bar = pci_bars_[index];
  auto &region = regions_[index];
  if (region.mmap_areas.empty()) {
    RemoveIoResource(kIoResourceTypeRam, bar.address);
    munmap(bar.host_memory, region.size);
  } else {
    for (auto &area : region.mmap_areas) {
      RemoveIoResource(kIoResourceTypeRam, bar.address + area.offset);
      munmap(area.mmap, area.size);
    }
    RemoveIoResource(kIoResourceTypeMmio, bar.address);
  }
}

bool VfioPci::ActivatePciBar(uint8_t index) {
  auto &bar = pci_bars_[index];
  auto &region = regions_[index];
  if (region.flags & VFIO_REGION_INFO_FLAG_MMAP) {
    MV_ASSERT(!bar.active && bar.type == kIoResourceTypeMmio);
    MapBarRegion(index);
    bar.active = true;
    return true;
  }
  return PciDevice::ActivatePciBar(index);
}

bool VfioPci::DeactivatePciBar(uint8_t index) {
  auto &bar = pci_bars_[index];
  auto &region = regions_[index];
  if (region.flags & VFIO_REGION_INFO_FLAG_MMAP) {
    MV_ASSERT(bar.active && bar.type == kIoResourceTypeMmio);
    UnmapBarRegion(index);
    bar.active = false;
    return true;
  }
  return PciDevice::DeactivatePciBar(index);
}

ssize_t VfioPci::ReadRegion(uint8_t index, uint64_t offset, void* data, uint32_t length) {
  MV_ASSERT(index < MAX_VFIO_REGIONS);
  auto &region = regions_[index];
  return pread(device_fd_, data, length, region.offset + offset);
}

ssize_t VfioPci::WriteRegion(uint8_t index, uint64_t offset, void* data, uint32_t length) {
  MV_ASSERT(index < MAX_VFIO_REGIONS);
  auto &region = regions_[index];
  return pwrite(device_fd_, data, length, region.offset + offset);
}

int VfioPci::FindRegion(uint32_t type, uint32_t subtype) {
  for (uint8_t i = 0; i < regions_.size(); i++) {
    if (regions_[i].type == type && regions_[i].subtype == subtype) {
      return i;
    }
  }
  return -ENOENT;
}

void VfioPci::Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  for (int i = 0; i < PCI_BAR_NUMS; i++) {
    if (pci_bars_[i].address == resource->base) {
      WriteRegion(i, offset, data, size);
      if (msi_config_.is_msix && i == msi_config_.msix_bar) {
        // we need to continue to set msix table
        break;
      } 

      // just return
      return;
    }
  }
  PciDevice::Write(resource, offset, data, size);
}

void VfioPci::Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  for (int i = 0; i < PCI_BAR_NUMS; i++) {
    if (pci_bars_[i].address == resource->base) {
      ReadRegion(i, offset, data, size);
      return;
    }
  }
  PciDevice::Read(resource, offset, data, size);
}

void VfioPci::EnableIRQ(uint32_t index, uint32_t vector, int *fds, uint8_t count) {
  uint8_t buffer[sizeof(vfio_irq_set) + sizeof(int) * count];
  auto irq_set = (vfio_irq_set *)buffer;
  irq_set->argsz = sizeof(vfio_irq_set) + sizeof(int) * count;
  irq_set->flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
  irq_set->index = index;
  irq_set->start = vector;
  irq_set->count = count;
  memcpy(irq_set->data, fds, sizeof(int) * count);
  MV_ASSERT(ioctl(device_fd_, VFIO_DEVICE_SET_IRQS, irq_set) == 0);
}

void VfioPci::UpdateMsiRoutes() {
  uint nr_vectors = 0;
  uint16_t control = 0;
  if (msi_config_.is_msix) {
    control = msi_config_.msix->control;
    nr_vectors = msi_config_.msix_table_size;
    msi_config_.enabled = control & PCI_MSIX_FLAGS_ENABLE;
  } else {
    control =  msi_config_.is_64bit ? msi_config_.msi64->control : msi_config_.msi32->control;
    nr_vectors = 1 << ((control & PCI_MSI_FLAGS_QSIZE) >> 4);
    msi_config_.enabled = control & PCI_MSI_FLAGS_ENABLE;
  }

  if (msi_config_.enabled) {
    int fds[nr_vectors];
    memset(fds, -1, sizeof(fds));
    EnableIRQ(msi_config_.is_msix ? VFIO_PCI_MSIX_IRQ_INDEX : VFIO_PCI_MSI_IRQ_INDEX, 0, fds, nr_vectors);
  }

  for (uint vector = 0; vector < nr_vectors; vector++) {
    auto &interrupt = interrupts_[vector];
    if (interrupt.event_fd == -1) {
      interrupt.event_fd = eventfd(0, 0);
    }
  
    uint64_t msi_address;
    uint32_t msi_data;
    if (msi_config_.is_msix) {
      msi_address = ((uint64_t)msi_config_.msix_table[vector].message.address_hi << 32) | msi_config_.msix_table[vector].message.address_lo;
      msi_data = msi_config_.msix_table[vector].message.data;
    } else if (msi_config_.is_64bit) {
      msi_address = ((uint64_t)msi_config_.msi64->address1 << 32) | msi_config_.msi64->address0;
      msi_data = msi_config_.msi64->data;
    } else {
      msi_address = msi_config_.msi32->address;
      msi_data = msi_config_.msi32->data;
    }

    if (debug_) {
      MV_LOG("msi_address=0x%lx msi_data=%d vector=%d", msi_address, msi_data, vector);
    }

    if (msi_config_.enabled && msi_address) {
      if (interrupt.gsi == -1) {
        interrupt.gsi = manager_->AddMsiRoute(msi_address, msi_data, interrupt.event_fd);

        // set irq with KVM_IRQFD
        EnableIRQ(msi_config_.is_msix ? VFIO_PCI_MSIX_IRQ_INDEX : VFIO_PCI_MSI_IRQ_INDEX, vector, &interrupt.event_fd, 1);
      } else {
        manager_->UpdateMsiRoute(interrupt.gsi, msi_address, msi_data, interrupt.event_fd);
      }
    } else {
      if (interrupt.gsi != -1) {
        manager_->UpdateMsiRoute(interrupt.gsi, 0, 0, interrupt.event_fd);
        interrupt.gsi = -1;
      }
    }
  }
}

void VfioPci::WritePciCommand(uint16_t new_command) {
  int toggle_io = (pci_header_.command ^ new_command) & PCI_COMMAND_IO;
  int toggle_mem = (pci_header_.command ^ new_command) & PCI_COMMAND_MEMORY;

  for (int i = 0; i < PCI_BAR_NUMS; i++) {
    auto &bar = pci_bars_[i];
    if (!bar.address || bar.active) {
      continue;
    }

    // DeactivatePciBar only can be called in vfio-pci Disconnect routine
    if (toggle_io && bar.type == kIoResourceTypePio && (new_command & PCI_COMMAND_IO)) {
      ActivatePciBar(i);
    } else if (toggle_mem && bar.type != kIoResourceTypePio && (new_command & PCI_COMMAND_MEMORY)) {
      ActivatePciBar(i);
    }
  }
  pci_header_.command = new_command;
}

void VfioPci::WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
  MV_ASSERT(length <= 4);
  MV_ASSERT(offset + length <= PCIE_DEVICE_CONFIG_SIZE);
  auto &config_region = regions_[VFIO_PCI_CONFIG_REGION_INDEX];

  /* write the VFIO device, check if msi */
  auto ret = pwrite(device_fd_, data, length, config_region.offset + offset);
  if (ret != (ssize_t)length) {
    MV_PANIC("failed to write VFIO device, offset=0x%x length=0x%x data=0x%x ret=%d",
      offset, length, *(uint32_t*)data, ret);
  }

  /* the default bahavior detects BAR activate/deactivate */
  PciDevice::WritePciConfigSpace(offset, data, length);

  /* update interrupts if needed */
  if (ranges_overlap(offset, length, msi_config_.offset + PCI_MSI_FLAGS, 1)) {
    UpdateMsiRoutes();
  }
}

void VfioPci::ReadPciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
  MV_ASSERT(offset + length <= PCIE_DEVICE_CONFIG_SIZE);

  /* read from VFIO device */
  auto &config_region = regions_[VFIO_PCI_CONFIG_REGION_INDEX];
  auto ret = pread(device_fd_, pci_header_.data + offset, length, config_region.offset + offset);
  MV_ASSERT(ret == (ssize_t)length);

  PciDevice::ReadPciConfigSpace(offset, data, length);
}

void VfioPci::SetMigrationDeviceState(uint32_t device_state) {
  MV_ASSERT(migration_.enabled);
  pwrite(device_fd_, &device_state, sizeof(device_state), migration_.region->offset);
}

bool VfioPci::SaveState(MigrationWriter* writer) {
  if (!migration_.enabled) {
    MV_LOG("%s:%s blocked migration", name_, device_name_.c_str());
    return false;
  }

  /* Map buffer for saving */
  auto& area = migration_.region->mmap_areas.front();
  void* buffer = mmap(nullptr, area.size, PROT_READ | PROT_WRITE, MAP_SHARED,
    device_fd_, migration_.region->offset + area.offset);
  if (buffer == MAP_FAILED) {
    MV_PANIC("failed to map area offset=0x%lx size=0x%lx", area.offset, area.size);
  }

  SetMigrationDeviceState(VFIO_DEVICE_STATE_SAVING);
  int fd = writer->BeginWrite("DATA");
  bool success = false;

  while (true) {
    uint64_t pending_bytes;
    auto ret = pread(device_fd_, &pending_bytes, sizeof(pending_bytes),
      migration_.region->offset + offsetof(vfio_device_migration_info, pending_bytes));
    if (ret < 0)
      break;

    if (pending_bytes == 0) {
      success = true;
      break;
    } else if (pending_bytes > area.size) {
      pending_bytes = area.size;
    }

    uint64_t data_offset = 0;
    pread(device_fd_, &data_offset, sizeof(data_offset),
      migration_.region->offset + offsetof(vfio_device_migration_info, data_offset));
    uint64_t data_size = 0;
    pread(device_fd_, &data_size, sizeof(data_size),
      migration_.region->offset + offsetof(vfio_device_migration_info, data_size));
    MV_ASSERT(data_offset == area.offset);
    MV_ASSERT(data_size <= area.size);

    /* nVidia vGPU migration fixes */
    if (data_size == 0)
      data_size = pending_bytes;
    
    ret = write(fd, buffer, data_size);
    MV_ASSERT(ret == (ssize_t)data_size);
  }

  writer->EndWrite("DATA");
  SetMigrationDeviceState(VFIO_DEVICE_STATE_STOP);

  munmap(buffer, area.size);

  return success && PciDevice::SaveState(writer);
}

bool VfioPci::LoadState(MigrationReader* reader) {
  if (!PciDevice::LoadState(reader)) {
    return false;
  }

  /* Restore the PCI config space to VFIO device */
  auto &config_region = regions_[VFIO_PCI_CONFIG_REGION_INDEX];
  for (uint i = 0; i < pci_config_size(); i += 2) {
    pwrite(device_fd_, &pci_header_.data[i], 2, config_region.offset + i);
  }

  /* Update MSI routes */
  UpdateMsiRoutes();

  /* Map buffer for restoring */
  auto& area = migration_.region->mmap_areas.front();
  void* buffer = mmap(nullptr, area.size, PROT_READ | PROT_WRITE, MAP_SHARED,
    device_fd_, migration_.region->offset + area.offset);
  if (buffer == MAP_FAILED) {
    MV_PANIC("failed to map area offset=0x%lx size=0x%lx", area.offset, area.size);
  }

  SetMigrationDeviceState(VFIO_DEVICE_STATE_RESUMING);
  int fd = reader->BeginRead("DATA");

  while (true) {
    uint64_t data_offset = 0;
    pread(device_fd_, &data_offset, sizeof(data_offset),
      migration_.region->offset + offsetof(vfio_device_migration_info, data_offset));
    MV_ASSERT(data_offset == area.offset);
    
    auto ret = read(fd, buffer, area.size);
    if (ret > 0) {
      pwrite(device_fd_, &ret, sizeof(ret),
        migration_.region->offset + offsetof(vfio_device_migration_info, data_size));
    }
    if (ret < (ssize_t)area.size)
      break;
  }

  reader->EndRead("DATA");
  SetMigrationDeviceState(VFIO_DEVICE_STATE_STOP);
  
  munmap(buffer, area.size);
  return true;
}

DECLARE_DEVICE(VfioPci);
