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
#include "linuz/vfio.h"

VfioPci::VfioPci() {
  for (auto &interrupt : interrupts_) {
    interrupt.event_fd = -1;
    interrupt.gsi = -1;
  }
  bzero(&regions_, sizeof(regions_));
  bzero(&migration_, sizeof(migration_));
}

VfioPci::~VfioPci() {

}

void VfioPci::Connect() {
  vfio_manager_ = manager_->machine()->vfio_manager();
  if (!has_key("sysfs")) {
    MV_PANIC("Please specify 'sysfs' for vfio-pci, like '/sys/bus/mdev/devices/xxx' or '/sys/bus/pci/devices/xxx'");
  }
  sysfs_path_ = std::get<std::string>(key_values_["sysfs"]);
  device_name_ = basename(sysfs_path_.c_str());

  /* Check if host pci address 0000:01:00.[function] */
  if (sysfs_path_.find("pci") != std::string::npos) {
    // separate by '.' to get device and function
    auto dot_pos = device_name_.find('.');
    if (dot_pos == std::string::npos) {
      MV_PANIC("invalid device name %s (should be like 0000:01:00.0)", device_name_.c_str());
    }
    // if function is not 0, we need to locate the multi-function device
    auto function = device_name_.substr(dot_pos + 1);
    if (function != "0") {
      auto master_device = device_name_.substr(0, dot_pos) + ".0";
      auto objects = manager_->machine()->LookupObjects([=](auto obj) {
        if (this != obj && obj->has_key("sysfs")) {
          auto sysfs = std::get<std::string>((*obj)["sysfs"]);
          return sysfs.find(master_device) != std::string::npos;
        }
        return false;
      });
      if (objects.empty()) {
        MV_WARN("multi-function device %s not found", master_device.c_str());
      } else {
        auto master = dynamic_cast<VfioPci*>(objects[0]);
        if (master->slot_ == 0xFF) {
          MV_PANIC("multi-function device %s not registered before %s", master_device.c_str(), device_name_.c_str());
        }
        this->slot_ = master->slot_;
        this->function_ = std::stoi(function);
      }
    } else {
      multi_function_ = true;
    }
  }

  PciDevice::Connect();

  SetupVfioDevice();
  SetupPciConfiguration();
  SetupGfxPlane();
  SetupMigrationV1Info();
  SetupMigrationV2Info();

  if (migration_.enabled && debug_) {
    MV_LOG("%s migration enabled version=%d", device_name_.c_str(), migration_.version);
  }
}

void VfioPci::Disconnect() {
  auto machine = manager_->machine();
  if (state_change_listener_) {
    machine->UnregisterStateChangeListener(&state_change_listener_);
  }

  DisableAllInterrupts();
  manager_->machine()->vfio_manager()->DetachDevice(device_fd_);
  safe_close(&device_fd_);

  /* unmap vfio regions */
  for (auto &region : regions_) {
    if (region.mmap) {
      munmap(region.mmap, region.size);
    }
    for (auto &area : region.mmap_areas) {
      if (area.mmap) {
        munmap(area.mmap, area.size);
      }
    }
  }
  if (pci_rom_.data) {
    free(pci_rom_.data);
    pci_rom_.data = nullptr;
  }
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

  DisableAllInterrupts();
  PciDevice::Reset();
}

void VfioPci::DisableAllInterrupts() {
  /* deactivate current irq */
  if (active_irq_index_ != -1) {
    if (debug_) {
      MV_LOG("%s deactivate irq index=%d", device_name_.c_str(), active_irq_index_);
    }
    vfio_irq_set irq_set = {
      .argsz = sizeof(vfio_irq_set),
      .flags = VFIO_IRQ_SET_DATA_NONE | VFIO_IRQ_SET_ACTION_TRIGGER,
      .index = (uint)active_irq_index_,
      .start = 0,
      .count = 0
    };
    MV_ASSERT(ioctl(device_fd_, VFIO_DEVICE_SET_IRQS, &irq_set) == 0);
  }

  /* remove notifiers */
  if (active_irq_index_ == VFIO_PCI_MSI_IRQ_INDEX || active_irq_index_ == VFIO_PCI_MSIX_IRQ_INDEX) {
    for (auto &interrupt : interrupts_) {
      if (interrupt.gsi > 0) {
        manager_->RemoveMsiNotifier(interrupt.gsi, interrupt.event_fd);
        interrupt.gsi = -1;
      }
      safe_close(&interrupt.event_fd);
    }
  } else if (active_irq_index_ == VFIO_PCI_INTX_IRQ_INDEX) {
    if (intx_trigger_fd_ != -1) {
      manager_->SetPciIrqNotifier(this, intx_trigger_fd_, -1, false);
      safe_close(&intx_trigger_fd_);
      safe_close(&intx_unmask_fd_);
    }
  }
  active_irq_index_ = -1;
}

void VfioPci::EnableIntxInterrupt() {
  if (!pci_header_.irq_pin) {
    return;
  }
  if (debug_) {
    MV_LOG("%s enable INTx interrupt", device_name_.c_str());
  }

  if (intx_trigger_fd_ == -1) {
    intx_trigger_fd_ = eventfd(0, 0);
    intx_unmask_fd_ = eventfd(0, 0);
    SetInterruptEventFds(VFIO_PCI_INTX_IRQ_INDEX, 0, VFIO_IRQ_SET_ACTION_TRIGGER, &intx_trigger_fd_, 1);
    SetInterruptEventFds(VFIO_PCI_INTX_IRQ_INDEX, 0, VFIO_IRQ_SET_ACTION_UNMASK, &intx_unmask_fd_, 1);
    manager_->SetPciIrqNotifier(this, intx_trigger_fd_, intx_unmask_fd_, true);
    SetInterruptMasked(VFIO_PCI_INTX_IRQ_INDEX, false);
  }
  active_irq_index_ = VFIO_PCI_INTX_IRQ_INDEX;
}

void VfioPci::SetupVfioDevice() {
  device_fd_ = vfio_manager_->AttachDevice(sysfs_path_);
 
  /* get device info */
  bzero(&device_info_, sizeof(device_info_));
  device_info_.argsz = sizeof(device_info_);
  MV_ASSERT(ioctl(device_fd_, VFIO_DEVICE_GET_INFO, &device_info_) == 0);
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

    int ret = ioctl(device_fd_, VFIO_DEVICE_GET_REGION_INFO, region_info);
    if (ret < 0) {
      free(region_info);
      continue;
    }
    if (region_info->argsz != argsz) {
      region_info = (vfio_region_info*)realloc(region_info, region_info->argsz);
      MV_ASSERT(ioctl(device_fd_, VFIO_DEVICE_GET_REGION_INFO, region_info) == 0);
    }
    if (!region_info->size) {
      free(region_info);
      continue;
    }
    if (region_info->index >= MAX_VFIO_REGIONS) {
      MV_WARN("region index %d exceeds max regions %d", region_info->index, MAX_VFIO_REGIONS);
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
              .size = cap_mmap->areas[j].size,
              .mmap = nullptr
            });
          }
        } else if (cap_header->id == VFIO_REGION_INFO_CAP_TYPE) {
          auto cap_type = (vfio_region_info_cap_type*)cap_header;
          region.type = cap_type->type;
          region.subtype = cap_type->subtype;
        } else if (cap_header->id == VFIO_REGION_INFO_CAP_MSIX_MAPPABLE) {
          msix_table_mapped_ = true;
        } else {
          MV_WARN("unhandled region cap id=%d", cap_header->id);
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

    /* map regions */
    if (region.flags & VFIO_REGION_INFO_FLAG_MMAP) {
      int protect = 0;
      if (region.flags & VFIO_REGION_INFO_FLAG_READ)
        protect |= PROT_READ;
      if (region.flags & VFIO_REGION_INFO_FLAG_WRITE)
        protect |= PROT_WRITE;
      if (region.mmap_areas.empty()) {
        region.mmap = mmap(nullptr, region.size, protect, MAP_SHARED, device_fd_, region.offset);
      } else {
        for (auto &area : region.mmap_areas) {
          area.mmap = mmap(nullptr, area.size, protect, MAP_SHARED, device_fd_, region.offset + area.offset);
          if (area.mmap == MAP_FAILED) {
            MV_PANIC("failed to map region %d, area offset=0x%lx size=0x%lx", index, area.offset, area.size);
          }
        }
      }
    }
  }

  /* find vfio interrupts */
  for (auto &interrupt : interrupts_) {
    interrupt.event_fd = -1;
  }
  for (uint index = 0; index < device_info_.num_irqs; index++) {
    vfio_irq_info irq_info;
    bzero(&irq_info, sizeof(irq_info));
    irq_info.argsz = sizeof(irq_info);
    irq_info.index = index;
    auto ret = ioctl(device_fd_, VFIO_DEVICE_GET_IRQ_INFO, &irq_info);
    if (irq_info.count == 0) {
      continue;
    }
    if (debug_) {
      MV_LOG("irq index=%d size=%u flags=%x count=%d ret=%d", index,
        irq_info.argsz, irq_info.flags, irq_info.count, ret);
    }

    /* check if interrupt flags satisfy our requirements */
    if (index == VFIO_PCI_INTX_IRQ_INDEX) {
      MV_ASSERT(irq_info.flags & VFIO_IRQ_INFO_EVENTFD);
      MV_ASSERT(irq_info.flags & VFIO_IRQ_INFO_MASKABLE);
      MV_ASSERT(irq_info.flags & VFIO_IRQ_INFO_AUTOMASKED);
    } else if (index == VFIO_PCI_MSI_IRQ_INDEX || index == VFIO_PCI_MSIX_IRQ_INDEX) {
      MV_ASSERT(irq_info.flags & VFIO_IRQ_INFO_EVENTFD);
      MV_ASSERT(irq_info.flags & VFIO_IRQ_INFO_NORESIZE);
    }
  }
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

  /* Setup bars */
  for (uint8_t i = 0; i < VFIO_PCI_ROM_REGION_INDEX; i++) {
    auto &bar_region = regions_[i];
    if (!bar_region.size)
      continue;
    auto bar = pci_header_.bars[i];
    if (bar & PCI_BASE_ADDRESS_SPACE_IO) {
      SetupPciBar(i, bar_region.size, kIoResourceTypePio);
    } else {
      if (bar & PCI_BASE_ADDRESS_MEM_TYPE_64) {
        SetupPciBar64(i, bar_region.size, kIoResourceTypeMmio);
      } else {
        SetupPciBar(i, bar_region.size, kIoResourceTypeMmio);
      }
    }
  }

  /* Setup ROM bar */
  auto &rom_region = regions_[VFIO_PCI_ROM_REGION_INDEX];
  if (rom_region.size) {
    if (pci_rom_.data) {
      free(pci_rom_.data);
    }
    pci_rom_.data = valloc(rom_region.size);
    pci_rom_.size = rom_region.size;
    ret = pread(device_fd_, pci_rom_.data, rom_region.size, rom_region.offset);
    MV_ASSERT(ret == (int)rom_region.size);
  }

  /* Setup capabilites */
  if (pci_header_.status & PCI_STATUS_CAP_LIST) {
    PciCapabilityHeader* last_cap = nullptr;
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
        break;
      }
      case PCI_CAP_ID_VNDR:
        /* ignore vendor specific data */
        break;
      case PCI_CAP_ID_PM:
        /* ignore power management */
        break;
      case PCI_CAP_ID_EXP:
        is_pcie_ = true;
        /* skip pcie capability */
        if (last_cap) {
          last_cap->next = cap->next;
        } else {
          pci_header_.capability = cap->next;
        }
        break;
      default:
        if (debug_) {
          MV_WARN("unhandled capability=0x%x", cap->type);
        }
        break;
      }
      last_cap = cap;
      pos = cap->next;
    }
  }
}

void VfioPci::SetupGfxPlane() {
  vfio_device_gfx_plane_info gfx_plane_info;
  bzero(&gfx_plane_info, sizeof(gfx_plane_info));
  gfx_plane_info.argsz = sizeof(gfx_plane_info);
  gfx_plane_info.flags = VFIO_GFX_PLANE_TYPE_PROBE | VFIO_GFX_PLANE_TYPE_REGION;

  auto ret = ioctl(device_fd_, VFIO_DEVICE_QUERY_GFX_PLANE, &gfx_plane_info);
  if (ret == 0) {
    /* Register GFX plane */
  }
}

void VfioPci::SetupMigrationV1Info() {
  bzero(&migration_, sizeof(migration_));
  auto it = std::find_if(regions_.begin(), regions_.end(), [](auto &region) {
    return region.type == VFIO_REGION_TYPE_MIGRATION_DEPRECATED &&
            region.subtype == VFIO_REGION_SUBTYPE_MIGRATION_DEPRECATED; 
  });
  if (it == regions_.end()) {
    return;
  }
  migration_.enabled = true;
  migration_.region = it;
  migration_.version = 1;
  MV_ASSERT(migration_.region->mmap_areas.size() == 1);

  auto machine = manager_->machine();
  state_change_listener_ = machine->RegisterStateChangeListener([=]() {
    if (machine->IsPaused()) {
      SetMigrationDeviceState(VFIO_DEVICE_STATE_V1_STOP);
    } else {
      SetMigrationDeviceState(VFIO_DEVICE_STATE_V1_RUNNING);
    }
  });
}

void VfioPci::SetupMigrationV2Info() {
  uint64_t buf[DIV_ROUND_UP(sizeof(struct vfio_device_feature) +
                            sizeof(struct vfio_device_feature_migration),
                            sizeof(uint64_t))] = {};
  auto feature = (struct vfio_device_feature *)buf;
  auto mig = (struct vfio_device_feature_migration *)feature->data;

  feature->argsz = sizeof(buf);
  feature->flags = VFIO_DEVICE_FEATURE_GET | VFIO_DEVICE_FEATURE_MIGRATION;
  if (ioctl(device_fd_, VFIO_DEVICE_FEATURE, feature)) {
    return;
  }
  
  if (mig->flags & VFIO_MIGRATION_STOP_COPY) {
    migration_.enabled = true;
    migration_.version = 2;
  }

  auto machine = manager_->machine();
  state_change_listener_ = machine->RegisterStateChangeListener([=]() {
    if (machine->IsPaused()) {
      SetMigrationDeviceState(VFIO_DEVICE_STATE_STOP);
    } else {
      SetMigrationDeviceState(VFIO_DEVICE_STATE_RUNNING);
    }
  });
}

void VfioPci::WritePciCommand(uint16_t command) {
  uint16_t old_comand = pci_header_.command;
  PciDevice::WritePciCommand(command);

  if (active_irq_index_ > VFIO_PCI_INTX_IRQ_INDEX) {
    return;
  }
  if ((old_comand ^ command) & PCI_COMMAND_INTX_DISABLE) {
    if (!(command & PCI_COMMAND_INTX_DISABLE)) {
      EnableIntxInterrupt();
    } else {
      DisableAllInterrupts();
    }
  }
}

bool VfioPci::ActivatePciBar(uint index) {
  auto &bar = pci_bars_[index];
  auto &region = regions_[index];
  if (region.flags & VFIO_REGION_INFO_FLAG_MMAP) {
    MV_ASSERT(!bar.active);
    MV_ASSERT(bar.type == kIoResourceTypeMmio);
    if (region.mmap_areas.empty()) {
      AddIoResource(kIoResourceTypeRam, bar.address64, bar.size, "VFIO BAR RAM", region.mmap);
    } else {
      /* The MMIO region is overlapped by the mmap areas */
      AddIoResource(kIoResourceTypeMmio, bar.address64, bar.size, "VFIO BAR MMIO");
      for (auto &area : region.mmap_areas) {
        AddIoResource(kIoResourceTypeRam, bar.address64 + area.offset, area.size, "VFIO BAR RAM", area.mmap);
      }
    }
    bar.active = true;
    return true;
  }
  return PciDevice::ActivatePciBar(index);
}

bool VfioPci::DeactivatePciBar(uint index) {
  auto &bar = pci_bars_[index];
  auto &region = regions_[index];
  if (region.flags & VFIO_REGION_INFO_FLAG_MMAP) {
    if (bar.active) {
      if (region.mmap_areas.empty()) {
        RemoveIoResource(kIoResourceTypeRam, bar.address64);
      } else {
        for (auto &area : region.mmap_areas) {
          RemoveIoResource(kIoResourceTypeRam, bar.address64 + area.offset);
        }
        RemoveIoResource(kIoResourceTypeMmio, bar.address64);
      }
      bar.active = false;
    }
    return true;
  }
  return PciDevice::DeactivatePciBar(index);
}

void VfioPci::Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  for (int i = 0; i < PCI_BAR_NUMS; i++) {
    if (pci_bars_[i].address == resource->base) {
      auto ret = pwrite(device_fd_, data, size, regions_[i].offset + offset);
      if (ret != (ssize_t)size) {
        MV_PANIC("failed to write VFIO device, bar=%d offset=0x%x size=0x%x data=0x%x", i, offset, size, *(uint32_t*)data);
      }

      if (msi_config_.is_msix && i == msi_config_.msix_bar) {
        // we need to continue to set msix table
        break;
      }
      return;
    }
  }
  PciDevice::Write(resource, offset, data, size);
}

void VfioPci::Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  for (int i = 0; i < PCI_BAR_NUMS; i++) {
    if (pci_bars_[i].address == resource->base) {
      auto ret = pread(device_fd_, data, size, regions_[i].offset + offset);
      if (ret != (ssize_t)size) {
        MV_PANIC("failed to read VFIO device, bar=%d offset=0x%x size=0x%x", i, offset, size);
      }
      return;
    }
  }
  PciDevice::Read(resource, offset, data, size);
}

void VfioPci::SetInterruptEventFds(uint32_t index, uint32_t vector, int action, int *fds, uint8_t count) {
  uint8_t buffer[sizeof(vfio_irq_set) + sizeof(int) * count];
  auto irq_set = (vfio_irq_set *)buffer;
  irq_set->argsz = sizeof(vfio_irq_set) + sizeof(int) * count;
  irq_set->flags = VFIO_IRQ_SET_DATA_EVENTFD | action;
  irq_set->index = index;
  irq_set->start = vector;
  irq_set->count = count;
  if (fds == nullptr) {
    irq_set->flags = VFIO_IRQ_SET_DATA_NONE | action;
  }
  memcpy(irq_set->data, fds, sizeof(int) * count);
  auto ret = ioctl(device_fd_, VFIO_DEVICE_SET_IRQS, irq_set);
  if (debug_) {
    MV_LOG("%s set irq event fds, index=%d vector=%d action=%d count=%d ret=%d", device_name_.c_str(),
      index, vector, action, count, ret);
  }
}

void VfioPci::SetInterruptMasked(uint32_t index, bool masked) {
  vfio_irq_set irq_set = {
    .argsz = sizeof(vfio_irq_set),
    .flags = VFIO_IRQ_SET_DATA_NONE,
    .index = index,
    .start = 0,
    .count = 1
  };
  if (masked) {
    irq_set.flags |= VFIO_IRQ_SET_ACTION_MASK;
  } else {
    irq_set.flags |= VFIO_IRQ_SET_ACTION_UNMASK;
  }
  auto ret = ioctl(device_fd_, VFIO_DEVICE_SET_IRQS, &irq_set);
  if (debug_) {
    MV_LOG("%s set irq masked, index=%d masked=%d ret=%d", device_name_.c_str(), index, masked, ret);
  }
}

void VfioPci::UpdateMsiRoutes() {
  DisableAllInterrupts();

  if (!msi_config_.enabled) {
    return;
  }

  active_irq_index_ = msi_config_.is_msix ? VFIO_PCI_MSIX_IRQ_INDEX : VFIO_PCI_MSI_IRQ_INDEX;
  uint nr_vectors = 0;
  if (msi_config_.is_msix) {
    nr_vectors = msi_config_.msix_table_size;
  } else {
    uint16_t control =  msi_config_.is_64bit ? msi_config_.msi64->control : msi_config_.msi32->control;
    nr_vectors = 1 << ((control & PCI_MSI_FLAGS_QSIZE) >> 4);
  }

  int fds[nr_vectors];
  for (uint i = 0; i < nr_vectors; i++) {
    fds[i] = -1;
  }

  for (uint vector = 0; vector < nr_vectors; vector++) {
    auto &interrupt = interrupts_[vector];
    MV_ASSERT(interrupt.gsi == -1 && interrupt.event_fd == -1);
    interrupt.event_fd = eventfd(0, 0);
    fds[vector] = interrupt.event_fd;
  
    uint64_t msi_address;
    uint32_t msi_data;
    if (msi_config_.is_msix) {
      auto entries = msi_config_.msix_table;
      if (msix_table_mapped_) {
        /* If MSIX table is mmaped, no MMIO write is handled and the table is not updated,
         * so we need to read the table from the mmaped region */
        entries = (MsiXTableEntry*)((uint8_t*)regions_[msi_config_.msix_bar].mmap + msi_config_.msix_space_offset);
      }
      msi_address = ((uint64_t)entries[vector].message.address_hi << 32) | entries[vector].message.address_lo;
      msi_data = entries[vector].message.data;
    } else if (msi_config_.is_64bit) {
      msi_address = ((uint64_t)msi_config_.msi64->address1 << 32) | msi_config_.msi64->address0;
      msi_data = msi_config_.msi64->data;
    } else {
      msi_address = msi_config_.msi32->address;
      msi_data = msi_config_.msi32->data;
    }

    if (debug_) {
      MV_LOG("%s set msi vector=%d address=0x%lx data=0x%x", device_name_.c_str(), vector, msi_address, msi_data);
    }
    interrupt.gsi = manager_->AddMsiNotifier(msi_address, msi_data, interrupt.event_fd);
  }
  SetInterruptEventFds(active_irq_index_, 0, VFIO_IRQ_SET_ACTION_TRIGGER, fds, nr_vectors);
}

void VfioPci::WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
  MV_ASSERT(length <= 4);
  MV_ASSERT(offset + length <= pci_config_size());

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
  MV_ASSERT(offset + length <= pci_config_size());

  /* We want to emulate the capability fields (skip PCIe cap)
   * Disable PCI-e capability for NVIDIA GPUs to avoid error code 10
   */
  if (offset + length <= 0x40) {
    auto &config_region = regions_[VFIO_PCI_CONFIG_REGION_INDEX];
    auto ret = pread(device_fd_, pci_header_.data + offset, length, config_region.offset + offset);
    MV_ASSERT(ret == (ssize_t)length);
  }

  if (multi_function_) {
    pci_header_.header_type = 0x80;
  } else {
    pci_header_.header_type = 0;
  }

  PciDevice::ReadPciConfigSpace(offset, data, length);
}

void VfioPci::SetMigrationDeviceState(uint32_t device_state) {
  MV_ASSERT(migration_.enabled);
  if (migration_.version == 1) {
    pwrite(device_fd_, &device_state, sizeof(device_state), migration_.region->offset);
  } else {
    uint64_t buf[DIV_ROUND_UP(sizeof(struct vfio_device_feature) +
                              sizeof(struct vfio_device_feature_mig_state),
                              sizeof(uint64_t))] = {};
    auto feature = (struct vfio_device_feature *)buf;
    auto mig_state = (struct vfio_device_feature_mig_state *)feature->data;
    feature->argsz = sizeof(buf);
    feature->flags = VFIO_DEVICE_FEATURE_SET | VFIO_DEVICE_FEATURE_MIG_DEVICE_STATE;
    mig_state->device_state = device_state;
    if (ioctl(device_fd_, VFIO_DEVICE_FEATURE, feature)) {
      MV_ERROR("failed to set migration device state %d", device_state);
    }
    if (mig_state->data_fd != -1) {
      migration_.data_fd = mig_state->data_fd;
    }
  }
}

bool VfioPci::SaveDeviceStateV1(MigrationWriter* writer) {
  /* buffer for saving */
  auto& area = migration_.region->mmap_areas.front();

  SetMigrationDeviceState(VFIO_DEVICE_STATE_V1_SAVING);
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
    
    if (dynamic_cast<MigrationNetworkWriter*>(writer)) {
      if (writer->WriteRaw("VFIO_DATA", area.mmap, data_size)) {
        ret = data_size;
      } else {
        MV_ERROR("send vfio data error");
        break;
      }
    } else {
      ret = write(fd, area.mmap, data_size);
    }
    MV_ASSERT(ret == (ssize_t)data_size);
  }

  writer->EndWrite("DATA");
  SetMigrationDeviceState(VFIO_DEVICE_STATE_V1_STOP);
  return success;
}

bool VfioPci::SaveDeviceStateV2(MigrationWriter* writer) {
  SetMigrationDeviceState(VFIO_DEVICE_STATE_STOP_COPY);

  int fd = writer->BeginWrite("DATA");
  bool success = false;
  size_t buffer_size = 1024 * 1024;
  auto buffer = new uint8_t[buffer_size];

  /* Read from data_fd and write to fd */
  while (true) {
    auto ret = read(migration_.data_fd, buffer, buffer_size);
    if (ret < 0) {
      success = false;
      break;
    }
    if (ret == 0) {
      success = true;
      break;
    }
    if (dynamic_cast<MigrationNetworkWriter*>(writer)) {
      if (!writer->WriteRaw("VFIO_DATA", buffer, ret)) {
        success = false;
        break;
      }
    } else {
      MV_ASSERT(write(fd, buffer, ret) == ret);
    }
  }

  delete[] buffer;
  writer->EndWrite("DATA");
  SetMigrationDeviceState(VFIO_DEVICE_STATE_STOP);
  return success;
}

bool VfioPci::SaveState(MigrationWriter* writer) {
  if (!migration_.enabled) {
    MV_LOG("%s:%s blocked migration", name_, device_name_.c_str());
    return false;
  }

  bool success = false;
  if (migration_.version == 1) {
    success = SaveDeviceStateV1(writer);
  } else {
    success = SaveDeviceStateV2(writer);
  }
  return success && PciDevice::SaveState(writer);
}

bool VfioPci::LoadDeviceStateV1(MigrationReader* reader) {
  SetMigrationDeviceState(VFIO_DEVICE_STATE_V1_RESUMING);

  /* buffer for restoring */
  auto& area = migration_.region->mmap_areas.front();
  int fd = reader->BeginRead("DATA");
  bool success = false;

  while (true) {
    uint64_t data_offset = 0;
    pread(device_fd_, &data_offset, sizeof(data_offset),
      migration_.region->offset + offsetof(vfio_device_migration_info, data_offset));
    if (data_offset != area.offset) {
      MV_ERROR("failed to read vfio, data_offset=0x%lx area.offset=0x%lx", data_offset, area.offset);
      break;
    }
    
    ssize_t ret;
    if (dynamic_cast<MigrationNetworkReader*>(reader)) {
      ret = reader->ReadRawWithLimit("VFIO_DATA", area.mmap, area.size);
    } else {
      ret = read(fd, area.mmap, area.size);
    }

    if (ret > 0) {
      pwrite(device_fd_, &ret, sizeof(ret),
        migration_.region->offset + offsetof(vfio_device_migration_info, data_size));
    }
    if (ret < (ssize_t)area.size) {
      success = true;
      break;
    }
  }

  reader->EndRead("DATA");
  SetMigrationDeviceState(VFIO_DEVICE_STATE_V1_STOP);
  return success;
}

bool VfioPci::LoadDeviceStateV2(MigrationReader* reader) {
  SetMigrationDeviceState(VFIO_DEVICE_STATE_RESUMING);

  int fd = reader->BeginRead("DATA");
  bool success = false;
  size_t buffer_size = 1024 * 1024;
  auto buffer = new uint8_t[buffer_size];

  while (true) {
    ssize_t ret;
    if (dynamic_cast<MigrationNetworkReader*>(reader)) {
      ret = reader->ReadRawWithLimit("VFIO_DATA", buffer, buffer_size);
    } else {
      ret = read(fd, buffer, buffer_size);
    }

    if (ret > 0) {
      MV_ASSERT(write(migration_.data_fd, buffer, ret) == ret);
    }
    if (ret < (ssize_t)buffer_size) {
      success = true;
      break;
    }
  }

  delete[] buffer;
  reader->EndRead("DATA");
  SetMigrationDeviceState(VFIO_DEVICE_STATE_STOP);
  return success;
}

bool VfioPci::LoadState(MigrationReader* reader) {
  bool success = false;
  if (!PciDevice::LoadState(reader)) {
    return success;
  }

  if (!migration_.enabled) {
    MV_LOG("%s:%s blocked migration", name_, device_name_.c_str());
    return false;
  }

  /* Restore the PCI config space to VFIO device */
  auto &config_region = regions_[VFIO_PCI_CONFIG_REGION_INDEX];
  for (uint i = 0; i < pci_config_size(); i += 4) {
    pwrite(device_fd_, &pci_header_.data[i], 4, config_region.offset + i);
  }

  /* Update MSI routes */
  if (msi_config_.enabled) {
    UpdateMsiRoutes();
  } else {
    EnableIntxInterrupt();
  }

  if (migration_.version == 1) {
    success = LoadDeviceStateV1(reader);
  } else {
    success = LoadDeviceStateV2(reader);
  }
  return success;
}

DECLARE_DEVICE(VfioPci);
