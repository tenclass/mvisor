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
  bzero(&regions_, sizeof(regions_));
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
  SetupMigraionInfo();
}

void VfioPci::Disconnect() {
  auto machine = manager_->machine();
  if (state_change_listener_) {
    machine->UnregisterStateChangeListener(&state_change_listener_);
  }

  DisableInterrupts();
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

  DisableInterrupts();
  PciDevice::Reset();
}

void VfioPci::DisableInterrupts() {
  if (msi_config_.enabled) {
    for (auto &interrupt : interrupts_) {
      if (interrupt.gsi > 0) {
        manager_->RemoveMsiNotifier(interrupt.gsi, interrupt.event_fd);
        interrupt.gsi = -1;
      }
      safe_close(&interrupt.event_fd);
    }
  } else {
    SetIntxInterruptEnabled(false);
  }
}

void VfioPci::SetIntxInterruptEnabled(bool enabled) {
  if (!pci_header_.irq_pin) {
    return;
  }
  if (debug_) {
    MV_LOG("%s set intx interrupt %s", device_name_.c_str(), enabled ? "enabled" : "disabled");
  }

  if (enabled) {
    if (intx_trigger_fd_ == -1) {
      SetInterruptMasked(VFIO_PCI_INTX_IRQ_INDEX, true);
      intx_trigger_fd_ = eventfd(0, 0);
      intx_unmask_fd_ = eventfd(0, 0);
      SetInterruptEventFds(VFIO_PCI_INTX_IRQ_INDEX, 0, VFIO_IRQ_SET_ACTION_TRIGGER, &intx_trigger_fd_, 1);
      SetInterruptEventFds(VFIO_PCI_INTX_IRQ_INDEX, 0, VFIO_IRQ_SET_ACTION_UNMASK, &intx_unmask_fd_, 1);
      manager_->SetPciIrqNotifier(this, intx_trigger_fd_, intx_unmask_fd_, true);
      SetInterruptMasked(VFIO_PCI_INTX_IRQ_INDEX, false);
    }
  } else {
    if (intx_trigger_fd_ != -1) {
      SetInterruptMasked(VFIO_PCI_INTX_IRQ_INDEX, true);
      manager_->SetPciIrqNotifier(this, intx_trigger_fd_, -1, false);
      safe_close(&intx_trigger_fd_);
      safe_close(&intx_unmask_fd_);
      SetInterruptMasked(VFIO_PCI_INTX_IRQ_INDEX, false);
      SetInterruptEventFds(VFIO_PCI_INTX_IRQ_INDEX, 0, VFIO_IRQ_SET_ACTION_TRIGGER, nullptr, 0);
    }
  }
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
      if (debug_) {
        MV_ERROR("failed to get region info %d, ret=%d", index, ret);
      }
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
    if (debug_) {
      MV_LOG("irq index=%d size=%u flags=%x count=%d ret=%d", index,
        irq_info.argsz, irq_info.flags, irq_info.count, ret);
    }

    if (index == VFIO_PCI_INTX_IRQ_INDEX || index == VFIO_PCI_MSI_IRQ_INDEX || index == VFIO_PCI_MSIX_IRQ_INDEX) {
      if (!(irq_info.flags & VFIO_IRQ_INFO_EVENTFD)) {
        MV_PANIC("%s irq index %d does not support eventfd", device_name_.c_str(), index);
      }
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
      case PCI_CAP_ID_PM:
        /* ignore power management */
        break;
      case PCI_CAP_ID_EXP:
        is_pcie_ = true;
        break;
      default:
        if (debug_) {
          MV_WARN("unhandled capability=0x%x", cap->type);
        }
        break;
      }
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

void VfioPci::WritePciCommand(uint16_t command) {
  uint16_t old_comand = pci_header_.command;
  PciDevice::WritePciCommand(command);

  if (!msi_config_.enabled) {
    if ((old_comand ^ command) & PCI_COMMAND_INTX_DISABLE) {
      if (command & PCI_COMMAND_INTX_DISABLE) {
        SetIntxInterruptEnabled(false);
      } else {
        SetIntxInterruptEnabled(true);
      }
    }
  }
}

void VfioPci::MapBarRegion(uint8_t index) {
  uint64_t address;
  if (pci_bars_[index].special_bits & PCI_BASE_ADDRESS_MEM_TYPE_64) {
    address = pci_header_.bars[index + 1] & PCI_BASE_ADDRESS_MEM_MASK;
    address <<= 32;
    address |= pci_header_.bars[index] & PCI_BASE_ADDRESS_MEM_MASK;
  } else {
    address = pci_header_.bars[index] & PCI_BASE_ADDRESS_MEM_MASK;
  }
  uint64_t length = pci_bars_[index].size;

  auto &region = regions_[index];
  if (region.mmap_areas.empty()) {
    AddIoResource(kIoResourceTypeRam, address, length, "VFIO BAR RAM", region.mmap);
  } else {
    /* The MMIO region is overlapped by the mmap areas */
    AddIoResource(kIoResourceTypeMmio, address, length, "VFIO BAR MMIO");
    for (auto &area : region.mmap_areas) {
      AddIoResource(kIoResourceTypeRam, address + area.offset, area.size, "VFIO BAR RAM", area.mmap);
    }
  }
}

void VfioPci::UnmapBarRegion(uint8_t index) {
  uint64_t address;
  if (pci_bars_[index].special_bits & PCI_BASE_ADDRESS_MEM_TYPE_64) {
    address = pci_header_.bars[index + 1] & PCI_BASE_ADDRESS_MEM_MASK;
    address <<= 32;
    address |= pci_header_.bars[index] & PCI_BASE_ADDRESS_MEM_MASK;
  } else {
    address = pci_header_.bars[index] & PCI_BASE_ADDRESS_MEM_MASK;
  }

  auto &region = regions_[index];
  if (region.mmap_areas.empty()) {
    RemoveIoResource(kIoResourceTypeRam, address);
  } else {
    for (auto &area : region.mmap_areas) {
      RemoveIoResource(kIoResourceTypeRam, address + area.offset);
    }
    RemoveIoResource(kIoResourceTypeMmio, address);
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
    if (bar.active) {
      UnmapBarRegion(index);
      bar.active = false;
    }
    return true;
  }
  return PciDevice::DeactivatePciBar(index);
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
      auto ret = pwrite(device_fd_, data, size, regions_[i].offset + offset);
      if (ret != (ssize_t)size) {
        MV_PANIC("failed to write VFIO device, offset=0x%x size=0x%x data=0x%x", offset, size, *(uint32_t*)data);
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
        MV_PANIC("failed to read VFIO device, offset=0x%x size=0x%x", offset, size);
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
  MV_ASSERT(ioctl(device_fd_, VFIO_DEVICE_SET_IRQS, irq_set) == 0);
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
  ioctl(device_fd_, VFIO_DEVICE_SET_IRQS, &irq_set);
}

void VfioPci::UpdateMsiRoutes() {
  uint nr_vectors = 0;
  uint16_t control = 0;
  if (msi_config_.is_msix) {
    control = msi_config_.msix->control;
    nr_vectors = msi_config_.msix_table_size;
  } else {
    control =  msi_config_.is_64bit ? msi_config_.msi64->control : msi_config_.msi32->control;
    nr_vectors = 1 << ((control & PCI_MSI_FLAGS_QSIZE) >> 4);
  }

  DisableInterrupts();

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

    if (msi_config_.enabled && msi_address) {
      if (interrupt.gsi == -1) {
        interrupt.gsi = manager_->AddMsiNotifier(msi_address, msi_data, interrupt.event_fd);
        SetInterruptEventFds(msi_config_.is_msix ? VFIO_PCI_MSIX_IRQ_INDEX : VFIO_PCI_MSI_IRQ_INDEX,
          vector, VFIO_IRQ_SET_ACTION_TRIGGER, &interrupt.event_fd, 1);
      }
    } else {
      if (interrupt.gsi != -1) {
        manager_->RemoveMsiNotifier(interrupt.gsi, interrupt.event_fd);
        interrupt.gsi = -1;
      }
    }
  }
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

  /* read from VFIO device */
  if (!ranges_overlap(offset, length, msi_config_.offset + PCI_MSI_FLAGS, 1)) {
    auto &config_region = regions_[VFIO_PCI_CONFIG_REGION_INDEX];
    auto ret = pread(device_fd_, pci_header_.data + offset, length, config_region.offset + offset);
    MV_ASSERT(ret == (ssize_t)length);
  }

  if (multi_function_) {
    pci_header_.header_type = 0x80;
  } else {
    pci_header_.header_type = 0;
  }

  /* Disable PCI-e capability for NVIDIA GPUs to avoid error code 10 */
  if (pci_header_.vendor_id == 0x10DE && pci_header_.data[0x69] == 0x78) {
    pci_header_.data[0x69] = 0xB4;
  }

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
    
    if (dynamic_cast<MigrationNetworkWriter*>(writer)) {
      if (writer->WriteRaw("VFIO_DATA", buffer, data_size)) {
        ret = data_size;
      } else {
        MV_ERROR("send vfio data error");
        break;
      }
    } else {
      ret = write(fd, buffer, data_size);
    }
    MV_ASSERT(ret == (ssize_t)data_size);
  }

  writer->EndWrite("DATA");
  SetMigrationDeviceState(VFIO_DEVICE_STATE_STOP);
  
  munmap(buffer, area.size);
  return success && PciDevice::SaveState(writer);
}

bool VfioPci::LoadState(MigrationReader* reader) {
  bool success = false;
  if (!PciDevice::LoadState(reader)) {
    return success;
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
    if (data_offset != area.offset) {
      MV_ERROR("failed to read vfio, data_offset=0x%lx area.offset=0x%lx", data_offset, area.offset);
      break;
    }
    
    ssize_t ret;
    if (dynamic_cast<MigrationNetworkReader*>(reader)) {
      ret = reader->ReadRawWithLimit("VFIO_DATA", buffer, area.size);
    } else {
      ret = read(fd, buffer, area.size);
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
  SetMigrationDeviceState(VFIO_DEVICE_STATE_STOP);
  
  munmap(buffer, area.size);
  return success;
}

DECLARE_DEVICE(VfioPci);
