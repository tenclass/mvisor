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

#include "pci_host.h"

#include <cstring>

#include "logger.h"
#include "device_manager.h"
#include "machine.h"
#include "pci_host.pb.h"


PciHost::PciHost() {
  slot_ = 0;
  function_ = 0;
  set_default_parent_class("SystemRoot");
  
  pci_header_.class_code = 0x060000;
  pci_header_.header_type = PCI_HEADER_TYPE_NORMAL;
  pci_header_.subsys_vendor_id = 0x1AF4;
  pci_header_.subsys_id = 0x1100;

  AddIoResource(kIoResourceTypePio, 0xCF8, 4, "PCI Config Address Port",
    nullptr, kIoResourceFlagCoalescingMmio);
  AddIoResource(kIoResourceTypePio, 0xCFC, 4, "PCI Config Data Port");
}

bool PciHost::SaveState(MigrationWriter* writer) {
  PciHostState state;
  state.set_config_address(config_.value);
  writer->WriteProtobuf("PCI_HOST", state);
  return PciDevice::SaveState(writer);
}

bool PciHost::LoadState(MigrationReader* reader) {
  if (!PciDevice::LoadState(reader)) {
    return false;
  }

  PciHostState state;
  /* For old snapshots, the name is not PCI_HOST, so just skip it */
  if (!reader->Exists("PCI_HOST")) {
    return true;
  }
  if (!reader->ReadProtobuf("PCI_HOST", state)) {
    return false;
  }
  config_.value = state.config_address();
  return true;
}

void PciHost::Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  if (resource->base == 0xCF8) {
    if (offset == 1 && size == 1) {
      /* 0xCF9 old system reset (DOS)
       * data[0] bit 2: soft reset 4: hard reset 8: full reset */
      if (debug_) {
        MV_LOG("system reset");
      }
      manager_->machine()->Reset();
      return;
    }
    memcpy(config_.data + offset, data, size);
  } else if (resource->base == 0xCFC) {
    PciDevice* pci = manager_->LookupPciDevice(config_.bus, config_.slot, config_.function);
    if (pci) {
      std::lock_guard<std::recursive_mutex> lock(pci->mutex());
      config_.reg_offset = offset;
      pci->WritePciConfigSpace(config_.value & 0xFF, data, size);
    } else {
      if (debug_) {
        MV_ERROR("failed to lookup pci %x:%x.%x", config_.bus, config_.slot, config_.function);
      }
    }
  } else {
    PciDevice::Write(resource, offset, data, size);
  }
}

void PciHost::Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  if (resource->base == 0xCF8) {
    MV_ASSERT(size == 4);
    memcpy(data, config_.data + offset, size);
  } else if (resource->base == 0xCFC) {
    PciDevice* pci = manager_->LookupPciDevice(config_.bus, config_.slot, config_.function);
    if (pci) {
      std::lock_guard<std::recursive_mutex> lock(pci->mutex());
      config_.reg_offset = offset;
      pci->ReadPciConfigSpace(config_.value & 0xFF, data, size);
    } else {
      memset(data, 0xFF, size);
    }
  } else {
    PciDevice::Read(resource, offset, data, size);
  }
}
