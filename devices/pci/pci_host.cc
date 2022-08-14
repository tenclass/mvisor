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

#include <cstring>
#include "logger.h"
#include "device_manager.h"
#include "pci_device.h"
#include "pci_host.pb.h"

#define MCH_CONFIG_ADDR             0xCF8
#define MCH_CONFIG_DATA             0xCFC

#define MCH_PCIEXBAR                0x60
#define MCH_PCIEXBAR_SIZE           0x04

class PciHost : public PciDevice {
 private:
  uint64_t          pcie_xbar_base_ = 0;
  PciConfigAddress  config_;

 public:
  PciHost() {
    slot_ = 0;
    function_ = 0;
    is_pcie_ = true;
    set_parent_name("system-root");
    
    pci_header_.vendor_id = 0x8086;
    pci_header_.device_id = 0x29C0;
    pci_header_.class_code = 0x060000;
    pci_header_.header_type = PCI_HEADER_TYPE_NORMAL;
    pci_header_.subsys_vendor_id = 0x1AF4;
    pci_header_.subsys_id = 0x1100;

    AddIoResource(kIoResourceTypePio, MCH_CONFIG_ADDR, 4, "MCH Config Base");
    AddIoResource(kIoResourceTypePio, MCH_CONFIG_DATA, 4, "MCH Config Data");
  }

  virtual bool SaveState(MigrationWriter* writer) {
    MchState state;
    state.set_config(config_.value);
    writer->WriteProtobuf("MCH", state);
    return PciDevice::SaveState(writer);
  }

  virtual bool LoadState(MigrationReader* reader) {
    if (!PciDevice::LoadState(reader)) {
      return false;
    }
    MchState state;
    if (!reader->ReadProtobuf("MCH", state)) {
      return false;
    }
    config_.value = state.config();
    MchUpdatePcieXBar();
    return true;
  }

  void MchUpdatePcieXBar() {
    uint32_t pciexbar = *(uint32_t*)(pci_header_.data + MCH_PCIEXBAR);
    int enable = pciexbar & 1;
    if (!!enable != !!pcie_xbar_base_) {
      uint32_t base = pciexbar & Q35_MASK(64, 35, 28);
      uint64_t length = (1LL << 20) * 256;
      if (pcie_xbar_base_) {
        RemoveIoResource(kIoResourceTypeMmio, pcie_xbar_base_);
        pcie_xbar_base_ = 0;
      }
      if (enable) {
        AddIoResource(kIoResourceTypeMmio, base, length, "PCIE XBAR");
        pcie_xbar_base_ = base;
      }
    }
  }

  void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    if (resource->base == MCH_CONFIG_ADDR) {
      memcpy(config_.data + offset, data, size);
    
    } else if (resource->base == MCH_CONFIG_DATA) {
      MV_ASSERT(size <= 4);
      
      PciDevice* pci = manager_->LookupPciDevice(config_.bus, config_.slot, config_.function);
      if (pci) {
        config_.reg_offset = offset;
        pci->WritePciConfigSpace(config_.value & 0xFF, data, size);
      } else {
        MV_ERROR("failed to lookup pci %x:%x.%x", config_.bus, config_.slot, config_.function);
      }
    
    } else if (pcie_xbar_base_ && resource->base == pcie_xbar_base_) {
      /*
      * PCI express ECAM (Enhanced Configuration Address Mapping) format.
      * AKA mmcfg address
      * bit 20 - 28: bus number
      * bit 15 - 19: device number
      * bit 12 - 14: function number
      * bit  0 - 11: offset in configuration space of a given device
      */
      uint32_t addr = offset;
      uint16_t bus = (addr >> 20) & 0x1FF;
      uint8_t slot = (addr >> 15) & 0x1F;
      uint8_t function = (addr >> 12) & 0x7;
      PciDevice* pci = manager_->LookupPciDevice(bus, slot, function);
      if (pci) {
        pci->WritePciConfigSpace(addr & 0xFFF, data, size);
      } else {
        MV_ERROR("failed to lookup pci %x:%x.%x offset=0x%lx", bus, slot, function, offset);
      }
    
    } else {
      MV_PANIC("not implemented base=0x%lx offset=0x%lx data=0x%x size=%x",
        resource->base, offset, *(uint32_t*)data, size);
    }
  }

  void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    if (resource->base == MCH_CONFIG_ADDR) {
      memcpy(data, config_.data + offset, size);
    
    } else if (resource->base == MCH_CONFIG_DATA) {
      if (size > 4)
        size = 4;
      
      PciDevice* pci = manager_->LookupPciDevice(config_.bus, config_.slot, config_.function);
      if (pci) {
        config_.reg_offset = offset;
        pci->ReadPciConfigSpace(config_.value & 0xFF, data, size);
      } else {
        memset(data, 0xFF, size);
      }
    
    } else if (pcie_xbar_base_ && resource->base == pcie_xbar_base_) {
      uint32_t addr = offset;
      uint16_t bus = (addr >> 20) & 0x1FF;
      uint8_t slot = (addr >> 15) & 0x1F;
      uint8_t function = (addr >> 12) & 0x7;
      PciDevice* pci = manager_->LookupPciDevice(bus, slot, function);
      if (pci) {
        pci->ReadPciConfigSpace(addr & 0xFFF, data, size);
      } else {
        memset(data, 0xFF, size);
      }
    
    } else {
      MV_PANIC("not implemented base=0x%lx offset=0x%lx data=0x%x size=%x",
        resource->base, offset, *(uint32_t*)data, size);
    }
  }

  void WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
    PciDevice::WritePciConfigSpace(offset, data, length);
    if (ranges_overlap(offset, length, MCH_PCIEXBAR, MCH_PCIEXBAR_SIZE)) {
      MchUpdatePcieXBar();
    
    } else if (ranges_overlap(offset, length, 0x9D, 2)) {
      MV_PANIC("SMRAM not supported yet");
    
    }
  }

};

DECLARE_DEVICE(PciHost);
