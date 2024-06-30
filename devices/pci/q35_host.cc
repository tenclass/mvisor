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
#include "pci_host.h"

#define MCH_PCIE_XBAR_OFFSET         0x60
#define MCH_PCIE_XBAR_SIZE           0x04

class Q35Host : public PciHost {
 private:
  uint64_t  pcie_xbar_base_ = 0;

 public:
  Q35Host() {
    pci_header_.vendor_id = 0x8086;
    pci_header_.device_id = 0x29C0;

    is_pcie_ = true;
  }

  virtual bool LoadState(MigrationReader* reader) {
    if (!PciHost::LoadState(reader)) {
      return false;
    }

    MchUpdatePcieXBar();
    return true;
  }

  void MchUpdatePcieXBar() {
    uint32_t xbar = *(uint32_t*)(pci_header_.data + MCH_PCIE_XBAR_OFFSET);
    int enabled = xbar & 1;

    if (!!enabled != !!pcie_xbar_base_) {
      uint32_t base = xbar & Q35_MASK(64, 35, 28);
      uint64_t length = (1LL << 20) * 256;
      if (pcie_xbar_base_) {
        RemoveIoResource(kIoResourceTypeMmio, pcie_xbar_base_);
        pcie_xbar_base_ = 0;
      }
      if (enabled) {
        AddIoResource(kIoResourceTypeMmio, base, length, "PCIE XBAR");
        pcie_xbar_base_ = base;
      }
    }
  }

  void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    if (pcie_xbar_base_ && resource->base == pcie_xbar_base_) {
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
      PciHost::Write(resource, offset, data, size);
    }
  }

  void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    if (pcie_xbar_base_ && resource->base == pcie_xbar_base_) {
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
      PciHost::Read(resource, offset, data, size);
    }
  }

  void WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
    PciHost::WritePciConfigSpace(offset, data, length);

    if (ranges_overlap(offset, length, MCH_PCIE_XBAR_OFFSET, MCH_PCIE_XBAR_SIZE)) {
      MchUpdatePcieXBar();
    } else if (ranges_overlap(offset, length, 0x9D, 2)) {
      MV_PANIC("SMRAM not supported yet");
    }
  }

};

DECLARE_DEVICE(Q35Host);
