/* 
 * MVisor
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

#include <cstring>

#include "logger.h"
#include "pmio.h"
#include "device_manager.h"
#include "machine.h"
#include "piix4_pm.pb.h"


#define PIIX_PMIO_BASE      0x40
#define PIIX_PMIO_MISC      0x80
#define PIIX_SMBUS_BASE     0x90
#define PIIX_SMBUS_MISC     0xD2
#define PIIX_PMIO_SIZE      64


class Piix4Pm : public Pmio {
 private:
  struct {
    uint8_t     control;
    uint8_t     status;
  } apm_;

  bool          initialized_pmio_ = false;
  bool          initialized_smbus_ = false;

  void UpdatePmio() {
    /* PM IO base should be 0xB000 */
    pmio_base_ = *(uint32_t*)(pci_header_.data + PIIX_PMIO_BASE);
    pmio_base_ &= 0xFFC0;
    uint8_t misc = pci_header_.data[PIIX_PMIO_MISC];

    if (misc & 1) {
      if (!initialized_pmio_) {
        AddIoResource(kIoResourceTypePio, pmio_base_, PIIX_PMIO_SIZE, "PMIO");
        initialized_pmio_ = true;
      }
    } else {
      if (initialized_pmio_) {
        RemoveIoResource(kIoResourceTypePio, "PMIO");
        pmio_base_ = 0;
        initialized_pmio_ = false;
      }
    }
  }

  void UpdateSmbus() {
    uint32_t smbus_base = *(uint32_t*)(pci_header_.data + PIIX_SMBUS_BASE);
    smbus_base &= 0xFFC0;
    uint8_t smbus_misc = pci_header_.data[PIIX_SMBUS_MISC];
    if (smbus_misc & 1) {
      if (!initialized_smbus_) {
        AddIoResource(kIoResourceTypePio, smbus_base, 64, "SMBUS");
        initialized_smbus_ = true;
      }
    } else {
      if (initialized_smbus_) {
        RemoveIoResource(kIoResourceTypePio, "SMBUS");
        initialized_smbus_ = false;
      }
    }
  }

 public:
  Piix4Pm() {
    slot_ = 1;
    function_ = 3;

    pci_header_.vendor_id = 0x8086;
    pci_header_.device_id = 0x7113;
    pci_header_.class_code = 0x068000;
    pci_header_.revision_id = 3;
    pci_header_.header_type = PCI_HEADER_TYPE_NORMAL;
    pci_header_.subsys_vendor_id = 0x1AF4;
    pci_header_.subsys_id = 0x1100;
    pci_header_.irq_pin = 1;

    AddIoResource(kIoResourceTypePio, 0xB2, 2, "LPC APM");
    // /* https://qemu.readthedocs.io/en/v6.2.0/specs/acpi_pci_hotplug.html */
    AddIoResource(kIoResourceTypePio, 0xAE00, 20, "ACPI PCI HOTPLUG");
  }

  void Connect() {
    Pmio::Connect();

    /* Device present mark at pci_header_.data 0x5F 0x67 not handled yet */
    pci_header_.data[0x5F] = 0;
  }

  void Reset() {
    Pmio::Reset();

    /* Mark SMM initialized (not supported now) */
    pci_header_.data[0x5B] = 2;
    
    bzero(&apm_, sizeof(apm_));
    *(uint32_t*)(pci_header_.data + PIIX_PMIO_BASE) = 1;
    pci_header_.data[PIIX_PMIO_MISC] = 0;
    UpdatePmio();
  }

  bool SaveState(MigrationWriter* writer) {
    Piix4PmState state;
    auto apm = state.mutable_apm();
    apm->set_control(apm_.control);
    apm->set_status(apm_.status);

    writer->WriteProtobuf("PIIX4_PM", state);
    return Pmio::SaveState(writer);
  }

  bool LoadState(MigrationReader* reader) {
    if (!Pmio::LoadState(reader)) {
      return false;
    }

    Piix4PmState state;
    if (!reader->ReadProtobuf("PIIX4_PM", state)) {
      return false;
    }
    auto& apm = state.apm();
    apm_.control = apm.control();
    apm_.status = apm.status();

    UpdatePmio();
    return true;
  }

  void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    if (resource->base == 0xB2) { // APM IO
      if (offset == 0) {
        data[0] = apm_.control;
      } else {
        data[0] = apm_.status;
      }
    } else if (resource->base == 0xAE00) { // ACPI PCI HOTPLUG
      if (offset == 0x0C) { // PCI removability status
        /* disable all PCIs hotplug */
        bzero(data, size);
      }
    } else {
      Pmio::Read(resource, offset, data, size);
    }
  }

  void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    if (resource->base == 0xB2) { // APM IO
      if (offset == 0) {
        apm_.control = data[0];
        if (apm_.control == 2) { // Enable ACPI
          pm1_.control |= PMIO_PM1_CTRL_SMI_EN;
        } else if (apm_.control == 3) { // Disable ACPI
          pm1_.control &= ~PMIO_PM1_CTRL_SMI_EN;
        } else {
          MV_PANIC("unknown apm control=0x%x", *data);
        }
      } else {
        apm_.status = data[0];
      }
    } else {
      Pmio::Write(resource, offset, data, size);
    }
  }

  void WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
    Pmio::WritePciConfigSpace(offset, data, length);

    if (ranges_overlap(offset, length, PIIX_PMIO_BASE, 4) ||
        ranges_overlap(offset, length, PIIX_PMIO_MISC, 1)) {
      UpdatePmio();
    } else if (ranges_overlap(offset, length, PIIX_SMBUS_BASE, 4) ||
               ranges_overlap(offset, length, PIIX_SMBUS_MISC, 1)) {
      UpdateSmbus();
    }
  }
};

DECLARE_DEVICE(Piix4Pm);
