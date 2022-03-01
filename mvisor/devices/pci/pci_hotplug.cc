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

#include "device.h"
#include <cstring>
#include "logger.h"

#define ACPI_PCI_HOTPLUG_ADDR   0xAE00
#define ACPI_PCI_HOTPLUG_SIZE   0x14


/*
 * https://qemu.readthedocs.io/en/v6.2.0/specs/acpi_pci_hotplug.html
 */
class PciHotplug : public Device {
 private:
 public:
  PciHotplug() {
    AddIoResource(kIoResourceTypePio, ACPI_PCI_HOTPLUG_ADDR, ACPI_PCI_HOTPLUG_SIZE, "ACPI PCI HOTPLUG");
  }

  virtual void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    MV_ASSERT(size == 4);
  
    if (offset == 0x0C) { // PCI removability status
      /* disable all PCIs hotplug */
      bzero(data, size);
      return;
    }
    Device::Read(resource, offset, data, size);
  }

};

DECLARE_DEVICE(PciHotplug);
