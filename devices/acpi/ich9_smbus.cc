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

#include "pci_device.h"

class Ich9Smbus : public PciDevice {
 public:
  Ich9Smbus() {
    slot_ = 31;
    function_ = 3;
    
    pci_header_.vendor_id = 0x8086;
    pci_header_.device_id = 0x2930;
    pci_header_.class_code = 0x0C0500;
    pci_header_.revision_id = 2;
    pci_header_.header_type = PCI_MULTI_FUNCTION | PCI_HEADER_TYPE_NORMAL;
    pci_header_.subsys_vendor_id = 0x1AF4;
    pci_header_.subsys_id = 0x1100;
    pci_header_.irq_pin = 1;

    SetupPciBar(4, 64, kIoResourceTypePio);
  }
};

DECLARE_DEVICE(Ich9Smbus);
