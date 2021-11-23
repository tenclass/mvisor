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

class SmBus : public PciDevice {
 public:
  SmBus() {
    devfn_ = PCI_MAKE_DEVFN(0x1f, 3);
    
    pci_header_.vendor_id = 0x8086;
    pci_header_.device_id = 0x2930;
    pci_header_.class_code = 0x0C0500;
    pci_header_.revision_id = 2;
    pci_header_.header_type = PCI_HEADER_TYPE_NORMAL;
    pci_header_.subsys_vendor_id = 0x1af4;
    pci_header_.subsys_id = 0x1100;
    pci_header_.irq_pin = 1;

    AddPciBar(4, 64, kIoResourceTypePio);
  }
};

DECLARE_DEVICE(SmBus);
