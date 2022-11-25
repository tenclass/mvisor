/* 
 * MVisor USB 1.0 UHCI
 * https://wiki.osdev.org/Universal_Host_Controller_Interface
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

#include "uhci_host.h"

class Piix3Uhci : public UhciHost {
 public:
  Piix3Uhci() {
    slot_ = 1;
    function_ = 2;

    /* PCI_DEVICE_ID_INTEL_82371SB_2 */
    pci_header_.vendor_id = 0x8086;
    pci_header_.device_id = 0x7020;
    pci_header_.irq_pin = 3;
  }
};

DECLARE_DEVICE(Piix3Uhci);
