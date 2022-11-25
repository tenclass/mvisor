/* 
 * MVisor - AHCI Host Controller
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

#include "ahci_host.h"

class Ich9Ahci : public AhciHost {
 public:
  Ich9Ahci() {
    slot_ = 31;
    function_ =  2;
    
    pci_header_.vendor_id = 0x8086;
    pci_header_.device_id = 0x2922;
  }
};

DECLARE_DEVICE(Ich9Ahci);
