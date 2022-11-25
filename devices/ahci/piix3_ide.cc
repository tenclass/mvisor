/* 
 * MVisor - IDE Host Controller
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

#include "ide_host.h"

class Piix3Ide : public IdeHost {
 public:
  Piix3Ide() {
    slot_ = 1;
    function_ =  1;
    
    /* PIIX3-IDE */
    pci_header_.vendor_id = 0x8086;
    pci_header_.device_id = 0x7010;
  }
};

DECLARE_DEVICE(Piix3Ide);
