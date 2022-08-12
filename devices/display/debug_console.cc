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

#include "device.h"
#include "machine.h"

class DebugConsole : public Device {
 public:
  DebugConsole() {
    AddIoResource(kIoResourceTypePio, 0x402, 1, "SeaBIOS Output");
  }

  void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    MV_UNUSED(resource);
    MV_UNUSED(offset);
    MV_UNUSED(size);
    if (manager_->machine()->debug()) {
      putchar(*data);
    }
  }

  void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    MV_UNUSED(resource);
    MV_UNUSED(offset);
    MV_UNUSED(size);
    *data = 0xe9;
  }

};

DECLARE_DEVICE(DebugConsole);
