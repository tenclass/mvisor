/* 
 * MVisor
 * Copyright (C) 2024 Terrence <terrence@tenclass.com>
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
#include "logger.h"

#define PVPANIC_PANICKED	(1 << 0)
#define PVPANIC_CRASH_LOADED	(1 << 1)


class Pvpanic : public Device {
  uint8_t supported_events = PVPANIC_PANICKED | PVPANIC_CRASH_LOADED;

 public:
  Pvpanic() {
    set_default_parent_class("Ich9Lpc", "Piix3");

    AddIoResource(kIoResourceTypePio, 0x505, 1, "PvPanic");
  }

  void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    if (offset == 0 && size == 1) {
      data[0] = supported_events;
    } else {
      Device::Read(resource, offset, data, size);
    }
  }

  void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    if (offset == 0 && size == 1) {
      int event = data[0];

      if (event & PVPANIC_PANICKED) {
        MV_WARN("Guest panicked");
      }

      if (event & PVPANIC_CRASH_LOADED) {
        MV_WARN("Guest crashed");
      }
    } else {
      Device::Write(resource, offset, data, size);
    }
  }
};

DECLARE_DEVICE(Pvpanic);
