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
#include "device_interface.h"
#include "logger.h"
#include "pvpanic.hex"

#define PVPANIC_PANICKED	(1 << 0)
#define PVPANIC_CRASH_LOADED	(1 << 1)


class Pvpanic : public Device, public AcpiTableInterface {
  uint8_t   supported_events = PVPANIC_PANICKED | PVPANIC_CRASH_LOADED;
  uint16_t  port = 0x505;

 public:
  Pvpanic() {
    set_default_parent_class("Ich9Lpc", "Piix3");

    AddIoResource(kIoResourceTypePio, port, 1, "PvPanic");
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

  std::string GetAcpiTable() override {
    auto data = std::string((const char*)pvpanic_aml_code, sizeof(pvpanic_aml_code));
    // Patch port
    auto it = data.find("\xFF\xFF");
    if (it != std::string::npos) {
      *(uint16_t*)&data[it] = port;
    }
    // Recalculate checksum
    int checksum = 0;
    data[9] = 0;
    for (size_t i = 0; i < data.size(); i++) {
      checksum += data[i];
    }
    data[9] = -checksum;
    return data;
  }
};

DECLARE_DEVICE(Pvpanic);
