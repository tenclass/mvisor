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

#ifndef _MVISOR_DEVICES_IDE_HOST_H
#define _MVISOR_DEVICES_IDE_HOST_H

#include "pci_device.h"
#include "ide_port.h"
#include <array>

class IdeHost : public PciDevice {
 public:
  IdeHost();
  virtual ~IdeHost();

  virtual void Connect();
  virtual void Disconnect();
  virtual void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void Reset();

  bool SaveState(MigrationWriter* writer);
  bool LoadState(MigrationReader* reader);

  void SetPortIrq(int index, uint level);

 private:
  std::array<IdePort*, 2>       ports_ = { 0 };
};

#endif // _MVISOR_DEVICES_IDE_HOST_H
