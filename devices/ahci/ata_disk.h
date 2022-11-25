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

#ifndef _MVISOR_DEVICES_AHCI_ATA_DISK_H
#define _MVISOR_DEVICES_AHCI_ATA_DISK_H

#include "ata_storage.h"


class AtaDisk : public AtaStorageDevice {
 public:
  AtaDisk();
  virtual void Connect();

 private:
  void ReadLba(LbaMode mode);
  void WriteLba();
  void Ata_IdentifyDevice();
  void Ata_DmaReadWriteSectors(bool is_write, size_t sectors);
  void Ata_PioReadSectors(size_t sectors);
  void Ata_PioWriteSectors(size_t sectors);
  void Ata_Trim();
};

#endif // _MVISOR_DEVICES_AHCI_ATA_DISK_H
