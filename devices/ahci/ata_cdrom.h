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

#ifndef _MVISOR_DEVICES_AHCI_ATA_CDROM_H
#define _MVISOR_DEVICES_AHCI_ATA_CDROM_H

#include <string>

#include "ata_storage.h"


class AtaCdrom : public AtaStorageDevice {
 public:
  AtaCdrom();
  virtual void Connect();
  virtual bool SaveState(MigrationWriter* writer);
  virtual bool LoadState(MigrationReader* reader);
  virtual void StartTransfer(TransferType type, VoidCallback end_cb);
  virtual void EndCommand();

 private:
  void ParseCommandPacket();
  void Atapi_IdentifyData();
  void Atapi_Inquiry();
  void Atapi_ReadSectors(size_t sectors);
  void Atapi_ReadTableOfContent();
  void Atapi_ModeSense();
  void Atapi_RequestSense();
  void Atapi_GetMediaCapacity();
  void Atapi_ReadSubchannel();
  void SetError(uint sense_key, uint asc);

  uint    sense_key_;
  uint    asc_;
  size_t  image_block_size_;

  VoidCallback atapi_handlers_[256];
};


#endif // _MVISOR_DEVICES_AHCI_ATA_CDROM_H
