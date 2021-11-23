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

#ifndef _MVISOR_DEVICES_IDE_STORAGE_H
#define _MVISOR_DEVICES_IDE_STORAGE_H

#include "device.h"
#include "storage_device.h"

#define IDE_MAX_REGISTERS 18

enum IdeLbaMode {
  kIdeLbaModeChs,
  kIdeLbaMode28,
  kIdeLbaMode48
};

enum IdeDmaMode {
  kIdePioMode,
  kIdeSingleWordDmaMode,
  kIdeMDmaMode,
  kIdeUDmaMode
};

struct IdeRegisters
{\
  uint8_t feature0;
  uint8_t feature1;
  
  uint8_t count0;
  uint8_t count1;
  uint8_t lba0;
  uint8_t lba1;
  uint8_t lba2;
  uint8_t lba3;
  uint8_t lba4;
  uint8_t lba5;

  uint8_t device;
  uint8_t command;

  uint8_t control;

  uint8_t error;
  uint8_t status;
} __attribute__((packed));


enum IdeTransferType {
  kIdeNoTransfer,
  kIdeTransferToDevice,
  kIdeTransferToHost
};

struct IdeIo {
  ssize_t         buffer_size;
  uint8_t*        buffer;
  ssize_t         position;
  ssize_t         nbytes;
  IdeTransferType transfer_type;
  IdeLbaMode      lba_mode;
  size_t          lba_count;
  size_t          lba_position;
  uint32_t        dma_status;
};

struct IdeDriveInfo {
  char serial[21];
  char model[41];
  char version[41];
  uint64_t world_wide_name;
};

enum IdeStorageType {
  kIdeStorageTypeHarddisk,
  kIdeStorageTypeCdrom
};

class AhciPort;

class IdeStorageDevice : public StorageDevice {
 public:
  IdeStorageDevice();
  virtual ~IdeStorageDevice();

  void Reset();
  void BindPort(AhciPort* port);

  virtual void StartCommand();
  virtual void EndCommand();
  virtual void AbortCommand();
  virtual void StartTransfer(IdeTransferType type);
  virtual void EndTransfer(IdeTransferType type);
  virtual void Ata_ResetSignature();

  IdeStorageType type() { return type_; }
 protected:
  virtual void Ata_IdentifyDevice();
  virtual void Ata_SetFeatures();

  /* disk or cdrom */
  IdeStorageType type_;
  /* port_ will be set if the drive is selected */
  AhciPort* port_;

  IdeDriveInfo drive_info_;
};


class Cdrom : public IdeStorageDevice {
 public:
  Cdrom();
  void Connect();
  void StartCommand();
  void EndCommand();
  void StartTransfer(IdeTransferType type);
  void EndTransfer(IdeTransferType type);

 private:
  void ParseCommandPacket();
  void Atapi_IdentifyData();
  void Atapi_Inquiry();
  void Atapi_ReadSectors(int chunk_count);
  void Atapi_TableOfContent();
  void Atapi_ModeSense();
  void Atapi_RequestSense();
  void SetError(int sense_key, int asc);

  int sense_key_;
  int asc_;
  size_t total_tracks_;
  size_t track_size_ = 2048; // Cdrom always use 2048 track size
  size_t image_block_size_;
};


struct DiskGemometry {
  size_t sector_size;
  size_t total_sectors;
  size_t sectors_per_cylinder;
  size_t cylinders_per_heads;
  size_t heads;
};

class Harddisk : public IdeStorageDevice {
 public:
  Harddisk();
  void Connect();
  void StartCommand();
  void EndTransfer(IdeTransferType type);

 private:
  void ReadLba();
  void WriteLba();
  void InitializeGemometry();
  void Ata_IdentifyDevice();
  void Ata_ReadSectors(int chunk_count);
  void Ata_WriteSectors(int chunk_count);

  DiskGemometry gemometry_;
  int multiple_sectors_;
};

#endif // _MVISOR_DEVICES_IDE_STORAGE_H
