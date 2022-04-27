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
#include "disk_image.h"
#include <sys/uio.h>
#include <vector>
#include <functional>

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
{
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


struct IdeIo {
  ssize_t         buffer_size;
  uint8_t*        buffer;
  ssize_t         nbytes;
  IdeLbaMode      lba_mode;
  size_t          lba_count;
  size_t          lba_block;
  uint32_t        dma_status;
  uint8_t         atapi_command[16];
  std::vector<struct iovec> vector; 
};

struct IdeDriveInfo {
  char serial[21];
  char model[41];
  char version[41];
  uint64_t world_wide_name;
};

enum IdeStorageType {
  kIdeStorageTypeDisk,
  kIdeStorageTypeCdrom
};

typedef std::function<void(void)> VoidCallback;
class AhciPort;

class IdeStorageDevice : public Device {
 public:
  IdeStorageDevice();
  virtual void Connect();
  virtual void Disconnect();
  void Reset();
  bool IsAvailable();

  virtual bool StartCommand(VoidCallback iocp);
  virtual void AbortCommand();
  virtual void CompleteCommand();

  IdeStorageType  type() { return type_; }
  IdeIo*          io() { return &io_; }
  IdeRegisters*   regs() { return &regs_; }
  bool            io_async() { return io_async_; }

 protected:
  virtual void Ata_ResetSignature();
  virtual void Ata_IdentifyDevice();
  virtual void Ata_SetFeatures();

  DiskImage*      image_;
  IdeRegisters    regs_;
  IdeIo           io_;
  
  IdeStorageType  type_; /* disk or cdrom */

  IdeDriveInfo    drive_info_;
  VoidCallback    ata_handlers_[256];
  bool            write_cache_ = true;
  VoidCallback    io_complete_;
  bool            io_async_ = false;
};


class AhciCdrom : public IdeStorageDevice {
 public:
  AhciCdrom();
  virtual void Connect();
  virtual bool SaveState(MigrationWriter* writer);
  virtual bool LoadState(MigrationReader* reader);

 private:
  void ParseCommandPacket();
  void Atapi_IdentifyData();
  void Atapi_Inquiry();
  void Atapi_ReadSectorsAsync();
  void Atapi_TableOfContent();
  void Atapi_ModeSense();
  void Atapi_RequestSense();
  void SetError(uint sense_key, uint asc);

  uint    sense_key_;
  uint    asc_;
  size_t  total_tracks_;
  size_t  track_size_ = 2048; // Cdrom always use 2048 track size
  size_t  image_block_size_;

  VoidCallback atapi_handlers_[256];
};


struct DiskGeometry {
  size_t sector_size;
  size_t total_sectors;
  size_t sectors_per_cylinder;
  size_t cylinders_per_heads;
  size_t heads;
};

class AhciDisk : public IdeStorageDevice {
 public:
  AhciDisk();
  virtual void Connect();

 private:
  void ReadLba();
  void WriteLba();
  void InitializeGeometry();
  void Ata_IdentifyDevice();
  void Ata_ReadWriteSectorsAsync(bool is_write);
  void Ata_TrimAsync();

  DiskGeometry geometry_;
  int multiple_sectors_;
};

#endif // _MVISOR_DEVICES_IDE_STORAGE_H
