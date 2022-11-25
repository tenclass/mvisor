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

#ifndef _MVISOR_DEVICES_AHCI_ATA_STORAGE_H
#define _MVISOR_DEVICES_AHCI_ATA_STORAGE_H

#include "device.h"
#include "disk_image.h"

#include <sys/uio.h>
#include <vector>
#include <functional>

#include "ata_internal.h"

enum LbaMode {
  kLbaModeChs,
  kLbaMode28,
  kLbaMode48
};

enum DmaMode {
  kPioMode            = 1,
  kSingleWordDmaMode  = 2,
  kMDmaMode           = 4,
  kUDmaMode           = 8
};

struct TaskFile
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


enum TransferType {
  kTransferAtapiCommand,
  kTransferDataToDevice,
  kTransferDataToHost
};

struct AtaIo {
  TransferType      type;
  size_t            buffer_size;
  uint8_t*          buffer;
  std::vector<iovec>vector;
  LbaMode           lba_mode;
  size_t            lba_count;
  size_t            lba_block;
  bool              dma_enabled;
  DmaMode           dma_mode;
  VoidCallback      dma_callback;
  size_t            pio_buffer_size;
  size_t            transfer_bytes;
  uint8_t           atapi_command[12];
  bool              atapi_set;
};

struct AtaDriveInfo {
  char serial[21];
  char model[41];
  char version[41];
  uint64_t world_wide_name;
};

struct AtaStorageGeometry {
  size_t sector_size;
  size_t total_sectors;
  size_t sectors_per_cylinder;
  size_t cylinders_per_heads;
  size_t heads;
};

enum AtaStorageType {
  kAtaStorageTypeDisk,
  kAtaStorageTypeCdrom
};

typedef std::function<void(void)> VoidCallback;

class AtaPort {
 public:
  virtual void OnDmaPrepare() = 0;
  virtual void OnDmaTransfer() = 0;
  virtual void OnPioTransfer() = 0;
  virtual void OnCommandDone() = 0;
  virtual ~AtaPort() = default;
};

class AtaStorageDevice : public Device {
 public:
  AtaStorageDevice();
  virtual void Connect();
  virtual void Disconnect();
  virtual void Reset();
  bool IsAvailable();

  virtual void SetSignature(TaskFile* task_file);
  virtual bool StartCommand(TaskFile* task_file);
  virtual void AbortCommand();
  virtual void EndCommand();

  virtual void StartTransfer(TransferType type, VoidCallback end_cb);
  virtual void StopTransfer();
  virtual void WaitForDma(VoidCallback dma_cb);

  inline AtaStorageType     type() { return type_; }
  inline AtaIo*             io() { return &io_; }
  inline void               set_port(AtaPort* port) { port_ = port; }
  inline DiskImage*         image() { return image_; }
  const AtaStorageGeometry& geometry() const { return geometry_; }

 protected:
  virtual void Ata_SetFeatures();

  AtaPort*            port_ = nullptr;
  DiskImage*          image_ = nullptr;
  TaskFile*           task_file_ = nullptr;
  AtaIo               io_;

  AtaStorageType      type_; /* disk or cdrom */

  AtaDriveInfo        drive_info_;
  VoidCallback        ata_handlers_[256];
  bool                write_cache_ = true;
  bool                io_async_ = false;
  VoidCallback        end_transfer_callback_;
  int                 multiple_sectors_;
  AtaStorageGeometry  geometry_;
};


#endif // _MVISOR_DEVICES_AHCI_ATA_STORAGE_H
