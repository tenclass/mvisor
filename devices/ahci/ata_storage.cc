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

#include "ata_storage.h"

#include <cstring>

#include "logger.h"
#include "device_manager.h"
#include "ahci_port.h"


AtaStorageDevice::AtaStorageDevice() {
  set_default_parent_class("Ich9Ahci", "Piix3Ide");

  image_ = nullptr;
  bzero(&drive_info_, sizeof(drive_info_));
  bzero(&geometry_, sizeof(geometry_));

  multiple_sectors_ = 16;

  ata_handlers_[0x00] = [=] () { // NOP
    MV_PANIC("nop");
  };
  
  ata_handlers_[0x08] = [=] () { // ATA_CMD_DEVICE_RESET
    SetSignature(task_file_);
    task_file_->status |= ATA_CB_STAT_BSY;
  };
  
  ata_handlers_[0x2F] = [=] () { // READ_LOG
    AbortCommand();
  };

  ata_handlers_[0xE0] = [=] () { // STANDBYNOW1
  };
  
  ata_handlers_[0xEF] = [=] () { // ATA_CMD_SET_FEATURES
    Ata_SetFeatures();
  };
}

void AtaStorageDevice::Disconnect() {
  if (image_) {
    delete image_;
    image_ = nullptr;
  }
  Device::Disconnect();
}

void AtaStorageDevice::Connect() {
  Device::Connect();

  /* Connect to backend image */
  bool readonly = has_key("readonly") && std::get<bool>(key_values_["readonly"]);
  bool snapshot = has_key("snapshot") && std::get<bool>(key_values_["snapshot"]);
  if (type_ == kAtaStorageTypeCdrom) {
    readonly = true;
  }
  if (has_key("image")) {
    auto path = std::get<std::string>(key_values_["image"]);
    image_ = DiskImage::Create(dynamic_cast<Device*>((Object*)this->parent()), this, path, readonly, snapshot);
  }
}

/* Windows checks CD-Rom status every second. It wastes CPU cycles.
 * We return false if image is not inserted but OS cannot detect the device at all.
 * Maybe we should support IDE hotplug?
 */
bool AtaStorageDevice::IsAvailable() {
  return image_ != nullptr;
}

void AtaStorageDevice::WaitForDma(VoidCallback dma_cb) {
  io_async_ = true;
  io_.dma_enabled = true;
  io_.dma_callback = std::move(dma_cb);
  port_->OnDmaPrepare();
}

void AtaStorageDevice::StartTransfer(TransferType type, VoidCallback end_cb) {
  io_async_ = true;
  io_.type = type;

  end_transfer_callback_ = std::move(end_cb);

  /* We remove BSY from status here, ensure that OS cannot touch the status until
   * the transfer function completes */
  MV_ASSERT(task_file_->status & ATA_CB_STAT_BSY);
  task_file_->status = (task_file_->status | ATA_CB_STAT_SKC | ATA_CB_STAT_DRQ) & ~ATA_CB_STAT_BSY;

  if (type != kTransferAtapiCommand && io_.dma_enabled) {
    // MV_LOG("%x DMA transfer data %lu status %x", task_file_->command, io_.transfer_bytes, task_file_->status);
    port_->OnDmaTransfer();
  } else {
    // MV_LOG("%x PIO transfer data %lu status %x", task_file_->command, io_.transfer_bytes, task_file_->status);
    port_->OnPioTransfer();
  }
}

void AtaStorageDevice::StopTransfer() {
  io_async_ = false;
  io_.transfer_bytes = 0;

  MV_ASSERT(task_file_->status & ATA_CB_STAT_DRQ);
  task_file_->status = (task_file_->status | ATA_CB_STAT_BSY) & ~ATA_CB_STAT_DRQ;

  auto cb = std::move(end_transfer_callback_);
  cb();

  if (!io_async_ && (task_file_->status & ATA_CB_STAT_BSY)) {
    EndCommand();
  }
}

void AtaStorageDevice::EndCommand() {
  MV_ASSERT(task_file_->status & ATA_CB_STAT_BSY);

  task_file_->status &= ~ATA_CB_STAT_BSY;
  port_->OnCommandDone();
}

bool AtaStorageDevice::StartCommand(TaskFile* tf) {
  MV_ASSERT(IsAvailable());

  task_file_ = tf;
  task_file_->error = 0;
  task_file_->status = ATA_CB_STAT_RDY | ATA_CB_STAT_BSY;

  io_.dma_enabled = false;
  io_.transfer_bytes = 0;
  io_async_ = false;

  auto handler = ata_handlers_[task_file_->command];
  if (handler) {
    handler();
  } else {
    AbortCommand();
    if (debug_) {
      MV_ERROR("%s unknown command 0x%x", name_, task_file_->command);
    }
  }

  if (!io_async_ && (task_file_->status & ATA_CB_STAT_BSY)) {
    EndCommand();
  }
  return io_async_;
}

/* Set Error and end this command */
void AtaStorageDevice::AbortCommand() {
  task_file_->status |= ATA_CB_STAT_ERR;
  task_file_->error = ATA_CB_ER_ABRT;
}

void AtaStorageDevice::Reset() {
  Device::Reset();

  if (type_ == kAtaStorageTypeCdrom) {
    io_.pio_buffer_size = 2048;
  } else {
    io_.pio_buffer_size = 512;
  }
}

void AtaStorageDevice::SetSignature(TaskFile* tf) {
  tf->device &= ~0xF;
  tf->count0 = 1;
  tf->lba0 = 1;
  tf->error = 1;
  if (type_ == kAtaStorageTypeCdrom) {
    tf->lba1 = 0x14;
    tf->lba2 = 0xEB;
    tf->status = 0;
  } else if (image_) {
    tf->lba1 = 0;
    tf->lba2 = 0;
    tf->status = ATA_CB_STAT_RDY | ATA_CB_STAT_SKC;
  } else {
    tf->lba1 = 0xFF;
    tf->lba2 = 0xFF;
  }
}

void AtaStorageDevice::Ata_SetFeatures() {
  switch (task_file_->feature0)
  {
  case 0x02: // enable write cache
    write_cache_ = true;
    break;
  case 0x82: // disable write cache
    write_cache_ = false;
    break;
  case 0x03: { // set transfer mode
    switch (task_file_->count0 >> 3)
    {
    case 0: // PIO default
    case 1: // PIO
      io_.dma_mode = kPioMode;
      break;
    case 2: // Single world DMA
      io_.dma_mode = kSingleWordDmaMode;
      break;
    case 4: // MDMA
      io_.dma_mode = kMDmaMode;
      break;
    case 8: // UDMA
      io_.dma_mode = kUDmaMode;
      break;
    default:
      MV_PANIC("unknown trasfer mode 0x%x", task_file_->count0);
      break;
    }
    if (debug_) {
      MV_LOG("%s switch to DMA mode 0x%x", name_, io_.dma_mode);
    }
    break;
  }
  case 0xcc: /* reverting to power-on defaults enable */
  case 0x66: /* reverting to power-on defaults disable */
    break;
  default:
    MV_ERROR("unknown set features 0x%x", task_file_->feature0);
    AbortCommand();
    return;
  }
  
  /* success */
  task_file_->status |= ATA_CB_STAT_SKC;
}

