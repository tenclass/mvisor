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

#include "ide_storage.h"
#include <cstring>
#include "logger.h"
#include "device_manager.h"
#include "ahci_port.h"
#include "ata_interval.h"


IdeStorageDevice::IdeStorageDevice() {
  set_parent_name("ahci-host");
  image_ = nullptr;
  bzero(&drive_info_, sizeof(drive_info_));
  bzero(&regs_, sizeof(regs_));

  ata_handlers_[0x00] = [=] () { // NOP
    MV_PANIC("nop");
  };
  
  ata_handlers_[0x08] = [=] () { // ATA_CMD_DEVICE_RESET
    regs_.error &= ~ATA_CB_ER_BBK;
    regs_.error = ATA_CB_ER_NDAM;
    regs_.status = 0; // ?
    Ata_ResetSignature();
  };
  
  ata_handlers_[0x2F] = [=] () { // READ_LOG
    AbortCommand();
  };
  
  ata_handlers_[0xEC] = [=] () { // ATA_CMD_IDENTIFY_DEVICE
    if (image_ && type_ != kIdeStorageTypeCdrom) {
      Ata_IdentifyDevice();
    } else {
      if (type_ == kIdeStorageTypeCdrom) {
        Ata_ResetSignature();
      }
      AbortCommand();
    }
  };
  
  ata_handlers_[0xEF] = [=] () { // ATA_CMD_SET_FEATURES
    Ata_SetFeatures();
  };
}

void IdeStorageDevice::Disconnect() {
  if (image_) {
    delete image_;
    image_ = nullptr;
  }
  Device::Disconnect();
}

void IdeStorageDevice::Connect() {
  Device::Connect();

  /* Connect to backend image */
  bool readonly = type_ == kIdeStorageTypeCdrom;
  if (has_key("readonly")) {
    readonly = std::get<bool>(key_values_["readonly"]);
  }
  if (has_key("image")) {
    std::string path = std::get<std::string>(key_values_["image"]);
    image_ = DiskImage::Create(this, path, readonly);
  }
}

/* Windows checks CD-Rom status every second. It wastes CPU cycles.
 * We return false if image is not inserted but OS cannot detect the device at all.
 * Maybe we should support IDE hotplug?
 */
bool IdeStorageDevice::IsAvailable() {
  return image_ != nullptr;
}

void IdeStorageDevice::CompleteCommand() {
  regs_.status &= ~ATA_SR_BSY;
  io_async_ = false;
  io_complete_();
}

bool IdeStorageDevice::StartCommand(VoidCallback iocp) {
  MV_ASSERT(IsAvailable());

  regs_.error = 0;
  io_.dma_status = 0;
  io_.nbytes = 0;
  
  if (regs_.status & (ATA_SR_BSY)) {
    if (regs_.command != ATA_CMD_DEVICE_RESET || type_ != kIdeStorageTypeCdrom) {
      MV_PANIC("%s invalid command 0x%x while busy, async=%d", name_, regs_.command, io_async_);
      return false;
    }
  }

  io_complete_ = iocp;
  io_async_ = false;
  regs_.status = ATA_SR_DRDY | ATA_SR_BSY;

  auto handler = ata_handlers_[regs_.command];
  if (handler) {
    handler();
    if (!io_async_) {
      CompleteCommand();
    }
  } else {
    MV_PANIC("unknown command 0x%x", regs_.command);
  }
  return io_async_;
}

/* Set Error and end this command */
void IdeStorageDevice::AbortCommand() {
  regs_.status = ATA_SR_DRDY | ATA_SR_ERR;
  regs_.error = ATA_CB_ER_ABRT;
}

void IdeStorageDevice::Reset() {
  Ata_ResetSignature();
  regs_.status = ATA_SR_DSC | ATA_SR_DF;
  if (type_ == kIdeStorageTypeCdrom) {
    regs_.status |= ATA_SR_DRDY;
  }
  regs_.error = ATA_CB_ER_NDAM;
}

void IdeStorageDevice::Ata_ResetSignature() {
  regs_.device = ~0xF;
  regs_.count0 = 1;
  regs_.lba0 = 1;
  if (type_ == kIdeStorageTypeCdrom) {
    regs_.lba1 = 0x14;
    regs_.lba2 = 0xEB;
  } else if (image_) {
    regs_.lba1 = 0;
    regs_.lba2 = 0;
  } else {
    regs_.lba1 = 0xFF;
    regs_.lba2 = 0xFF;
  }
}

void IdeStorageDevice::Ata_IdentifyDevice() {
  MV_PANIC("Not implemented. Harddisk should override this.");
}

void IdeStorageDevice::Ata_SetFeatures() {
  switch (regs_.feature0)
  {
  case 0x02: // enable write cache
    write_cache_ = true;
    break;
  case 0x82: // disable write cache
    write_cache_ = false;
    break;
  case 0x03: { // set transfer mode
    uint8_t value = regs_.count0 & 0b111;
    switch (regs_.count0 >> 3)
    {
    case 0: // PIO default
    case 1: // PIO
      MV_PANIC("not supported PIO mode");
      break;
    case 2: // Single world DMA
      MV_PANIC("not supported Single world DMA mode");
      break;
    case 4: // MDMA
      MV_PANIC("not supported MDMA mode");
      break;
    case 8: // UDMA
      if (debug_) MV_LOG("udma = %x", value);
      break;
    default:
      MV_PANIC("unknown trasfer mode 0x%x", regs_.count0);
      break;
    }
    break;
  }
  case 0xcc: /* reverting to power-on defaults enable */
  case 0x66: /* reverting to power-on defaults disable */
    break;
  default:
    MV_LOG("unknown set features 0x%x", regs_.feature0);
    AbortCommand();
    break;
  }
}

