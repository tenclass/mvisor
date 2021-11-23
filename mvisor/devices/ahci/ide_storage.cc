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
  bzero(&drive_info_, sizeof(drive_info_));
}

IdeStorageDevice::~IdeStorageDevice() {
  
}

void IdeStorageDevice::StartCommand() {
  auto regs = port_->registers();

  switch (regs->command)
  {
  case 0x00: // NOP
    MV_PANIC("nop");
    break;
  case 0x08: // ATA_CMD_DEVICE_RESET
    regs->error &= ~ATA_CB_ER_BBK;
    regs->error = ATA_CB_ER_NDAM;
    regs->status = 0; // ?
    Ata_ResetSignature();
    break;
  case 0x2F: // READ LOG
    AbortCommand();
    break;
  case 0xEC: // ATA_CMD_IDENTIFY_DEVICE
    Ata_IdentifyDevice();
    break;
  case 0xEF: // ATA_CMD_SET_FEATURES
    Ata_SetFeatures();
    break;
  default:
    MV_PANIC("unknown command 0x%x", regs->command);
    break;
  }
}

void IdeStorageDevice::AbortCommand() {
  auto regs = port_->registers();
  regs->status = ATA_SR_DRDY | ATA_SR_ERR;
  regs->error = ATA_CB_ER_ABRT;
  // port_->RaiseIrq();
}

void IdeStorageDevice::EndCommand() {
  auto regs = port_->registers();
  regs->status = ATA_SR_DRDY;
  // port_->RaiseIrq();
}

void IdeStorageDevice::StartTransfer(IdeTransferType type) {
  auto regs = port_->registers();
  auto io = port_->io();
  io->position = 0;
  io->transfer_type = type;
  regs->status |= ATA_SR_DRQ | ATA_SR_DSC;
  // port_->RaiseIrq();
}

void IdeStorageDevice::EndTransfer(IdeTransferType type) {
  auto regs = port_->registers();
  auto io = port_->io();
  regs->status &= ~ATA_SR_DRQ;
  io->position = 0; /* reset position */
}


void IdeStorageDevice::BindPort(AhciPort* port) {
  port_ = port;
}

void IdeStorageDevice::Reset() {
  auto regs = port_->registers();
  regs->status = ATA_SR_DRDY;
  Ata_ResetSignature();
}

void IdeStorageDevice::Ata_ResetSignature() {
  auto regs = port_->registers();
  regs->device = ~0xF;
  regs->count0 = 1;
  regs->lba0 = 1;
  if (type_ == kIdeStorageTypeCdrom) {
    regs->lba1 = 0x14;
    regs->lba2 = 0xEB;
  } else {
    regs->lba1 = 0;
    regs->lba2 = 0;
  }
}

void IdeStorageDevice::Ata_IdentifyDevice() {
  if (type_ != kIdeStorageTypeCdrom) {
    // transfer data
    MV_PANIC("not impl");
  } else {
    if (type_ == kIdeStorageTypeCdrom) {
      Ata_ResetSignature();
    }
    AbortCommand();
  }
}

void IdeStorageDevice::Ata_SetFeatures() {
  auto regs = port_->registers();
  switch (regs->feature0)
  {
  case 0x03: { // set transfer mode
    uint8_t value = regs->count0 & 0b111;
    switch (regs->count0 >> 3)
    {
    case 0: // PIO default
    case 1: // PIO
      MV_PANIC("not supported DMA mode");
      break;
    case 2: // Single world DMA
      MV_PANIC("not supported DMA mode");
      break;
    case 4: // MDMA
      MV_PANIC("not supported DMA mode");
      break;
    case 8: // UDMA
      MV_LOG("udma = %x", value);
      break;
    default:
      MV_PANIC("unknown trasfer mode 0x%x", regs->count0);
      break;
    }
    EndCommand();
    break;
  }
  case 0xcc: /* reverting to power-on defaults enable */
  case 0x66: /* reverting to power-on defaults disable */
    EndCommand();
    break;
  default:
    MV_LOG("unknown set features 0x%x", regs->feature0);
    AbortCommand();
    break;
  }
}

