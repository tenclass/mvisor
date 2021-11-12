#include "devices/ide/ide_storage.h"
#include <cstring>
#include "logger.h"
#include "device_manager.h"
#include "devices/ide/ide_port.h"
#include "devices/ide/ata_interval.h"


IdeStorageDevice::IdeStorageDevice(DiskImage* image) : StorageDevice(image) {
  name_ = "ide-storage";
  bzero(&drive_info_, sizeof(drive_info_));
}


void IdeStorageDevice::StartCommand() {
  auto regs = port_->registers();

  MV_LOG("ata command = 0x%x", regs->command);
  switch (regs->command)
  {
  case ATA_CMD_DEVICE_RESET:
    Ata_ResetSignature();
    break;
  case ATA_CMD_IDENTIFY_DEVICE:
    Ata_IdentifyDevice();
    break;
  case ATA_CMD_SET_FEATURES:
    Ata_SetFeatures();
    break;
  case ATA_CMD_NOP:
    MV_PANIC("nop");
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
}

void IdeStorageDevice::EndCommand() {
  auto regs = port_->registers();
  regs->status = ATA_SR_DRDY;
  port_->RaiseIrq();
}

void IdeStorageDevice::StartTransfer(IdeTransferType type) {
  auto regs = port_->registers();
  auto io = port_->io();
  io->position = 0;
  io->transfer_type = type;
  regs->lba1 = io->nbytes;
  regs->lba2 = io->nbytes >> 8;
  regs->status |= ATA_SR_DRQ;
  port_->RaiseIrq();
}

void IdeStorageDevice::EndTransfer(IdeTransferType type) {
  auto regs = port_->registers();
  auto io = port_->io();
  regs->status &= ~ATA_SR_DRQ;
  io->position = 0; /* reset position */
}


void IdeStorageDevice::BindPort(IdePort* port) {
  port_ = port;

  auto regs = port_->registers();
  regs->status = ATA_SR_DRDY;
  Ata_ResetSignature();
}

void IdeStorageDevice::Ata_ResetSignature() {
  auto regs = port_->registers();
  regs->devsel = ~0xF;
  regs->sectors0 = 1;
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
  switch (regs->features)
  {
  case 0x03:
    AbortCommand();
    break;
    switch (regs->sectors0)
    {
    case 0 ... 15: // PIO
      MV_PANIC("unhandled PIO mode");
      break;
    case 32 ... 39: // MDMA
      MV_PANIC("unhandled MDMA mode 0x%x 0x%x", regs->sectors0, 16 << (regs->sectors0 & 0b111));
      break;
    case 64 ... 113: // UDMA
      MV_PANIC("unhandled UDMA mode 0x%x 0x%x", regs->sectors0, 16 << (regs->sectors0 & 0b111));
      break;
    default:
      MV_PANIC("unknown trasfer mode 0x%x", regs->sectors0);
      break;
    }
    break;
  default:
    MV_LOG("unknown set features 0x%x", regs->features);
    break;
  }
}

