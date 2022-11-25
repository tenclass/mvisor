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

#include "ahci_port.h"

#include <cstring>

#include "logger.h"
#include "device_manager.h"
#include "ahci_host.h"
#include "ahci_internal.h"
#include "ata_storage.h"


AhciPort::AhciPort(DeviceManager* manager, AhciHost* host, int index)
  : manager_(manager), host_(host), port_index_(index)
{
  bzero(&port_control_, sizeof(port_control_));
}

AhciPort::~AhciPort() {
}

void AhciPort::AttachDevice(AtaStorageDevice* device) {
  drive_ = device;
  drive_->set_port(this);
}

void AhciPort::Reset() {
  port_control_.command_issue = 0;
  port_control_.irq_status = 0;
  port_control_.irq_mask = 0;
  port_control_.sata_control = 0;
  port_control_.command = PORT_CMD_SPIN_UP | PORT_CMD_POWER_ON;
  SoftReset();
}

void AhciPort::SoftReset() {
  port_control_.sata_status = 0;
  port_control_.sata_error = 0;
  port_control_.sata_active = 0;
  port_control_.task_flie_data = 0x7F;
  port_control_.signature = 0xFFFFFFFF;
  init_d2h_sent_ = false;
  busy_slot_ = -1;

  bzero(&task_file_, sizeof(task_file_));

  if (!drive_) {
    return;
  }
  if (host_->debug())
    MV_LOG("%s port reseted", drive_->name());

  drive_->Reset();
  drive_->SetSignature(&task_file_);
}

void AhciPort::UpdateInitD2H() {
  if (init_d2h_sent_) {
    return;
  }
  init_d2h_sent_ = true;

  UpdateFisRegisterD2H();

  if (!drive_) {
    return;
  }

  switch (drive_->type())
  {
  case kAtaStorageTypeCdrom:
    port_control_.signature = ATA_SIGNATURE_CDROM;
    break;
  case kAtaStorageTypeDisk:
    port_control_.signature = ATA_SIGNATURE_DISK;
    break;
  }
}

void AhciPort::SetNcqError(int slot) {
  task_file_.error = ATA_CB_ER_ABRT;
  task_file_.status = ATA_CB_STAT_RDY | ATA_CB_STAT_ERR;
  port_control_.sata_error |= (1 << slot);
}

void AhciPort::HandleNcqCommand(int slot, void* fis) {
  auto frame = (AhciNcqFrame*)fis;
  MV_ASSERT((frame->tag >> 3) == slot);

  size_t lba =  ((size_t)frame->lba5 << 40) |
                ((size_t)frame->lba4 << 32) |
                ((size_t)frame->lba3 << 24) |
                ((size_t)frame->lba2 << 16) |
                ((size_t)frame->lba1 << 8)  |
                ((size_t)frame->lba0);
  size_t sector_count = (size_t(frame->count1) << 8) | frame->count0;
  if (!sector_count) {
    sector_count = 0x10000;
  }

  auto image = drive_->image();
  auto sector_size = drive_->geometry().sector_size;

  ImageIoRequest request = {
    .position = sector_size * lba,
    .length = sector_size * sector_count,
    .vector = std::move(current_dma_vector_)
  };

  switch (frame->command) {
    case 0x60: // READ_FPDMA_QUEUED
      request.type = kImageIoRead;
      break;
    case 0x61: // WRITE_FPDMA_QUEUED
      request.type = kImageIoWrite;
      break;
    default:
      SetNcqError(slot);
      return;
  }

  if (host_->debug()) {
    MV_LOG("NCQ slot %d %s lba=%lu count=%lu", slot, 
      request.type == kImageIoRead ? "read" : "write", lba, sector_count);
  }

  image->QueueIoRequest(request, [this, slot](auto ret) {
    if (ret < 0) {
      SetNcqError(slot);
    } else {
      task_file_.status = ATA_CB_STAT_RDY | ATA_CB_STAT_SKC;
      task_file_.error = 0;
    }
    UpdateFisSetDeviceBits(slot);
  });
}

/* return true if we have done handling the command */
bool AhciPort::HandleCommand(int slot) {
  MV_ASSERT(command_list_);
  MV_ASSERT(drive_);

  current_command_ = &((AhciCommandHeader*)command_list_)[slot];
  auto command_table = (AhciCommandTable*)manager_->TranslateGuestMemory(current_command_->command_table_base);

  auto fis = (AhciFisRegH2D*)command_table->command_fis;
  if (fis->fis_type != kAhciFisTypeRegH2D) {
    MV_ERROR("unknown fis type 0x%x", fis->fis_type);
    return true;
  }

  if (!fis->is_command) {
    MV_ERROR("not a command fis, control=%x", fis->control);
    return true;
  }

  if (fis->command >= 0x60 && fis->command <= 0x65) {
    ParseDmaVector(command_table->prdt_entries, current_command_->prdt_length, fis->command == 0x60);
    HandleNcqCommand(slot, fis);
    return true;
  }

  /* Copy IDE command parameters */
  auto io = drive_->io();
  task_file_.command = fis->command;
  task_file_.feature0 = fis->feature0;
  task_file_.lba0 = fis->lba0;
  task_file_.lba1 = fis->lba1;
  task_file_.lba2 = fis->lba2;
  task_file_.device = fis->device;
  task_file_.lba3 = fis->lba3;
  task_file_.lba4 = fis->lba4;
  task_file_.lba5 = fis->lba5;
  task_file_.control = fis->feature1;
  task_file_.count0 = fis->count0;
  task_file_.count1 = fis->count1;

  /* Copy the ACMD field (ATAPI packet, if any) from the AHCI command
   * table to ide_state->io_buffer */
  if (current_command_->is_atapi) {
    memcpy(io->atapi_command, command_table->atapi_command, sizeof(io->atapi_command));
    io->atapi_set = true;
  }

  bool is_write = fis->command == 0xC8 || io->atapi_command[0] == 0x28;
  ParseDmaVector(command_table->prdt_entries, current_command_->prdt_length, is_write);

  /* We have only one DMA engine each drive.
   * when async IO is running by IO thread, we should wait for the slot */
  MV_ASSERT(busy_slot_ == -1);

  io->vector = std::move(current_dma_vector_);
  if (io->vector.empty()) {
    io->buffer = nullptr;
    io->buffer_size = 0;
  } else {
    io->buffer = (uint8_t*)io->vector.front().iov_base;
    io->buffer_size = io->vector.front().iov_len;
  }

  bool should_wait = drive_->StartCommand(&task_file_);
  if (should_wait) { // BUSY
    busy_slot_ = slot;
    return false;
  }
  return true;
}

/* current_dma_vector_ contains a shadow copy to the PRDT (physical region descriptor table)
 * If prdt_length is zero, the function only clears the vector.
 * is_write=true means this dma vector is used for writing into.
 */
void AhciPort::ParseDmaVector(AhciPrdtEntry* entries, uint16_t prdt_length, bool is_write) {
  current_dma_vector_.clear();

  for (int prdt_index = 0; prdt_index < prdt_length; prdt_index++) {
    void* host = manager_->TranslateGuestMemory(entries[prdt_index].address);
    size_t length = entries[prdt_index].size + 1;
    if (is_write) {
      manager_->AddDirtyMemory(entries[prdt_index].address, length);
    }
    current_dma_vector_.emplace_back(iovec { .iov_base = host, .iov_len = length });
  }
}

void AhciPort::OnDmaPrepare() {
  auto io = drive_->io();
  /* We have already built io->dma_vector before StartCommand() */
  auto cb = std::move(io->dma_callback);
  cb();
}

void AhciPort::OnDmaTransfer() {
  auto io = drive_->io();

  current_command_->bytes_transferred += io->transfer_bytes;
  drive_->StopTransfer();
}

void AhciPort::OnPioTransfer() {
  auto io = drive_->io();

  if (io->transfer_bytes > 0) {
    UpdateFisSetupPio();
  }
  current_command_->bytes_transferred += io->transfer_bytes;
  drive_->StopTransfer();
}

void AhciPort::OnCommandDone() {
  UpdateFisRegisterD2H();

  if (busy_slot_ != -1) {
    port_control_.command_issue &= ~(1U << busy_slot_);
    busy_slot_ = -1;
    current_command_ = nullptr;
    /* Check next command */
    CheckCommand();
  }
}

void AhciPort::CheckCommand() {
  if (busy_slot_ != -1) {
    return;
  }
  if ((port_control_.command & PORT_CMD_START) && port_control_.command_issue) {
    for (int slot = 0; (slot < 32) && port_control_.command_issue; slot++) {
      if (port_control_.command_issue & (1U << slot)) {
        if (HandleCommand(slot)) {
          port_control_.command_issue &= ~(1U << slot);
        } else {
          /* Stop executing other commands if an async command is running */
          return;
        }
      }
    }
  }
}

void AhciPort::Read(uint64_t offset, uint8_t* data, uint32_t size) {
  AhciPortReg reg_index = (AhciPortReg)(offset / sizeof(uint32_t));
  MV_ASSERT(reg_index < 32);
  if (reg_index == kAhciPortRegSataStatus) {
    if (drive_ && drive_->IsAvailable()) {
      port_control_.sata_status = ATA_SCR_SSTATUS_DET_DEV_PRESENT_PHY_UP |  // Physical communication established
        ATA_SCR_SSTATUS_SPD_GEN1 |                      // Speed
        ATA_SCR_SSTATUS_IPM_ACTIVE;                     // Full power
    } else {
      port_control_.sata_status = ATA_SCR_SSTATUS_DET_NODEV;  // No device
    }
  }
  memcpy(data, (uint8_t*)&port_control_ + offset, size);
  if (host_->debug()) {
    MV_LOG("%d:%s read port index=%d ret=0x%x", port_index_, drive_ ? drive_->name() : "", reg_index, *data);
  }
}

void AhciPort::CheckEngines() {
  bool cmd_start = port_control_.command & PORT_CMD_START;
  bool cmd_on    = port_control_.command & PORT_CMD_LIST_ON;
  bool fis_start = port_control_.command & PORT_CMD_FIS_RX;
  bool fis_on    = port_control_.command & PORT_CMD_FIS_ON;

  MV_ASSERT(manager_);
  if (cmd_start && !cmd_on) {
    command_list_ = (uint8_t*)manager_->TranslateGuestMemory(
      ((uint64_t)port_control_.command_list_base1 << 32) | port_control_.command_list_base0);
    if (command_list_ != nullptr) {
      port_control_.command |= PORT_CMD_LIST_ON;
    } else {
      port_control_.command &= ~(PORT_CMD_START | PORT_CMD_LIST_ON);
      return;
    }
  } else if (!cmd_start && cmd_on) {
    port_control_.command &= ~PORT_CMD_LIST_ON;
    command_list_ = nullptr;
  }

  if (fis_start && !fis_on) {
    rx_fis_ = (AhciRxFis*)manager_->TranslateGuestMemory
      (((uint64_t)port_control_.fis_base1 << 32) | port_control_.fis_base0);
    if (rx_fis_ != nullptr) {
      port_control_.command |= PORT_CMD_FIS_ON;
    } else {
      port_control_.command &= ~(PORT_CMD_FIS_RX | PORT_CMD_FIS_ON);
      return;
    }
  } else if (!fis_start && fis_on) {
    port_control_.command &= ~PORT_CMD_FIS_ON;
    rx_fis_ = nullptr;
  }
}

void AhciPort::Write(uint64_t offset, uint32_t value) {
  AhciPortReg reg_index = (AhciPortReg)(offset / sizeof(uint32_t));
  MV_ASSERT(reg_index < 32);
  if (host_->debug()) {
    MV_LOG("%d:%s write port index=%d value=0x%x", port_index_, drive_ ? drive_->name() : "", reg_index, value);
  }

  switch (reg_index)
  {
  case kAhciPortRegCommandListBase0:
    port_control_.command_list_base0 = value;
    break;
  case kAhciPortRegCommandListBase1:
    port_control_.command_list_base1 = value;
    break;
  case kAhciPortRegReceivedFisBase0:
    port_control_.fis_base0 = value;
    break;
  case kAhciPortRegReceivedFisBase1:
    port_control_.fis_base1 = value;
    break;
  case kAhciPortRegIrqStatus:
    port_control_.irq_status &= ~value;
    host_->CheckIrq();
    break;
  case kAhciPortRegIrqMask:
    port_control_.irq_mask = value & 0xFDC000FF;
    host_->CheckIrq();
    break;
  case kAhciPortRegCommand:
    /* Block any Read-only fields from being set;
     * including LIST_ON and FIS_ON.
     * The spec requires to set ICC bits to zero after the ICC change
     * is done. We don't support ICC state changes, therefore always
     * force the ICC bits to zero.
     */
    port_control_.command = (port_control_.command & PORT_CMD_RO_MASK) |
      (value & ~(PORT_CMD_RO_MASK | PORT_CMD_ICC_MASK));

    /* Check FIS RX and CLB engines */
    CheckEngines();

    /* XXX usually the FIS would be pending on the bus here and
     * issuing deferred until the OS enables FIS receival.
     * Instead, we only submit it once - which works in most
     * cases, but is a hack. */
    if (port_control_.command & PORT_CMD_FIS_ON) {
      UpdateInitD2H();
    }

    CheckCommand();
    break;
  case kAhciPortRegTaskFileData:
  case kAhciPortRegSignature:
  case kAhciPortRegSataStatus:
    /* Read Only */
    break;
  case kAhciPortRegSataControl:
    if (((port_control_.sata_control & AHCI_SCR_SCTL_DET) == 1) && ((value & AHCI_SCR_SCTL_DET) == 0)) {
      SoftReset();
    }
    port_control_.sata_control = value;
    break;
  case kAhciPortRegSataError:
    port_control_.sata_error &= ~value;
    break;
  case kAhciPortRegSataActive:
    /* RW1 */
    port_control_.sata_active |= value;
    break;
  case kAhciPortRegCommandIssue:
    port_control_.command_issue |= value;
    CheckCommand();
    break;
  default:
    MV_ERROR("not implemented reg index = %x", reg_index);
  }
}

void AhciPort::UpdateFisRegisterD2H() {
  MV_ASSERT(rx_fis_ && port_control_.command & PORT_CMD_FIS_RX);
  auto d2h_fis = &rx_fis_->d2h_fis;
  bzero(d2h_fis, sizeof(*d2h_fis));

  d2h_fis->fis_type = kAhciFisTypeRegD2H;
  d2h_fis->interrupt = 1;
  d2h_fis->status = task_file_.status;
  d2h_fis->error = task_file_.error;

  d2h_fis->lba0 = task_file_.lba0;
  d2h_fis->lba1 = task_file_.lba1;
  d2h_fis->lba2 = task_file_.lba2;
  d2h_fis->device = task_file_.device;
  d2h_fis->lba3 = task_file_.lba3;
  d2h_fis->lba4 = task_file_.lba4;
  d2h_fis->lba5 = task_file_.lba5;
  d2h_fis->count0 = task_file_.count0;
  d2h_fis->count1 = task_file_.count1;

  port_control_.task_flie_data = (task_file_.error << 8) | (task_file_.status);
  if (task_file_.status & ATA_CB_STAT_ERR) {
    TrigerIrq(kAhciPortIrqBitTaskFileError);
  }
  TrigerIrq(kAhciPortIrqBitDeviceToHostFis);
}

void AhciPort::UpdateFisSetupPio() {
  MV_ASSERT(rx_fis_ && port_control_.command & PORT_CMD_FIS_RX);
  auto pio_fis = &rx_fis_->pio_fis;
  bzero(pio_fis, sizeof(*pio_fis));

  auto io = drive_->io();

  pio_fis->fis_type = kAhciFisTypePioSetup;
  pio_fis->interrupt = 1;
  pio_fis->status = task_file_.status;
  pio_fis->error = task_file_.error;

  pio_fis->lba0 = task_file_.lba0;
  pio_fis->lba1 = task_file_.lba1;
  pio_fis->lba2 = task_file_.lba2;
  pio_fis->device = task_file_.device;
  pio_fis->lba3 = task_file_.lba3;
  pio_fis->lba4 = task_file_.lba4;
  pio_fis->lba5 = task_file_.lba5;
  pio_fis->count0 = task_file_.count0;
  pio_fis->count1 = task_file_.count1;
  pio_fis->e_status = task_file_.status;
  pio_fis->transfer_count = io->transfer_bytes;

  port_control_.task_flie_data = (task_file_.error << 8) | (task_file_.status);

  if (task_file_.status & ATA_CB_STAT_ERR) {
    TrigerIrq(kAhciPortIrqBitTaskFileError);
  }
  TrigerIrq(kAhciPortIrqBitPioSetupFis);
}

void AhciPort::UpdateFisSetDeviceBits(int slot) {
  MV_ASSERT(rx_fis_ && port_control_.command & PORT_CMD_FIS_RX);
  auto sdb_fis = &rx_fis_->sdb_fis;

  sdb_fis->type = kAhciFisTypeDeviceBits;
  sdb_fis->flags = 0x40; // interrupt
  sdb_fis->status = task_file_.status;
  sdb_fis->error = task_file_.error;

  if (!(port_control_.sata_error & (1 << slot))) {
    sdb_fis->payload = 1 << slot;
  } else {
    sdb_fis->payload = 0;
  }

  port_control_.sata_active &= ~sdb_fis->payload;
  port_control_.task_flie_data = (task_file_.error << 8) | (task_file_.status);

  TrigerIrq(kAhciPortIrqBitSetDeviceBitsFis);
}

void AhciPort::TrigerIrq(int irqbit) {
  MV_ASSERT(irqbit < 32);
  uint32_t irq = 1U << irqbit;
  uint32_t irq_status = port_control_.irq_status | irq;
  port_control_.irq_status = irq_status;
  host_->CheckIrq();
}

void AhciPort::SaveState(AhciHostState_PortState* port_state) {
  /* Make sure no commands are running */
  MV_ASSERT(port_control_.command_issue == 0);

  port_state->set_index(port_index_);
  port_state->set_busy_slot(busy_slot_);
  port_state->set_init_d2h_sent(init_d2h_sent_);
  auto regs = port_state->mutable_registers();
  regs->set_command_list_base0(port_control_.command_list_base0);
  regs->set_command_list_base1(port_control_.command_list_base1);
  regs->set_fis_base0(port_control_.fis_base0);
  regs->set_fis_base1(port_control_.fis_base1);
  regs->set_irq_status(port_control_.irq_status);
  regs->set_irq_mask(port_control_.irq_mask);
  regs->set_command(port_control_.command);
  regs->set_task_flie_data(port_control_.task_flie_data);
  regs->set_signature(port_control_.signature);
  regs->set_sata_status(port_control_.sata_status);
  regs->set_sata_control(port_control_.sata_control);
  regs->set_sata_error(port_control_.sata_error);
  regs->set_sata_active(port_control_.sata_active);
}

void AhciPort::LoadState(const AhciHostState_PortState* port_state) {
  port_index_ = port_state->index();
  busy_slot_ = port_state->busy_slot();
  init_d2h_sent_ = port_state->init_d2h_sent();
  auto &regs = port_state->registers();
  port_control_.command_list_base0 = regs.command_list_base0();
  port_control_.command_list_base1 = regs.command_list_base1();
  port_control_.fis_base0 = regs.fis_base0();
  port_control_.fis_base1 = regs.fis_base1();
  port_control_.irq_status = regs.irq_status();
  port_control_.irq_mask = regs.irq_mask();
  port_control_.command = regs.command();
  port_control_.task_flie_data = regs.task_flie_data();
  port_control_.signature = regs.signature();
  port_control_.sata_status = regs.sata_status();
  port_control_.sata_control = regs.sata_control();
  port_control_.sata_error = regs.sata_error();
  port_control_.sata_active = regs.sata_active();

  /* Setup command_list_ and fis_rx_ pointers */
  port_control_.command &= ~(PORT_CMD_LIST_ON | PORT_CMD_FIS_ON);
  CheckEngines();
}
