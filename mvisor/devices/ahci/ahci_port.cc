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
#include "ide_storage.h"
#include "ahci_internal.h"


static inline int is_native_command_queueing(uint8_t ata_cmd)
{
    /* Based on SATA 3.2 section 13.6.3.2 */
    switch (ata_cmd) {
    case 0x60: // READ_FPDMA_QUEUED
    case 0x61: // WRITE_FPDMA_QUEUED
    case 0x63: // NCQ_NON_DATA
    case 0x64: // SEND_FPDMA_QUEUED
    case 0x65: // RECEIVE_FPDMA_QUEUED
        return 1;
    default:
        return 0;
    }
}

AhciPort::AhciPort(DeviceManager* manager, AhciHost* host, int index)
  : manager_(manager), host_(host), port_index_(index)
{
  bzero(&port_control_, sizeof(port_control_));
}

AhciPort::~AhciPort() {
}

void AhciPort::AttachDevice(IdeStorageDevice* device) {
  drive_ = device;
}

void AhciPort::Reset() {
  if (drive_->debug()) {
    MV_LOG("reset, command issue=0x%x", port_control_.command_issue);
  }
  port_control_.command_issue = 0;
  port_control_.irq_status = 0;
  port_control_.irq_mask = 0;
  port_control_.sata_control = 0;
  port_control_.command = PORT_CMD_SPIN_UP | PORT_CMD_POWER_ON;

  port_control_.sata_status = 0;
  port_control_.sata_error = 0;
  port_control_.sata_active = 0;
  port_control_.task_flie_data = 0x7F;
  port_control_.signature = 0xFFFFFFFF;
  init_d2h_sent_ = false;
  busy_slot_ = -1;

  if (!drive_) {
    return;
  }

  drive_->Reset();
}

void AhciPort::UpdateInitD2H() {
  if (init_d2h_sent_) {
    return;
  }
  init_d2h_sent_ = true;

  UpdateRegisterD2H();
  if (drive_->type() == kIdeStorageTypeCdrom) {
    port_control_.signature = ATA_SIGNATURE_CDROM;
  } else {
    port_control_.signature = ATA_SIGNATURE_DISK;
  }
}

/* io->vector contains a shadow copy to the PRDT (physical region descriptor table)
 * io->buffer is always set to the first region of the vector for fast access.
 * If prdt_length is zero, the function only clears the vector.
 */
void AhciPort::PrepareIoVector(AhciPrdtEntry* entries, uint16_t prdt_length) {
  auto io = drive_->io();
  io->vector.clear();
  for (int prdt_index = 0; prdt_index < prdt_length; prdt_index++) {
    void* host = manager_->TranslateGuestMemory(entries[prdt_index].address);
    MV_ASSERT(host);
    size_t length = entries[prdt_index].size + 1;
    io->vector.emplace_back(iovec { .iov_base = host, .iov_len = length });
    if (prdt_index == 0) {
      io->buffer = (uint8_t*)host;
      io->buffer_size = length;
    }
  }
}

bool AhciPort::HandleCommand(int slot) {
  MV_ASSERT(command_list_);
  AhciCommandHeader* command_ = &((AhciCommandHeader*)command_list_)[slot];
  if (drive_ == nullptr) {
    MV_PANIC("bad port %d", port_index_);
    return false;
  }

  AhciCommandTable* command_table = (AhciCommandTable*)manager_->TranslateGuestMemory(command_->command_table_base);
  MV_ASSERT(command_table);
  AhciFisRegH2D* fis = (AhciFisRegH2D*)command_table->command_fis;

  if (fis->fis_type != kAhciFisTypeRegH2D) {
    MV_LOG("unknown fis type 0x%x", fis->fis_type);
    /* done handling the command */
    return true;
  }

  if (!fis->is_command) {
    MV_PANIC("not a command fis");
    return false;
  }

  if (is_native_command_queueing(fis->command)) {
    /* NCQ is not necessary, VirtIO is the best choice */
    MV_PANIC("not supported NCQ yet %x", fis->command);
  }

  /* Copy IDE command parameters */
  auto regs = drive_->regs();
  auto io = drive_->io();
  regs->command = fis->command;
  regs->feature0 = fis->feature0;
  regs->lba0 = fis->lba0;
  regs->lba1 = fis->lba1;
  regs->lba2 = fis->lba2;
  regs->device = fis->device;
  regs->lba3 = fis->lba3;
  regs->lba4 = fis->lba4;
  regs->lba5 = fis->lba5;
  regs->control = fis->feature1;
  regs->count0 = fis->count0;
  regs->count1 = fis->count1;

  /* Copy the ACMD field (ATAPI packet, if any) from the AHCI command
   * table to ide_state->io_buffer */
  if (command_->is_atapi) {
    memcpy(io->atapi_command, command_table->atapi_command, sizeof(io->atapi_command));
  }

  PrepareIoVector(command_table->prdt_entries, command_->prdt_length);

  /* We have only one DMA engine each drive.
   * when async IO is running by IO thread, we should wait for the slot */
  drive_->StartCommand([this, io, command_, slot, regs]() {
    if (io->nbytes <= 0 || io->dma_status) {
      UpdateRegisterD2H();
    } else {
      UpdateSetupPio();
    }
    command_->bytes_transferred = io->nbytes;

    if (busy_slot_ != -1) {
      port_control_.command_issue &= ~(1U << busy_slot_);
      busy_slot_ = -1;
      /* Check next command */
      CheckCommand();
    }
  });

  if (regs->status & 0x80) { // BUSY
    busy_slot_ = slot;
    return false;
  }
  return true;
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

void AhciPort::Read(uint64_t offset, uint32_t* data) {
  AhciPortReg reg_index = (AhciPortReg)(offset / sizeof(uint32_t));
  MV_ASSERT(reg_index < 32);
  if (reg_index == kAhciPortRegSataStatus) {
    if (drive_ && drive_->IsAvailable()) {
      *data = ATA_SCR_SSTATUS_DET_DEV_PRESENT_PHY_UP |  // Physical communication established
        ATA_SCR_SSTATUS_SPD_GEN1 |                      // Speed
        ATA_SCR_SSTATUS_IPM_ACTIVE;                     // Full power
    } else {
      *data = ATA_SCR_SSTATUS_DET_NODEV;  // No device
    }
  } else {
    *data = *((uint32_t*)&port_control_ + reg_index);
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
    port_control_.irq_mask = value & 0xfdc000ff;
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

    manager_->io()->Schedule([this](){
      /* XXX usually the FIS would be pending on the bus here and
        issuing deferred until the OS enables FIS receival.
        Instead, we only submit it once - which works in most
        cases, but is a hack. */
      if ((port_control_.command & PORT_CMD_FIS_ON) && !init_d2h_sent_) {
        UpdateInitD2H();
      }
      CheckCommand();
    });
    break;
  case kAhciPortRegTaskFileData:
  case kAhciPortRegSignature:
  case kAhciPortRegSataStatus:
    /* Read Only */
    break;
  case kAhciPortRegSataControl:
    if (((port_control_.sata_control & AHCI_SCR_SCTL_DET) == 1) &&
        ((value & AHCI_SCR_SCTL_DET) == 0)) {
      Reset();
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
    manager_->io()->Schedule([this](){
      CheckCommand();
    });
    break;
  default:
    MV_PANIC("not implemented reg index = %x", reg_index);
  }
}

void AhciPort::UpdateRegisterD2H() {
  MV_ASSERT(rx_fis_ && port_control_.command & PORT_CMD_FIS_RX);
  auto d2h_fis = &rx_fis_->d2h_fis;
  bzero(d2h_fis, sizeof(*d2h_fis));

  auto regs = drive_->regs();

  d2h_fis->fis_type = kAhciFisTypeRegD2H;
  d2h_fis->interrupt = 1;
  d2h_fis->status = regs->status;
  d2h_fis->error = regs->error;

  d2h_fis->lba0 = regs->lba0;
  d2h_fis->lba1 = regs->lba1;
  d2h_fis->lba2 = regs->lba2;
  d2h_fis->device = regs->device;
  d2h_fis->lba3 = regs->lba3;
  d2h_fis->lba4 = regs->lba4;
  d2h_fis->lba5 = regs->lba5;
  d2h_fis->count0 = regs->count0;
  d2h_fis->count1 = regs->count1;

  port_control_.task_flie_data = (regs->error << 8) | (regs->status);
  if (regs->status & 1) {
    TrigerIrq(kAhciPortIrqBitTaskFileError);
  }
  TrigerIrq(kAhciPortIrqBitDeviceToHostFis);
}

void AhciPort::UpdateSetupPio() {
  MV_ASSERT(rx_fis_ && port_control_.command & PORT_CMD_FIS_RX);
  auto pio_fis = &rx_fis_->pio_fis;
  bzero(pio_fis, sizeof(*pio_fis));

  auto regs = drive_->regs();
  auto io = drive_->io();

  pio_fis->fis_type = kAhciFisTypePioSetup;
  pio_fis->interrupt = 1;
  pio_fis->status = regs->status;
  pio_fis->error = regs->error;

  pio_fis->lba0 = regs->lba0;
  pio_fis->lba1 = regs->lba1;
  pio_fis->lba2 = regs->lba2;
  pio_fis->device = regs->device;
  pio_fis->lba3 = regs->lba3;
  pio_fis->lba4 = regs->lba4;
  pio_fis->lba5 = regs->lba5;
  pio_fis->count0 = regs->count0;
  pio_fis->count1 = regs->count1;
  pio_fis->e_status = regs->status;
  pio_fis->transfer_count = io->nbytes;

  port_control_.task_flie_data = (regs->error << 8) | (regs->status);

  if (regs->status & 0x1) { /* Check error bit */
    TrigerIrq(kAhciPortIrqBitTaskFileError);
  }
  TrigerIrq(kAhciPortIrqBitPioSetupFis);
}

void AhciPort::TrigerIrq(int irqbit) {
  MV_ASSERT(irqbit < 32);
  uint32_t irq = 1U << irqbit;
  uint32_t irq_status = port_control_.irq_status | irq;
  port_control_.irq_status = irq_status;
  host_->CheckIrq();
}

