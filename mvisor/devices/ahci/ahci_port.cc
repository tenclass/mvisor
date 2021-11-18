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
  bzero(&ide_regs_, sizeof(ide_regs_));
  bzero(&ide_io_, sizeof(ide_io_));
  ide_io_.buffer_size = 512 * 256 * 256;
  ide_io_.buffer = (uint8_t*)valloc(ide_io_.buffer_size);
}

AhciPort::~AhciPort() {
  if (ide_io_.buffer) {
    free(ide_io_.buffer);
  }
}

void AhciPort::AttachDevice(IdeStorageDevice* device) {
  drive_ = device;
  drive_->BindPort(this);
}

void AhciPort::Reset() {
  port_control_.irq_status = 0;
  port_control_.irq_mask = 0;
  port_control_.sata_control = 0;
  port_control_.command = PORT_CMD_SPIN_UP | PORT_CMD_POWER_ON;

  port_control_.sata_status = 0;
  port_control_.sata_error = 0;
  port_control_.sata_active = 0;
  port_control_.task_flie_data = 0x7F;
  port_control_.signature = 0xFFFFFFFF;
  reg_d2h_fis_posted_ = false;

  if (!drive_) {
    return;
  }

  drive_->Ata_ResetSignature();
  if (drive_->type() == kIdeStorageTypeCdrom) {
    port_control_.signature = ATA_SIGNATURE_CDROM;
  } else {
    port_control_.signature = ATA_SIGNATURE_DISK;
  }
}

bool AhciPort::HandleCommand(int slot) {
  MV_ASSERT(command_list_);
  current_command_ = &((AhciCommandHeader*)command_list_)[slot];
  if (drive_ == nullptr) {
    MV_PANIC("bad port %d", port_index_);
    return false;
  }

  AhciCommandTable* command_table = (AhciCommandTable*)manager_->TranslateGuestMemory(current_command_->command_table_base);
  MV_ASSERT(command_table);
  AhciFisRegH2D* fis = (AhciFisRegH2D*)command_table->command_fis;

  if (fis->fis_type != kAhciFisTypeRegH2D) {
    MV_PANIC("unknown fis type 0x%x", fis->fis_type);
    return false;
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
  ide_regs_.command = fis->command;
  ide_regs_.feature0 = fis->feature0;
  ide_regs_.lba0 = fis->lba0;
  ide_regs_.lba1 = fis->lba1;
  ide_regs_.lba2 = fis->lba2;
  ide_regs_.device = fis->device;
  ide_regs_.lba3 = fis->lba3;
  ide_regs_.lba4 = fis->lba4;
  ide_regs_.lba5 = fis->lba5;
  ide_regs_.control = fis->feature1;
  ide_regs_.count0 = fis->count0;
  ide_regs_.count1 = fis->count1;

  /* Copy the ACMD field (ATAPI packet, if any) from the AHCI command
    * table to ide_state->io_buffer */
  if (current_command_->is_atapi) {
    memcpy(ide_io_.buffer, command_table->atapi_command, 0x10);
  }

  ide_regs_.error = 0;
  ide_io_.transfer_type = kIdeNoTransfer;
  
  current_command_->bytes_transferred = 0;
  drive_->StartCommand();

  port_control_.command_issue &= ~(1U << slot);

  if (ide_io_.transfer_type == kIdeTransferToDevice) {
    /* Move data from sglist to iobuffer ? */
    drive_->EndTransfer(kIdeTransferToDevice);
  }

  if (ide_io_.transfer_type == kIdeTransferToHost) {
    AhciPrdtEntry* sg = command_table->prdt_entries;
    int prdt_index = 0;
    ssize_t remain_bytes = ide_io_.nbytes;
    uint8_t* ptr = ide_io_.buffer;
    while (remain_bytes > 0 && prdt_index < current_command_->prdt_length) {
      void* host = manager_->TranslateGuestMemory(sg[prdt_index].address);
      MV_ASSERT(host);
      size_t count = remain_bytes < sg[prdt_index].size + 1 ? remain_bytes : sg[prdt_index].size + 1;
      memcpy(host, ptr, count);
      current_command_->bytes_transferred += count;
      prdt_index++;
      ptr += count;
    }
    drive_->EndTransfer(kIdeTransferToHost);
    
    if (ide_io_.dma_status) {
      UpdateRegisterD2H();
    } else {
      UpdateSetupPio();
    }
  } else {
    UpdateRegisterD2H();
  }
  return true;
}

void AhciPort::CheckCommand() {
  if ((port_control_.command & PORT_CMD_START) && port_control_.command_issue) {
    for (int slot = 0; (slot < 32) && port_control_.command_issue; slot++) {
      if ((port_control_.command_issue & (1U << slot)) && !HandleCommand(slot)) {
        port_control_.command_issue &= ~(1U << slot);
      }
    }
  }
}

void AhciPort::Read(uint64_t offset, uint32_t* data) {
  AhciPortReg reg_index = (AhciPortReg)(offset / sizeof(uint32_t));
  MV_ASSERT(reg_index < 32);
  if (reg_index == kAhciPortRegSataStatus) {
    if (drive_) {
      *data = ATA_SCR_SSTATUS_DET_DEV_PRESENT_PHY_UP |  // Physical communication established
        ATA_SCR_SSTATUS_SPD_GEN1 |                      // Speed
        ATA_SCR_SSTATUS_IPM_ACTIVE;                     // Full power
    } else {
      *data = ATA_SCR_SSTATUS_DET_NODEV;  // No device
    }
  } else {
    *data = *((uint32_t*)&port_control_ + reg_index);
  }
  // MV_LOG("%d read reg 0x%x value %x", port_index_, reg_index, *data);
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

    /* XXX usually the FIS would be pending on the bus here and
      issuing deferred until the OS enables FIS receival.
      Instead, we only submit it once - which works in most
      cases, but is a hack. */
    if ((port_control_.command & PORT_CMD_FIS_ON) && !reg_d2h_fis_posted_) {
      reg_d2h_fis_posted_ = true;
      UpdateRegisterD2H();
    }
    CheckCommand();
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
    CheckCommand();
    break;
  default:
    MV_PANIC("not implemented reg index = %x", reg_index);
  }
}

void AhciPort::UpdateRegisterD2H() {
  MV_ASSERT(rx_fis_ && port_control_.command & PORT_CMD_FIS_RX);
  auto d2h_fis = &rx_fis_->d2h_fis;
  bzero(d2h_fis, sizeof(*d2h_fis));

  d2h_fis->fis_type = kAhciFisTypeRegD2H;
  d2h_fis->interrupt = 1;
  d2h_fis->status = ide_regs_.status;
  d2h_fis->error = ide_regs_.error;

  d2h_fis->lba0 = ide_regs_.lba0;
  d2h_fis->lba1 = ide_regs_.lba1;
  d2h_fis->lba2 = ide_regs_.lba2;
  d2h_fis->device = ide_regs_.device;
  d2h_fis->lba3 = ide_regs_.lba3;
  d2h_fis->lba4 = ide_regs_.lba4;
  d2h_fis->lba5 = ide_regs_.lba5;
  d2h_fis->count0 = ide_regs_.count0;
  d2h_fis->count1 = ide_regs_.count1;

  port_control_.task_flie_data = (ide_regs_.error << 8) | (ide_regs_.status);
  if (ide_regs_.status & 1) {
    TrigerIrq(kAhciPortIrqBitTaskFileError);
  }
  TrigerIrq(kAhciPortIrqBitDeviceToHostFis);
}

void AhciPort::UpdateSetupPio() {
  MV_ASSERT(rx_fis_ && port_control_.command & PORT_CMD_FIS_RX);
  auto pio_fis = &rx_fis_->pio_fis;
  bzero(pio_fis, sizeof(*pio_fis));

  pio_fis->fis_type = kAhciFisTypePioSetup;
  pio_fis->interrupt = 1;
  pio_fis->status = ide_regs_.status;
  pio_fis->error = ide_regs_.error;

  pio_fis->lba0 = ide_regs_.lba0;
  pio_fis->lba1 = ide_regs_.lba1;
  pio_fis->lba2 = ide_regs_.lba2;
  pio_fis->device = ide_regs_.device;
  pio_fis->lba3 = ide_regs_.lba3;
  pio_fis->lba4 = ide_regs_.lba4;
  pio_fis->lba5 = ide_regs_.lba5;
  pio_fis->count0 = ide_regs_.count0;
  pio_fis->count1 = ide_regs_.count1;
  pio_fis->e_status = ide_regs_.status;
  pio_fis->transfer_count = ide_io_.nbytes;

  port_control_.task_flie_data = (ide_regs_.error << 8) | (ide_regs_.status);

  if (ide_regs_.status & 0x1) {
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

