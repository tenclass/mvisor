#include "devices/ahci/ahci_host.h"
#include <cstring>
#include "logger.h"
#include "device_manager.h"
#include "devices/ahci/ahci_internal.h"
#include "devices/ide/ide_storage.h"


static inline int is_native_command_queueing(uint8_t ata_cmd)
{
    /* Based on SATA 3.2 section 13.6.3.2 */
    switch (ata_cmd) {
    case READ_FPDMA_QUEUED:
    case WRITE_FPDMA_QUEUED:
    case NCQ_NON_DATA:
    case RECEIVE_FPDMA_QUEUED:
    case SEND_FPDMA_QUEUED:
        return 1;
    default:
        return 0;
    }
}

AhciPort::AhciPort(DeviceManager* manager, AhciHostDevice* host, int index)
  : IdePort(manager, index) {
  host_ = host;
}

AhciPort::~AhciPort() {
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
  register_fis_posted_ = false;

  if (!drive_) {
    return;
  }

  drive_->ResetSignature();
}

void AhciPort::AttachDevice(IdeStorageDevice* device) {
  drive_ = device;
  drive_->BindPort(this);
}

bool AhciPort::HandleCommand(int slot) {
  if (!command_list_) {
    MV_PANIC("command_list is null");
    return false;
  }

  current_command_ = &((AHCICommandHeader*)command_list_)[slot];
  if (drive_ == nullptr) {
    MV_PANIC("bad port %d", index_);
    return false;
  }

  uint8_t* cmd_fis = (uint8_t*)manager_->TranslateGuestMemory(current_command_->command_table_base);
  if (!cmd_fis) {
    MV_LOG("invalid fis address 0x%lx", current_command_->command_table_base);
    return false;
  }

  if (cmd_fis[0] != SATA_FIS_TYPE_REGISTER_H2D) {
    MV_PANIC("unknown fis type 0x%x", cmd_fis[0]);
    return false;
  }

  if (!(cmd_fis[1] & SATA_FIS_REG_H2D_UPDATE_COMMAND_REGISTER)) {
    MV_LOG("invalid fis 0x%x", cmd_fis[1]);
    return false;
  }

  if (is_native_command_queueing(cmd_fis[2])) {
    MV_PANIC("not supported yet %x", cmd_fis[2]);
  }

  /* Decompose the FIS:
    * AHCI does not interpret FIS packets, it only forwards them.
    * SATA 1.0 describes how to decode LBA28 and CHS FIS packets.
    * Later specifications, e.g, SATA 3.2, describe LBA48 FIS packets.
    *
    * ATA4 describes sector number for LBA28/CHS commands.
    * ATA6 describes sector number for LBA48 commands.
    * ATA8 deprecates CHS fully, describing only LBA28/48.
    *
    * We dutifully convert the FIS into IDE registers, and allow the
    * core layer to interpret them as needed. */
  registers_.command = cmd_fis[2];
  registers_.features = cmd_fis[3];
  registers_.lba0 = cmd_fis[4];        /* LBA 7:0 */
  registers_.lba1 = cmd_fis[5];        /* LBA 15:8  */
  registers_.lba2 = cmd_fis[6];        /* LBA 23:16 */
  registers_.devsel = cmd_fis[7];      /* LBA 27:24 (LBA28) */
  registers_.lba3 = cmd_fis[8];        /* LBA 31:24 */
  registers_.lba4 = cmd_fis[9];        /* LBA 39:32 */
  registers_.lba5 = cmd_fis[10];       /* LBA 47:40 */
  registers_.features = cmd_fis[11];
  registers_.sectors0 = cmd_fis[12];
  registers_.sectors1 = cmd_fis[13];
  /* 14, 16, 17, 18, 19: Reserved (SATA 1.0) */
  /* 15: Only valid when UPDATE_COMMAND not set. */

  /* Copy the ACMD field (ATAPI packet, if any) from the AHCI command
    * table to ide_state->io_buffer */
  if (current_command_->options & AHCI_CMD_ATAPI) {
    memcpy(io_.buffer, &cmd_fis[AHCI_COMMAND_TABLE_ACMD], 0x10);
  }

  registers_.error = 0;
  
  current_command_->bytes = 0;
  MV_LOG("################ command = %x", registers_.command);
  drive_->StartCommand();
  // data is ready
  port_control_.command_issue &= ~(1U << slot);
  UpdateRegisterD2H();


  drive_->EndTransfer(kIdeTransferToDevice);
  if (io_.nbytes > 0) {
    AHCI_SG* sg = (AHCI_SG*)(cmd_fis + 0x80);
    void* host = manager_->TranslateGuestMemory(sg[0].addr);
    MV_ASSERT(host);
    memcpy(host, io_.buffer, io_.nbytes);
    MV_LOG("copy bytes %ld to %p status: %lx", io_.nbytes, host, registers_.status);
    current_command_->bytes += io_.nbytes;
    UpdateSetupPio(io_.nbytes);
    drive_->EndTransfer(kIdeTransferToHost);
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
  AHCIPortReg reg_index = (AHCIPortReg)(offset / sizeof(uint32_t));
  MV_ASSERT(reg_index < (AHCI_PORT_ADDR_OFFSET_LEN)/ sizeof(uint32_t));
  if (reg_index == AHCI_PORT_REG_SCR_STAT) {
    if (drive_) {
      *data = SATA_SCR_SSTATUS_DET_DEV_PRESENT_PHY_UP |
        SATA_SCR_SSTATUS_SPD_GEN1 | SATA_SCR_SSTATUS_IPM_ACTIVE;
    } else {
      *data = SATA_SCR_SSTATUS_DET_NODEV;
    }
  } else {
    *data = *((uint32_t*)&port_control_ + reg_index);
  }
  if (reg_index == AHCI_PORT_REG_IRQ_STAT && *data == 0) {
    return;
  }
  MV_LOG("%d read reg 0x%x value %x", index_, reg_index, *data);
}

void AhciPort::CheckEngines() {
  bool cmd_start = port_control_.command & PORT_CMD_START;
  bool cmd_on    = port_control_.command & PORT_CMD_LIST_ON;
  bool fis_start = port_control_.command & PORT_CMD_FIS_RX;
  bool fis_on    = port_control_.command & PORT_CMD_FIS_ON;

  MV_ASSERT(manager_);
  if (cmd_start && !cmd_on) {
    command_list_ = (uint8_t*)manager_->TranslateGuestMemory(((uint64_t)port_control_.command_list_base_high << 32) |
      port_control_.command_list_base);
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
    res_fis_ = (uint8_t*)manager_->TranslateGuestMemory(((uint64_t)port_control_.fis_base_high << 32) |
      port_control_.fis_base);
    if (res_fis_ != nullptr) {
      port_control_.command |= PORT_CMD_FIS_ON;
    } else {
      port_control_.command &= ~(PORT_CMD_FIS_RX | PORT_CMD_FIS_ON);
      return;
    }
  } else if (!fis_start && fis_on) {
    port_control_.command &= ~PORT_CMD_FIS_ON;
    res_fis_ = nullptr;
  }
}

void AhciPort::Write(uint64_t offset, uint32_t value) {
  AHCIPortReg reg_index = (AHCIPortReg)(offset / sizeof(uint32_t));
  MV_ASSERT(reg_index < (AHCI_PORT_ADDR_OFFSET_LEN)/ sizeof(uint32_t));
  MV_LOG("%d write reg 0x%x value %x irq %x", index_, reg_index, value, port_control_.irq_status);
  switch (reg_index)
  {
  case AHCI_PORT_REG_LST_ADDR:
    port_control_.command_list_base = value;
    break;
  case AHCI_PORT_REG_LST_ADDR_HI:
    port_control_.command_list_base_high = value;
    break;
  case AHCI_PORT_REG_FIS_ADDR:
    port_control_.fis_base = value;
    break;
  case AHCI_PORT_REG_FIS_ADDR_HI:
    port_control_.fis_base_high = value;
    break;
  case AHCI_PORT_REG_IRQ_STAT:
    port_control_.irq_status &= ~value;
    host_->CheckIrq();
    break;
  case AHCI_PORT_REG_IRQ_MASK:
    port_control_.irq_mask = value & 0xfdc000ff;
    host_->CheckIrq();
    break;
  case AHCI_PORT_REG_CMD:
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
    if ((port_control_.command & PORT_CMD_FIS_ON) && !register_fis_posted_) {
      register_fis_posted_ = true;
      UpdateRegisterD2H();
    }
    CheckCommand();
    break;
  case AHCI_PORT_REG_TFDATA:
  case AHCI_PORT_REG_SIG:
  case AHCI_PORT_REG_SCR_STAT:
    /* Read Only */
    break;
  case AHCI_PORT_REG_SCR_CTL:
    if (((port_control_.sata_control & AHCI_SCR_SCTL_DET) == 1) &&
        ((value & AHCI_SCR_SCTL_DET) == 0)) {
      Reset();
    }
    port_control_.sata_control = value;
    break;
  case AHCI_PORT_REG_SCR_ERR:
    port_control_.sata_error &= ~value;
    break;
  case AHCI_PORT_REG_SCR_ACT:
    /* RW1 */
    port_control_.sata_active |= value;
    break;
  case AHCI_PORT_REG_CMD_ISSUE:
    port_control_.command_issue |= value;
    MV_LOG("issure %x all:%x", value, port_control_.command_issue);
    CheckCommand();
    break;
  default:
    MV_PANIC("not implemented reg index = %x", reg_index);
  }
  MV_LOG("%d write reg 0x%x value %x irq %x done", index_, reg_index, value, port_control_.irq_status);
}

void AhciPort::UpdateRegisterD2H() {
  if (!res_fis_ || !(port_control_.command & PORT_CMD_FIS_RX)) {
    return;
  }
  uint8_t *d2h_fis = &res_fis_[RES_FIS_RFIS];

  d2h_fis[0] = SATA_FIS_TYPE_REGISTER_D2H;
  d2h_fis[1] = (1 << 6); /* interrupt bit */
  d2h_fis[2] = registers_.status;
  d2h_fis[3] = registers_.error;

  d2h_fis[4] = registers_.lba0;
  d2h_fis[5] = registers_.lba1;
  d2h_fis[6] = registers_.lba2;
  d2h_fis[7] = registers_.devsel;
  d2h_fis[8] = registers_.lba3;
  d2h_fis[9] = registers_.lba4;
  d2h_fis[10] = registers_.lba5;
  d2h_fis[11] = 0;
  d2h_fis[12] = registers_.sectors0;
  d2h_fis[13] = registers_.sectors1;
  for (int i = 14; i < 20; i++) {
    d2h_fis[i] = 0;
  }

  port_control_.task_flie_data = (registers_.error << 8) | (registers_.status);
  if (registers_.status & 1) {
    TrigerIrq(AHCI_PORT_IRQ_BIT_TFES);
  }
  TrigerIrq(AHCI_PORT_IRQ_BIT_DHRS);
}

void AhciPort::UpdateSetupPio(uint32_t size) {
  if (!res_fis_ || !(port_control_.command & PORT_CMD_FIS_RX)) {
    return;
  }
  uint8_t *pio_fis = &res_fis_[RES_FIS_PSFIS];

  pio_fis[0] = SATA_FIS_TYPE_PIO_SETUP;
  pio_fis[1] = (1 << 6); /* interrupt bit */
  pio_fis[2] = registers_.status;
  pio_fis[3] = registers_.error;

  pio_fis[4] = registers_.lba0;
  pio_fis[5] = registers_.lba1;
  pio_fis[6] = registers_.lba2;
  pio_fis[7] = registers_.devsel;
  pio_fis[8] = registers_.lba3;
  pio_fis[9] = registers_.lba4;
  pio_fis[10] = registers_.lba5;
  pio_fis[11] = 0;
  pio_fis[12] = registers_.sectors0;
  pio_fis[13] = registers_.sectors1;
  pio_fis[14] = 0;
  pio_fis[15] = registers_.status;
  pio_fis[16] = size & 255;
  pio_fis[17] = size >> 8;

  port_control_.task_flie_data = (registers_.error << 8) | (registers_.status);

  if (registers_.status & 0x1) {
    TrigerIrq(AHCI_PORT_IRQ_BIT_TFES);
  }
  TrigerIrq(AHCI_PORT_IRQ_BIT_PSS);
  MV_LOG("UpdateSetupPio status=%x", registers_.status);
}

void AhciPort::TrigerIrq(int irqbit) {
  MV_ASSERT(irqbit < 32);
  uint32_t irq = 1U << irqbit;
  uint32_t irq_status = port_control_.irq_status | irq;
  port_control_.irq_status = irq_status;
  host_->CheckIrq();
}


void AhciPort::RaiseIrq() {
  if (registers_.control & 2) {
    return;
  }
  MV_LOG("Triger IRQ status=0x%x reason=0x%x", registers_.status, registers_.sectors0);
}

void AhciPort::LowerIrq() {
}

// bool AhciStorageDevice::PioTransfer(AHCICommandHeader* cmd_header, NCQFrame* frame, uint8_t* buffer, size_t nbytes) {
//   status_ |= DRQ_STAT;
//   AHCI_SG* sg = (AHCI_SG*)((uint64_t)frame + 0x80);
//   void* host = manager_->TranslateGuestMemory(sg[0].addr);
//   MV_ASSERT(host);
//   memcpy(host, buffer, nbytes);
//   MV_LOG("copy bytes %ld to %p status: %lx", nbytes, host, status_);
//   cmd_header->bytes += nbytes;
 
//   port_->UpdateSetupPio(nbytes);
//   return true;
// }

