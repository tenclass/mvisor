#include "devices/ahci_host.h"
#include "logger.h"
#include "device_manager.h"

AhciPort::AhciPort(DeviceManager* manager, AhciHostDevice* host)
  : manager_(manager), host_(host) {
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

  /* reset ncq queue */
  for (int i = 0; i < AHCI_MAX_CMDS; i++) {
    // NCQTransferState *ncq_tfs = &s->dev[port].ncq_tfs[i];
    // ncq_tfs->halt = false;
    // if (!ncq_tfs->used) {
    //     continue;
    // }
    // cancel ops
  }

  // s->dev[port].port_state = STATE_RUN;
  // if (ide_state->drive_kind == IDE_CD) {
  //   ahci_set_signature(d, SATA_SIGNATURE_CDROM);
  //   ide_state->status = SEEK_STAT | WRERR_STAT | READY_STAT;
  // } else {
  //   ahci_set_signature(d, SATA_SIGNATURE_DISK);
  //   ide_state->status = SEEK_STAT | WRERR_STAT;
  // }
}

bool AhciPort::HandleCommand(int slot) {
  // uint64_t tbl_addr;
  // AHCICmdHdr *cmd;
  // uint8_t *cmd_fis;
  // dma_addr_t cmd_len;

  // if (s->dev[port].port.ifs[0].status & (BUSY_STAT|DRQ_STAT)) {
  //     /* Engine currently busy, try again later */
  //     trace_handle_cmd_busy(s, port);
  //     return -1;
  // }

  // if (!s->dev[port].lst) {
  //     trace_handle_cmd_nolist(s, port);
  //     return -1;
  // }
  // cmd = get_cmd_header(s, port, slot);
  // /* remember current slot handle for later */
  // s->dev[port].cur_cmd = cmd;

  // /* The device we are working for */
  // ide_state = &s->dev[port].port.ifs[0];
  // if (!ide_state->blk) {
  //     trace_handle_cmd_badport(s, port);
  //     return -1;
  // }

  // tbl_addr = le64_to_cpu(cmd->tbl_addr);
  // cmd_len = 0x80;
  // cmd_fis = dma_memory_map(s->as, tbl_addr, &cmd_len,
  //                           DMA_DIRECTION_FROM_DEVICE);
  // if (!cmd_fis) {
  //     trace_handle_cmd_badfis(s, port);
  //     return -1;
  // } else if (cmd_len != 0x80) {
  //     ahci_trigger_irq(s, &s->dev[port], AHCI_PORT_IRQ_BIT_HBFS);
  //     trace_handle_cmd_badmap(s, port, cmd_len);
  //     goto out;
  // }
  // if (trace_event_get_state_backends(TRACE_HANDLE_CMD_FIS_DUMP)) {
  //     char *pretty_fis = ahci_pretty_buffer_fis(cmd_fis, 0x80);
  //     trace_handle_cmd_fis_dump(s, port, pretty_fis);
  //     g_free(pretty_fis);
  // }
  // switch (cmd_fis[0]) {
  //     case SATA_FIS_TYPE_REGISTER_H2D:
  //         handle_reg_h2d_fis(s, port, slot, cmd_fis);
  //         break;
  //     default:
  //         trace_handle_cmd_unhandled_fis(s, port,
  //                                         cmd_fis[0], cmd_fis[1], cmd_fis[2]);
  //         break;
  // }

  // out:
  // dma_memory_unmap(s->as, cmd_fis, cmd_len, DMA_DIRECTION_FROM_DEVICE,
  //                   cmd_len);

  // if (s->dev[port].port.ifs[0].status & (BUSY_STAT|DRQ_STAT)) {
  //     /* async command, complete later */
  //     s->dev[port].busy_slot = slot;
  //     return -1;
  // }

  // /* done handling the command */
  // return 0;
  MV_PANIC("not implemented command of slot %d", slot);
  return false;
}

void AhciPort::CheckCommand() {
  if ((port_control_.command & PORT_CMD_START) && port_control_.command_issue) {
    for (int slot = 0; (slot < 32) && port_control_.command_issue; slot++) {
      if ((port_control_.command_issue & (1U << slot)) &&
        !HandleCommand(slot)) {
        port_control_.command_issue &= ~(1U << slot);
      }
    }
  }
}

void AhciPort::Read(uint64_t offset, uint32_t* data) {
  AHCIPortReg reg_index = (AHCIPortReg)(offset / sizeof(uint32_t));
  MV_ASSERT(reg_index < (AHCI_PORT_ADDR_OFFSET_LEN)/ sizeof(uint32_t));
  if (reg_index == AHCI_PORT_REG_SCR_STAT) {
    if (true) {
      *data = SATA_SCR_SSTATUS_DET_DEV_PRESENT_PHY_UP |
        SATA_SCR_SSTATUS_SPD_GEN1 | SATA_SCR_SSTATUS_IPM_ACTIVE;
    } else {
      *data = SATA_SCR_SSTATUS_DET_NODEV;
    }
  } else {
    *data = *((uint32_t*)&port_control_ + reg_index);
  }
  MV_LOG("read reg 0x%x value %x", reg_index, *data);
}

void AhciPort::CheckEngines() {
  bool cmd_start = port_control_.command & PORT_CMD_START;
  bool cmd_on    = port_control_.command & PORT_CMD_LIST_ON;
  bool fis_start = port_control_.command & PORT_CMD_FIS_RX;
  bool fis_on    = port_control_.command & PORT_CMD_FIS_ON;

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
    fis_ = (uint8_t*)manager_->TranslateGuestMemory(((uint64_t)port_control_.fis_base_high << 32) |
      port_control_.fis_base);
    if (fis_ != nullptr) {
      port_control_.command |= PORT_CMD_FIS_ON;
    } else {
      port_control_.command &= ~(PORT_CMD_FIS_RX | PORT_CMD_FIS_ON);
      return;
    }
  } else if (!fis_start && fis_on) {
    port_control_.command &= ~PORT_CMD_FIS_ON;
    fis_ = nullptr;
  }
}

void AhciPort::Write(uint64_t offset, uint32_t value) {
  AHCIPortReg reg_index = (AHCIPortReg)(offset / sizeof(uint32_t));
  MV_ASSERT(reg_index < (AHCI_PORT_ADDR_OFFSET_LEN)/ sizeof(uint32_t));
  MV_LOG("write reg 0x%x value %x", reg_index, value);
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
    if ((port_control_.command & PORT_CMD_FIS_ON) && !registered_) {
      Register();
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
    CheckCommand();
    break;
  default:
    MV_PANIC("not implemented reg index = %x", reg_index);
  }
}

void AhciPort::Register() {
  port_control_.task_flie_data = 0;
}
