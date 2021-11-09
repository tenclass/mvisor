#include "devices/ide/ide_port.h"
#include <cstring>
#include "logger.h"
#include "device_manager.h"
#include "devices/ide/ide_controller.h"
#include "devices/ide/ata_interval.h"

IdePort::IdePort(DeviceManager* manager, IdeControllerDevice* controller, int index)
  : manager_(manager), controller_(controller), index_(index) {
  io_.buffer_size = 512 * 256;
  io_.buffer = (uint8_t*)valloc(io_.buffer_size);
}

IdePort::IdePort(DeviceManager* manager, int index)
  : manager_(manager), index_(index) {
  io_.buffer_size = 512 * 256;
  io_.buffer = (uint8_t*)valloc(io_.buffer_size);
  MV_ASSERT(manager_);
}

IdePort::~IdePort() {
  if (io_.buffer) {
    free(io_.buffer);
  }
}

void IdePort::AttachDevice(IdeStorageDevice* device) {
  attached_devices_.push_back(device);
}

void IdePort::Reset() {
  bzero(&registers_, sizeof(registers_));
  drive_ = nullptr;
  registers_.status = ATA_SR_DRDY | ATA_SR_DSC | ATA_SR_ERR;
  registers_.devsel = 0xA0;
}

void IdePort::RaiseIrq() {
  if (registers_.control & ATA_CB_DC_NIEN) {
    return;
  }
  int irq = controller_->pci_header().irq_line;
  manager_->SetIrq(irq, 1);
  MV_LOG("Triger IRQ %d status=0x%x reason=0x%x",
    irq, registers_.status, registers_.sectors0);
}

void IdePort::LowerIrq() {
  manager_->SetIrq(controller_->pci_header().irq_line, 0);
}

void IdePort::ReadControlPort(uint64_t offset, uint8_t* data, uint32_t size) {
  /* Only alternative status is avaiable at the moment
   * some ports are conflicted with Floppy device, just ignore them here
   */
  if (offset == ATA_CB_ASTAT) { /* alternative status */
    *data = registers_.status;
  }
}

void IdePort::WriteControlPort(uint64_t offset, uint8_t* data, uint32_t size) {
  uint8_t value = *data;
  if (offset == ATA_CB_DC) { /* control byte */
    if (value & ATA_CB_DC_SRST) {
      Reset();
    }
    if (value & ATA_CB_DC_NIEN) {
      LowerIrq();
    }
    registers_.control = value;
  }
}

void IdePort::ReadPort(uint64_t offset, uint8_t* data, uint32_t size) {
  MV_ASSERT(offset < IDE_MAX_REGISTERS);
  switch (offset)
  {
  case ATA_REG_STATUS:
    *data = registers_.status;
    // MV_LOG("read port %d STATUS data:0x%x size:0x%x", index_, *data, size);
    LowerIrq();
    break;
  case ATA_REG_ERROR:
    *data = registers_.error;
    break;
  case ATA_REG_DATA:
    ReadIoBuffer(data, size);
    break;
  default:
    /* Other registers provided by values */
    *data = registers_.values[offset];
    // MV_LOG("read port %s(0x%x) data:0x%x size:0x%x", __get_register_name(offset), offset, *data, size);
    break;
  }
}

void IdePort::WritePort(uint64_t offset, uint8_t* data, uint32_t size) {
  MV_ASSERT(offset < IDE_MAX_REGISTERS);
  switch (offset)
  {
  case ATA_REG_HDDEVSEL: {
    size_t devsel = (*data >> 4) & 1;
    if (devsel < attached_devices_.size()) {
      drive_ = attached_devices_[devsel];
      drive_->BindPort(this);
      registers_.devsel = *data;
    } else {
      drive_ = nullptr;
    }
    // MV_LOG("port %d select drive %x:%s", index_, *data, drive_ ? drive_->name().c_str() : "null");
    break;
  }
  case ATA_REG_COMMAND:
    registers_.command = *data;
    if (drive_) {
      LowerIrq();
      drive_->StartCommand();
    }
    break;
  case ATA_REG_DATA:
    WriteIoBuffer(data, size);
    break;
  case ATA_REG_FEATURES:
    MV_ASSERT(*data == 0); /* DMA not supported yet */
    /* fall through */
  default:
    if (drive_) {
      // update registers if drive available
      registers_.values[offset] = *data;
    }
  }
}

void IdePort::WriteIoBuffer(uint8_t* data, uint32_t size) {
  if (!drive_ || !(registers_.status & ATA_SR_DRQ)) {
    MV_PANIC("Invalid status 0x%x", registers_.status);
  }
  if (io_.position + size >= io_.buffer_size) {
    MV_PANIC("io buffer overflow");
  }
  uint8_t *dest = &io_.buffer[io_.position];
  switch (size)
  {
  case 1:
    *dest = *data;
    break;
  case 2:
    *(uint16_t*)dest = *(uint16_t*)data;
    break;
  case 4:
    *(uint32_t*)dest = *(uint32_t*)data;
  }
  io_.position += size;
  if (io_.position >= io_.nbytes) {
    drive_->EndTransfer(kIdeTransferToDevice);
  }
}

void IdePort::ReadIoBuffer(uint8_t* data, uint32_t size) {
  if (!drive_) {
    return;
  }
  if (!(registers_.status & ATA_SR_DRQ)) {
    MV_PANIC("Invalid status 0x%x", registers_.status);
  }
  if (io_.position + size >= io_.buffer_size) {
    MV_PANIC("io buffer overflow");
  }
  uint8_t *dest = &io_.buffer[io_.position];
  switch (size)
  {
  case 1:
    *data = *dest;
    break;
  case 2:
    *(uint16_t*)data = *(uint16_t*)dest;
    break;
  case 4:
    *(uint32_t*)data = *(uint32_t*)dest;
  }
  // MV_LOG("read io buffer port %d pos %x bytes %x data %x", index_, io_.position, size, *(uint16_t*)data);
  io_.position += size;
  if (io_.position >= io_.nbytes) {
    drive_->EndTransfer(kIdeTransferToHost);
  }
}
