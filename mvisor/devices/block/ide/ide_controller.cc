#include "devices/ide.h"
#include <cstring>
#include "logger.h"
#include "device_manager.h"
#include "devices/ata_interval.h"

IdeControllerDevice::IdeControllerDevice() {
  name_ = "ide-controller";

  /* FIXME: should gernerated by parent pci device */
  devfn_ = PCI_MAKE_DEVFN(1, 0);
  
  /* PCI config */
  pci_header_.vendor_id = 0x8848;
  pci_header_.device_id = 0xDEDE;
  /* IDE controller, primary and secondary channel in PCI native mode */
  pci_header_.class_code = 0x010185;
  pci_header_.revision_id = 0;
  pci_header_.header_type =  PCI_HEADER_TYPE_NORMAL;
  pci_header_.subsys_vendor_id = 0x1AF4;
  pci_header_.subsys_id = 0x1100;
  pci_header_.command = PCI_COMMAND_IO;
  pci_header_.status = 0;
  pci_header_.irq_pin = 1;

  /* IO bar, BIOS will allocate the address */
  pci_header_.bar[0] = PCI_BASE_ADDRESS_SPACE_IO;
  pci_header_.bar[1] = PCI_BASE_ADDRESS_SPACE_IO;
  pci_header_.bar[2] = PCI_BASE_ADDRESS_SPACE_IO;
  pci_header_.bar[3] = PCI_BASE_ADDRESS_SPACE_IO;
  bar_size_[0] = 8;
  bar_size_[1] = 4;
  bar_size_[2] = 8;
  bar_size_[3] = 4;
}

void IdeControllerDevice::Connect() {
  PciDevice::Connect();
  /* When the PCI initialized, we detect connected storages */
  for (auto device : children_) {
    IdeStorageDevice* drive = dynamic_cast<IdeStorageDevice*>(device);
    if (drive) {
      drives_.push_back(drive);
    }
  }
  Reset();
}

void IdeControllerDevice::ResetPort(int i) {
  auto *port = &ports_[i];
  bzero(port, sizeof(*port));
  port->index = i;
  port->status = ATA_SR_DRDY | ATA_SR_DSC;
  port->drive = drives_.size() > 0 ? drives_[0] : nullptr;
  port->io_buffer_size = 512 * 256;
  port->registers[ATA_REG_HDDEVSEL] = 0xA0;
  if (port->io_buffer) {
    free(port->io_buffer);
  }
  port->io_buffer = (uint8_t*)valloc(port->io_buffer_size);
}

void IdeControllerDevice::Reset() {
  ResetPort(0);
  ResetPort(1);
}

void IdeControllerDevice::ReadControlPort(int index, uint64_t offset, uint8_t* data, uint32_t size) {
  if (offset == ATA_CB_ASTAT) {
    /* alternative status */
    *data = ports_[index].status;
  }
}

void IdeControllerDevice::WriteControlPort(int index, uint64_t offset, uint8_t* data, uint32_t size) {
  uint8_t value = *data;
  if (offset == ATA_CB_DC) {
    /* control byte */
    if (value & ATA_CB_DC_SRST) {
      ResetPort(index);
    }
    MV_LOG("interrupt is %s", value & ATA_CB_DC_NIEN ? "DISABLED" : "ENABLED");
    ports_[index].control = value;
  }
}

void IdeControllerDevice::Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  if (ir.base == (pci_header_.bar[1] & PCI_BASE_ADDRESS_IO_MASK)) {
    return WriteControlPort(0, offset, data, size);
  } else if (ir.base == (pci_header_.bar[3] & PCI_BASE_ADDRESS_IO_MASK)) {
    return WriteControlPort(1, offset, data, size);
  }
  MV_LOG("write port %s data:0x%x size:0x%x", __get_register_name(offset), *data, size);

  size_t port_index = ir.base == (pci_header_.bar[0] & PCI_BASE_ADDRESS_IO_MASK) ? 0 : 1;
  IdePort* port = &ports_[port_index];
  switch (offset)
  {
  case ATA_REG_HDDEVSEL: {
    size_t index = (*data & ~0xA0) + port_index * 2;
    if (index < drives_.size()) {
      port->drive = drives_[index];
    } else {
      port->drive = nullptr;
      return;
    }
    MV_LOG("port %d select drive %s", index, port->drive->name().c_str());
    break;
  }
  case ATA_REG_ERROR:
    break;
  case ATA_REG_SECCOUNT0:
  case ATA_REG_SECCOUNT1:
  case ATA_REG_LBA0:
  case ATA_REG_LBA1:
  case ATA_REG_LBA2:
  case ATA_REG_LBA3:
  case ATA_REG_LBA4:
  case ATA_REG_LBA5:
    if (!port->drive) {
      /* no registers has been set if no drive availabe */
      return;
    }
    break;
  case ATA_REG_COMMAND:
    port->command = *data;
    ProcessCommandBegin(port);
    return;
  case ATA_REG_DATA:
    WriteIoBuffer(port, data, size);
    return;
  default:
    MV_PANIC("register %s is not writable reg=0x%x", __get_register_name(offset), offset);
    break;
  }
  // update registers
  port->registers[offset] = *data;
}

void IdeControllerDevice::Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  if (ir.base == (pci_header_.bar[1] & PCI_BASE_ADDRESS_IO_MASK)) {
    return ReadControlPort(0, offset, data, size);
  } else if (ir.base == (pci_header_.bar[3] & PCI_BASE_ADDRESS_IO_MASK)) {
    return ReadControlPort(1, offset, data, size);
  }
  size_t port_index = ir.base == (pci_header_.bar[0] & PCI_BASE_ADDRESS_IO_MASK) ? 0 : 1;
  IdePort* port = &ports_[port_index];

  MV_ASSERT(offset < sizeof(port->registers) / sizeof(port->registers[0]));
  switch (offset)
  {
  case ATA_REG_STATUS:
    *data = port->status;
    MV_LOG("read STATUS data:0x%x size:0x%x", *data, size);
    manager_->SetIrq(pci_header_.irq_line, 0);
    break;
  case ATA_REG_DATA:
    ReadIoBuffer(port, data, size);
    break;
  default:
    *data = port->registers[offset];
    MV_LOG("read port %s(0x%x) data:0x%x size:0x%x", __get_register_name(offset), ir.base + offset, *data, size);
    break;
  }
}

void IdeControllerDevice::WriteIoBuffer(IdePort* port, uint8_t* data, uint32_t size) {
  MV_LOG("write io buffer 0x%x size: %x", port->io_buffer_index, size);
  if (!port->drive || !(port->status & ATA_SR_DRQ)) {
    MV_PANIC("Invalid status 0x%x", port->status);
  }
  if (port->io_buffer_index + size >= port->io_buffer_size) {
    MV_PANIC("io buffer overflow");
  }
  uint8_t *dest = &port->io_buffer[port->io_buffer_index];
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
  port->io_buffer_index += size;
  if (port->io_buffer_index >= port->io_rw_count) {
    port->status &= ~ATA_SR_DRQ;
    ProcessCommandEnd(port);
  }
}

void IdeControllerDevice::ReadIoBuffer(IdePort* port, uint8_t* data, uint32_t size) {
  MV_LOG("read io buffer 0x%x size: %x", port->io_buffer_index, size);
  if (!port->drive || !(port->status & ATA_SR_DRQ)) {
    MV_PANIC("Invalid status 0x%x", port->status);
  }
  if (port->io_buffer_index + size >= port->io_buffer_size) {
    MV_PANIC("io buffer overflow");
  }
  uint8_t *dest = &port->io_buffer[port->io_buffer_index];
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
  port->io_buffer_index += size;
  if (port->io_buffer_index >= port->io_rw_count) {
    port->status &= ~ATA_SR_DRQ;
  }
}

void IdeControllerDevice::ProcessCommandBegin(IdePort* port) {
  if (!port->drive) {
    MV_PANIC("select port drive first command=0x%x", port->command);
    return;
  }
  port->status = ATA_SR_DRDY;
  switch (port->command)
  {
  case ATA_CMD_IDENTIFY_PACKET_DEVICE:
    port->drive->GetIdentityData(port->io_buffer, port->io_buffer_size, &port->io_rw_count);
    // Data is ready
    port->status |= ATA_SR_DRQ;
    port->io_buffer_index = 0;
    break;
  case ATA_CMD_PACKET:
    // Wait for packet data
    port->io_buffer_index = 0;
    port->io_rw_count = 12; /* PACKET CMD SIZE */
    port->registers[ATA_REG_SECCOUNT0] = 1;
    port->status |= ATA_SR_DRQ;
    break;
  case ATA_CMD_DEVICE_RESET: {
    auto *drive = port->drive;
    ResetPort(port->index);
    port->drive = drive;
    break;
  }
  case ATA_CMD_IDENTIFY_DEVICE:
    port->registers[ATA_REG_HDDEVSEL] &= ~0xF;
    port->registers[ATA_REG_SECCOUNT0] = 1;
    port->registers[ATA_REG_LBA0] = 1;
    if (port->drive && port->drive->type() == kIdeStorageTypeCdrom) {
      port->registers[ATA_REG_LBA1] = 0x14;
      port->registers[ATA_REG_LBA2] = 0xEB;
    } else if (port->drive) {
      port->registers[ATA_REG_LBA1] = 0;
      port->registers[ATA_REG_LBA2] = 0;
    } else {
      port->registers[ATA_REG_LBA1] = 0xFF;
      port->registers[ATA_REG_LBA2] = 0xFF;
    }
    break;
  case ATA_CMD_NOP:
    TrigerIrq(port);
    break;
  default:
    MV_PANIC("unknown command 0x%x", port->command);
    break;
  }
}

void IdeControllerDevice::ProcessCommandEnd(IdePort* port) {
  switch (port->command)
  {
  case ATA_CMD_PACKET:
    port->status |= ATA_SR_BSY;
    port->drive->ProcessPacket(port->io_buffer, &port->io_rw_count);
    port->status &= ~ATA_SR_BSY;
    if (port->io_rw_count > 0) {
      port->status |= ATA_SR_DRQ;
      port->io_buffer_index = 0;
      // if (port->io_rw_count == 0x24) {
      //   port->registers[ATA_REG_SECCOUNT0] = 2;
      // }
    }
    TrigerIrq(port);
    break;
  }
}

void IdeControllerDevice::TrigerIrq(IdePort* port) {
  if (port->control & ATA_CB_DC_NIEN) {
    MV_LOG("irq is disabled");
    return;
  }
  MV_LOG("triger irq %d status=0x%x", pci_header_.irq_line, port->status);
  manager_->SetIrq(pci_header_.irq_line, 1);
}
