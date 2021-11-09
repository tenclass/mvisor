#include "devices/ide/ide_controller.h"
#include "logger.h"
#include "device_manager.h"
#include "devices/ide/ata_interval.h"

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
  pci_header_.irq_line = 14;

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

IdeControllerDevice::~IdeControllerDevice() {
  for (int i = 0; i < IDE_MAX_PORTS; i++) {
    delete ports_[i];
  }
}

void IdeControllerDevice::WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
  if (offset == 0x3C) {
    /* SeaBIOS chagned this. */
    return;
  } else {
    PciDevice::WritePciConfigSpace(offset, data, length);
  }
}

void IdeControllerDevice::Connect() {
  PciDevice::Connect();

  for (int i = 0; i < IDE_MAX_PORTS; i++) {
    if (!ports_[i])
      ports_[i] = new IdePort(manager_, this, i);
  }

  /* When the PCI initialized, we detect connected storages */
  int port_index = 0;
  for (auto device : children_) {
    IdeStorageDevice* storage = dynamic_cast<IdeStorageDevice*>(device);
    if (storage) {
      ports_[port_index]->AttachDevice(storage);
      if (ports_[port_index]->attached_devices().size() >= IDE_MAX_STORAGES_PER_PORT) {
        ++port_index;
        if (port_index >= IDE_MAX_PORTS)
          break;
      }
    }
  }

  /* Set ports to initialized status */
  for (int i = 0; i < IDE_MAX_PORTS; i++) {
    ports_[i]->Reset();
  }
}

void IdeControllerDevice::Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  if (ir.base == (pci_header_.bar[1] & PCI_BASE_ADDRESS_IO_MASK)) {
    ports_[0]->WriteControlPort(offset, data, size);
  } else if (ir.base == (pci_header_.bar[3] & PCI_BASE_ADDRESS_IO_MASK)) {
    ports_[1]->WriteControlPort(offset, data, size);
  } else if (ir.base == (pci_header_.bar[0] & PCI_BASE_ADDRESS_IO_MASK)) {
    ports_[0]->WritePort(offset, data, size);
  } else if (ir.base == (pci_header_.bar[2] & PCI_BASE_ADDRESS_IO_MASK)) {
    ports_[1]->WritePort(offset, data, size);
  }
}

void IdeControllerDevice::Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  if (ir.base == (pci_header_.bar[1] & PCI_BASE_ADDRESS_IO_MASK)) {
    ports_[0]->ReadControlPort(offset, data, size);
  } else if (ir.base == (pci_header_.bar[3] & PCI_BASE_ADDRESS_IO_MASK)) {
    ports_[1]->ReadControlPort(offset, data, size);
  } else if (ir.base == (pci_header_.bar[0] & PCI_BASE_ADDRESS_IO_MASK)) {
    ports_[0]->ReadPort(offset, data, size);
  } else if (ir.base == (pci_header_.bar[2] & PCI_BASE_ADDRESS_IO_MASK)) {
    ports_[1]->ReadPort(offset, data, size);
  }
}
