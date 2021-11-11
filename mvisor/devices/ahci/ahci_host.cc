#include "devices/ahci/ahci_host.h"
#include <cstring>
#include "logger.h"
#include "device_manager.h"
#include "devices/ahci/ahci_internal.h"
#include "devices/ide/ide_storage.h"

AhciHostDevice::AhciHostDevice() {
  name_ = "ahci-controller";
  /* FIXME: should gernerated by parent pci device */
  devfn_ = PCI_MAKE_DEVFN(0x1f, 2);
  
  /* PCI config */
  pci_header_.vendor_id = 0x8086;
  pci_header_.device_id = 0x2922;
  pci_header_.class_code = 0x010601;
  pci_header_.revision_id = 2;
  pci_header_.header_type = PCI_MULTI_FUNCTION | PCI_HEADER_TYPE_NORMAL;
  pci_header_.subsys_vendor_id = 0x1af4;
  pci_header_.subsys_id = 0x1100;
  pci_header_.command = PCI_COMMAND_IO | PCI_COMMAND_MEMORY;
  pci_header_.status = 0x10; /* Support capabilities */
  pci_header_.cacheline_size = 8;
  pci_header_.irq_pin = 1;

  pci_header_.data[0x90] = 1 << 6; /* Address Map Register - AHCI mode */

  /* Add SATA capability */
  const uint8_t sata_cap[] = {
    0x12, 0x00, 0x10, 0x00,
    0x48, 0x00, 0x00, 0x00
  };
  memcpy(pci_header_.data + 0xA8, sata_cap, sizeof(sata_cap));

  /* Add MSI */
  const uint8_t msi_cap[] = {
    0x05, 0xA8, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
  };
  memcpy(pci_header_.data + 0x80, msi_cap, sizeof(msi_cap));

  /* Point to the above capability */
  pci_header_.capability = 0x80;

  /* Memory bar */
  AddPciBar(5, 4096, kIoResourceTypeMmio);
}

AhciHostDevice::~AhciHostDevice() {
  for (auto port : ports_) {
    if (port) {
      delete port;
    }
  }
}

void AhciHostDevice::Connect() {
  PciDevice::Connect();

  num_ports_ = children_.size();
  /* Add storage devices */
  for (int i = 0; i < num_ports_; i++) {
    IdeStorageDevice* device = dynamic_cast<IdeStorageDevice*>(children_[i]);
    MV_ASSERT(device);
    if (!ports_[i]) {
      ports_[i] = new AhciPort(manager_, this, i);
    }
    ports_[i]->AttachDevice(device);
  }

  ResetHost();
}

void AhciHostDevice::ResetHost() {
  bzero(&host_control_, sizeof(host_control_));
  host_control_.global_host_control = HOST_CTL_AHCI_EN;
  host_control_.capabilities = (num_ports_ - 1) | (AHCI_NUM_COMMAND_SLOTS << 8) |
    (AHCI_SUPPORTED_SPEED_GEN1 << AHCI_SUPPORTED_SPEED) |
    HOST_CAP_NCQ | HOST_CAP_AHCI | HOST_CAP_64;
  host_control_.ports_implemented = (1 << num_ports_) - 1;
  host_control_.version = AHCI_VERSION_1_0;
  
  for (int i = 0; i < num_ports_; i++) {
    ports_[i]->Reset();
  }
}

void AhciHostDevice::CheckIrq() {
  host_control_.irq_status = 0;
  for (int i = 0; i < num_ports_; i++) {
    auto &pc = ports_[i]->port_control_;
    if (pc.irq_status & pc.irq_mask) {
      host_control_.irq_status |= (1 << i);
    }
  }
  if (host_control_.irq_status && (host_control_.global_host_control & HOST_CTL_IRQ_EN)) {
    manager_->SetIrq(pci_header_.irq_line, 1);
  } else {
    manager_->SetIrq(pci_header_.irq_line, 0);
  }
}

void AhciHostDevice::Read(const IoResource& ir, uint64_t offset, uint8_t* _data, uint32_t size) {
  MV_ASSERT(size == 4 && ir.type == kIoResourceTypeMmio);
  uint32_t* data = (uint32_t*)_data;

  if (offset >= AHCI_PORT_REGS_START_ADDR) {
    int port = (offset - AHCI_PORT_REGS_START_ADDR) >> 7;
    ports_[port]->Read(offset & AHCI_PORT_ADDR_OFFSET_MASK, data);
  } else {
    switch (offset / 4)
    {
    case AHCI_HOST_REG_CAP:
      *data = host_control_.capabilities;
      break;
    case AHCI_HOST_REG_CTL:
      *data = host_control_.global_host_control;
      break;
    case AHCI_HOST_REG_PORTS_IMPL:
      *data = host_control_.ports_implemented;
      break;
    default:
      MV_PANIC("not implemented %s base=0x%lx offset=0x%lx size=%d data=0x%lx",
        name_.c_str(), ir.base, offset, size, *data);
    }
  }
}

void AhciHostDevice::Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  MV_ASSERT(size == 4 && ir.type == kIoResourceTypeMmio);
  uint32_t value = *(uint32_t*)data;
  
  if (offset >= AHCI_PORT_REGS_START_ADDR) {
    int port = (offset - AHCI_PORT_REGS_START_ADDR) >> 7;
    ports_[port]->Write(offset & AHCI_PORT_ADDR_OFFSET_MASK, value);
  } else {
    switch (offset / 4)
    {
    case AHCI_HOST_REG_CTL:
      host_control_.global_host_control = value;
      break;
    default:
      MV_PANIC("not implemented %s base=0x%lx offset=0x%lx size=%d data=0x%lx",
        name_.c_str(), ir.base, offset, size, value);
    }
  }
}
