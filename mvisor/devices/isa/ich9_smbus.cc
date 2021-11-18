#include "pci_device.h"

class SmBus : public PciDevice {
 public:
  SmBus() {
    devfn_ = PCI_MAKE_DEVFN(0x1f, 3);
    
    pci_header_.vendor_id = 0x8086;
    pci_header_.device_id = 0x2930;
    pci_header_.class_code = 0x0C0500;
    pci_header_.revision_id = 2;
    pci_header_.header_type = PCI_HEADER_TYPE_NORMAL;
    pci_header_.subsys_vendor_id = 0x1af4;
    pci_header_.subsys_id = 0x1100;
    pci_header_.irq_pin = 1;

    AddPciBar(4, 64, kIoResourceTypePio);
  }
};

DECLARE_DEVICE(SmBus);
