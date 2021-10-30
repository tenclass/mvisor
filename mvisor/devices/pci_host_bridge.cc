#include "devices/pci_host_bridge.h"
#include "logger.h"

#define PCI_CONFIG_ADDRESS	0xcf8
#define PCI_CONFIG_DATA		0xcfc


PciHostBridgeDevice::PciHostBridgeDevice(DeviceManager* manager)
  : Device(manager) {
  name_ = "pci-host-bridge";
  
  AddIoResource(kIoResourceTypePio, PCI_CONFIG_ADDRESS, 4);
  AddIoResource(kIoResourceTypePio, PCI_CONFIG_DATA, 4);
}


void PciHostBridgeDevice::OnWrite(uint64_t base, uint8_t* data, uint32_t size) {
  MV_PANIC("not implemented base=0x%lx data=0x%x size=%x", base, *(uint32_t*)data, size);
}

void PciHostBridgeDevice::OnRead(uint64_t base, uint8_t* data, uint32_t size) {
  MV_PANIC("not implemented base=0x%lx data=0x%x size=%x", base, *(uint32_t*)data, size);
}
