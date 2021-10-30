#include "devices/pci_host_bridge.h"
#include <cstring>
#include "logger.h"
#include "device_manager.h"

#define PCI_CONFIG_ADDRESS	0xcf8
#define PCI_CONFIG_DATA		0xcfc


PciHostBridgeDevice::PciHostBridgeDevice(DeviceManager* manager)
  : PciDevice(manager) {
  name_ = "pci-host-bridge";
  
  header_.vendor_id	= 0x8086;
  header_.device_id	= 0x29c0;
  header_.class_code[2]	= 0x03;
  header_.header_type	= PCI_HEADER_TYPE_NORMAL;
  header_.subsys_vendor_id = 0x1af4;
  header_.subsys_id	= 0x1100;

  AddIoResource(kIoResourceTypePio, PCI_CONFIG_ADDRESS, 4);
  AddIoResource(kIoResourceTypePio, PCI_CONFIG_DATA, 4);
}


void PciHostBridgeDevice::OnWrite(uint64_t base, uint8_t* data, uint32_t size) {
  if (base == PCI_CONFIG_ADDRESS) {
    uint8_t* pointer = (uint8_t*)&pci_config_address_.data + base - PCI_CONFIG_ADDRESS;
    memcpy(pointer, data, size);
  } else if (base == PCI_CONFIG_DATA) {
    if (size > 4)
      size = 4;
    
    PciDevice* pci_device = manager_->LookupPciDevice(pci_config_address_.device_number,
      pci_config_address_.function_number);
    if (pci_device) {
      pci_config_address_.reg_offset = base - PCI_CONFIG_DATA;
      pci_device->WritePciConfigSpace(
        pci_config_address_.data & PCI_DEVICE_CONFIG_MASK, data, size);
    } else {
      MV_LOG("failed to lookup pci device %d function %d", pci_config_address_.device_number,
        pci_config_address_.function_number);
    }
  } else {
    MV_PANIC("not implemented base=0x%lx data=0x%x size=%x", base, *(uint32_t*)data, size);
  }
}

void PciHostBridgeDevice::OnRead(uint64_t base, uint8_t* data, uint32_t size) {
  if (base >= PCI_CONFIG_ADDRESS && base < PCI_CONFIG_ADDRESS + 4) {
    uint8_t* pointer = (uint8_t*)&pci_config_address_.data + base - PCI_CONFIG_ADDRESS;
    memcpy(data, pointer, size);
  } else if (base >= PCI_CONFIG_DATA && base < PCI_CONFIG_DATA + 4) {
    if (size > 4)
      size = 4;
    
    PciDevice* pci_device = manager_->LookupPciDevice(pci_config_address_.device_number,
      pci_config_address_.function_number);
    if (pci_device) {
      pci_config_address_.reg_offset = base - PCI_CONFIG_DATA;
      pci_device->ReadPciConfigSpace(
        pci_config_address_.data & PCI_DEVICE_CONFIG_MASK, data, size);
    } else {
      memset(data, 0xff, size);
      MV_LOG("failed to lookup pci device %d function %d", pci_config_address_.device_number,
        pci_config_address_.function_number);
    }
  } else {
    MV_PANIC("not implemented base=0x%lx data=0x%x size=%x", base, *(uint32_t*)data, size);
  }
}
