#include "devices/pci_host.h"
#include <cstring>
#include "logger.h"
#include "device_manager.h"

#define MCH_CONFIG_ADDR            0xcf8
#define MCH_CONFIG_DATA            0xcfc

#define MCH_PCIEXBAR 0x60
#define MCH_PCIEXBAR_SIZE 0x04

/*
 * PCI express ECAM (Enhanced Configuration Address Mapping) format.
 * AKA mmcfg address
 * bit 20 - 28: bus number
 * bit 15 - 19: device number
 * bit 12 - 14: function number
 * bit  0 - 11: offset in configuration space of a given device
 */
#define PCIE_MMCFG_SIZE_MAX             (1ULL << 29)
#define PCIE_MMCFG_SIZE_MIN             (1ULL << 20)
#define PCIE_MMCFG_BUS_BIT              20
#define PCIE_MMCFG_BUS_MASK             0x1ff
#define PCIE_MMCFG_DEVFN_BIT            12
#define PCIE_MMCFG_DEVFN_MASK           0xff
#define PCIE_MMCFG_CONFOFFSET_MASK      0xfff
#define PCIE_MMCFG_BUS(addr)            (((addr) >> PCIE_MMCFG_BUS_BIT) & \
                                         PCIE_MMCFG_BUS_MASK)
#define PCIE_MMCFG_DEVFN(addr)          (((addr) >> PCIE_MMCFG_DEVFN_BIT) & \
                                         PCIE_MMCFG_DEVFN_MASK)
#define PCIE_MMCFG_CONFOFFSET(addr)     ((addr) & PCIE_MMCFG_CONFOFFSET_MASK)


PciHostDevice::PciHostDevice() {
  name_ = "pci-host-bridge";
  
  header_.vendor_id = 0x8086;
  header_.device_id = 0x29c0;
  header_.class_code = 0x060000;
  header_.header_type = PCI_HEADER_TYPE_NORMAL;
  header_.subsys_vendor_id = 0x1af4;
  header_.subsys_id = 0x1100;

  AddIoResource(kIoResourceTypePio, MCH_CONFIG_ADDR, 4);
  AddIoResource(kIoResourceTypePio, MCH_CONFIG_DATA, 4);
}


void PciHostDevice::Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  if (ir.base == MCH_CONFIG_ADDR) {
    uint8_t* pointer = (uint8_t*)&pci_config_address_.data + offset;
    memcpy(pointer, data, size);
  } else if (ir.base == MCH_CONFIG_DATA) {
    if (size > 4)
      size = 4;
    
    PciDevice* pci_device = manager_->LookupPciDevice(0, pci_config_address_.devfn);
    if (pci_device) {
      pci_config_address_.reg_offset = offset;
      pci_device->WritePciConfigSpace(
        pci_config_address_.data & PCI_DEVICE_CONFIG_MASK, data, size);
    }
  } else if (ir.name && strcmp(ir.name, "pciexbar") == 0) {
    uint8_t devfn = PCIE_MMCFG_DEVFN(ir.base + offset);
    PciDevice* pci_device = manager_->LookupPciDevice(0, devfn);
    uint64_t addr = PCIE_MMCFG_CONFOFFSET(ir.base + offset);
    if (pci_device) {
      pci_device->WritePciConfigSpace(addr, data, size);
    } else {
      memset(data, 0xff, size);
      MV_LOG("failed to lookup pci devfn 0x%02x", devfn);
    }
  } else {
    MV_PANIC("not implemented base=0x%lx offset=0x%lx data=0x%x size=%x",
      ir.base, offset, *(uint32_t*)data, size);
  }
}

void PciHostDevice::Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  if (ir.base == MCH_CONFIG_ADDR) {
    uint8_t* pointer = (uint8_t*)&pci_config_address_.data + offset;
    memcpy(data, pointer, size);
  } else if (ir.base == MCH_CONFIG_DATA) {
    if (size > 4)
      size = 4;
    
    PciDevice* pci_device = manager_->LookupPciDevice(0, pci_config_address_.devfn);
    if (pci_device) {
      pci_config_address_.reg_offset = offset;
      pci_device->ReadPciConfigSpace(
        pci_config_address_.data & PCI_DEVICE_CONFIG_MASK, data, size);
    } else {
      memset(data, 0xff, size);
    }
  } else if (ir.name && strcmp(ir.name, "pciexbar") == 0) {
    uint8_t devfn = PCIE_MMCFG_DEVFN(ir.base + offset);
    PciDevice* pci_device = manager_->LookupPciDevice(0, devfn);
    uint64_t addr = PCIE_MMCFG_CONFOFFSET(ir.base + offset);
    if (pci_device) {
      pci_device->ReadPciConfigSpace(addr, data, size);
    } else {
      memset(data, 0xff, size);
      MV_LOG("failed to lookup pci devfn 0x%02x", devfn);
    }
  } else {
    MV_PANIC("not implemented base=0x%lx offset=0x%lx data=0x%x size=%x",
      ir.base, offset, *(uint32_t*)data, size);
  }
}

void PciHostDevice::WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
  PciDevice::WritePciConfigSpace(offset, data, length);
  if (ranges_overlap(offset, length, MCH_PCIEXBAR, MCH_PCIEXBAR_SIZE)) {
    MchUpdatePcieXBar();
  }
}

void PciHostDevice::MchUpdatePcieXBar() {
  uint32_t pciexbar = *(uint32_t*)(header_.data + MCH_PCIEXBAR);
  int enable = pciexbar & 1;
  uint32_t addr = pciexbar & Q35_MASK(64, 35, 28);
  uint64_t length = (1LL << 20) * 256;
  RemoveIoResource(kIoResourceTypeMmio, "pciexbar");
  if (enable) {
    AddIoResource(kIoResourceTypeMmio, addr, length, "pciexbar");
  }
}
