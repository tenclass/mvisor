#ifndef _MVISOR_DEVICES_PCIHOSTBRIDGE_H
#define _MVISOR_DEVICES_PCIHOSTBRIDGE_H

#include "devices/pci_device.h"

class PciHostDevice : public PciDevice {
 public:
  PciHostDevice();
  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length);
private:
  void MchUpdatePcieXBar();

  PciConfigAddress pci_config_address_;
};

#endif // _MVISOR_DEVICES_PCIHOSTBRIDGE_H
