#ifndef _MVISOR_DEVICES_PCIHOSTBRIDGE_H
#define _MVISOR_DEVICES_PCIHOSTBRIDGE_H

#include "pci_device.h"

class PciHostBridgeDevice : public PciDevice {
 public:
  PciHostBridgeDevice(DeviceManager* manager);
  void OnRead(uint64_t base, uint8_t* data, uint32_t size);
  void OnWrite(uint64_t base, uint8_t* data, uint32_t size);

private:
  PciConfigAddress pci_config_address_;
};

#endif // _MVISOR_DEVICES_PCIHOSTBRIDGE_H
