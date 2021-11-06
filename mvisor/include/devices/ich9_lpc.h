#ifndef _MVISOR_DEVICES_LPC_H
#define _MVISOR_DEVICES_LPC_H

#include "devices/pci_device.h"

class Ich9LpcDevice : public PciDevice {
 public:
  Ich9LpcDevice();

  void WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length);
  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);

 private:
  void UpdatePmBaseSci();
  void UpdateRootComplexRegisterBLock();
};

#endif // _MVISOR_DEVICES_LPC_H
