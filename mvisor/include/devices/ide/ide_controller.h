#ifndef _MVISOR_DEVICES_IDE_CONTROLLER_H
#define _MVISOR_DEVICES_IDE_CONTROLLER_H

#include <cstdint>
#include <vector>
#include "devices/device.h"
#include "devices/pci_device.h"
#include "devices/ide/ide_port.h"

#define IDE_MAX_PORTS 2
#define IDE_MAX_STORAGES_PER_PORT 2

class IdeControllerDevice : public PciDevice {
 public:
  IdeControllerDevice();
  ~IdeControllerDevice();

  void Connect();
  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length);

 protected:
  IdePort* ports_[IDE_MAX_PORTS];
};

#endif // _MVISOR_DEVICES_IDE_CONTROLLER_H
