#ifndef _MVISOR_DEVICES_FLOPPY_H
#define _MVISOR_DEVICES_FLOPPY_H

#include "device.h"

class FloppyDevice : public Device {
 public:
  FloppyDevice(DeviceManager* manager);
  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
};

#endif // _MVISOR_DEVICES_FLOPPY_H
