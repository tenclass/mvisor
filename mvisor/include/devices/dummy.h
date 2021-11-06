#ifndef _MVISOR_DEVICES_DUMMY_H
#define _MVISOR_DEVICES_DUMMY_H

#include "devices/device.h"

class DummyDevice : public Device {
 public:
  DummyDevice();
  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
};

#endif // _MVISOR_DEVICES_DUMMY_H
