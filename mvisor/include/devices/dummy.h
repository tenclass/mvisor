#ifndef _MVISOR_DEVICES_DUMMY_H
#define _MVISOR_DEVICES_DUMMY_H

#include "device.h"

class DummyDevice : public Device {
 public:
  DummyDevice(DeviceManager* manager);
  void OnWrite(uint64_t base, uint8_t* data, uint32_t size);
  void OnRead(uint64_t base, uint8_t* data, uint32_t size);
};

#endif // _MVISOR_DEVICES_DUMMY_H
