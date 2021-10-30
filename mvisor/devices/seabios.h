#ifndef _MVISOR_DEVICES_SEABIOS_H
#define _MVISOR_DEVICES_SEABIOS_H

#include "device.h"

class SeaBiosDevice : public Device {
 public:
  SeaBiosDevice(DeviceManager* manager);
  void OnWrite(uint64_t base, uint8_t* data, uint32_t size);
  void OnRead(uint64_t base, uint8_t* data, uint32_t size);
};

#endif // _MVISOR_DEVICES_SEABIOS_H
