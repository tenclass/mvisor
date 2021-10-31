#ifndef _MVISOR_DEVICES_DEBUG_CONSOLE_H
#define _MVISOR_DEVICES_DEBUG_CONSOLE_H

#include "device.h"

class DebugConsoleDevice : public Device {
 public:
  DebugConsoleDevice(DeviceManager* manager);
  void OnWrite(uint64_t base, uint8_t* data, uint32_t size);
  void OnRead(uint64_t base, uint8_t* data, uint32_t size);
};

#endif // _MVISOR_DEVICES_DEBUG_CONSOLE_H
