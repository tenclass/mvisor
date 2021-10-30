#ifndef _MVISOR_DEVICES_RTC_H
#define _MVISOR_DEVICES_RTC_H

#include "device.h"

class RtcDevice : public Device {
 public:
  RtcDevice(DeviceManager* manager);
  void OnRead(uint64_t base, uint8_t* data, uint32_t size);
  void OnWrite(uint64_t base, uint8_t* data, uint32_t size);
};

#endif // _MVISOR_DEVICES_RTC_H
