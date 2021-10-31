#ifndef _MVISOR_DEVICES_RTC_H
#define _MVISOR_DEVICES_RTC_H

#include "device.h"

class RtcDevice : public Device {
 public:
  RtcDevice(DeviceManager* manager);
  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
};

#endif // _MVISOR_DEVICES_RTC_H
