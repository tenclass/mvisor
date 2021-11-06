#ifndef _MVISOR_DEVICES_CMOS_H
#define _MVISOR_DEVICES_CMOS_H

#include "devices/device.h"

class CmosDevice : public Device {
 public:
  CmosDevice();
  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
};

#endif // _MVISOR_DEVICES_CMOS_H
