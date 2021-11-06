#ifndef _MVISOR_DEVICES_SERIAL_PORT_H
#define _MVISOR_DEVICES_SERIAL_PORT_H

#include "devices/device.h"

class SerialPortDevice : public Device {
 public:
  SerialPortDevice();
  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
};

#endif // _MVISOR_DEVICES_SERIAL_PORT_H
