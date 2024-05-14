#ifndef _MVISOR_SERIAL_PORT_H
#define _MVISOR_SERIAL_PORT_H

#include "device_interface.h"
#include "device.h"

class SerialPort : public SerialPortInterface {
 public:
  virtual void OnMessage(uint8_t* data, size_t size);
  virtual void OnWritable();
  virtual void SendMessage(uint8_t* data, size_t size);
  virtual void Initialize(SerialDeviceInterface* device, uint32_t id);
  virtual std::list<SerialPortListener>::iterator RegisterSerialPortListener(SerialPortListener callback);
  virtual void UnregisterSerialPortListener(std::list<SerialPortListener>::iterator it);

  virtual void         set_ready(bool ready);
  virtual uint32_t     port_id() const { return port_id_; }
  virtual const char*  port_name() const { return port_name_; }
  virtual bool         ready() const { return ready_; }

 protected:
  SerialDeviceInterface*        device_;
  Device*                       real_device_;
  std::list<SerialPortListener> callbacks_;
  uint32_t  port_id_;
  char      port_name_[100];
  bool      ready_ = false;
  bool      writable_ = false;

};
#endif // _MVISOR_SERIAL_PORT_H
