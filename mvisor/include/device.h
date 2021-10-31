#ifndef _MVISOR_DEVICE_H
#define _MVISOR_DEVICE_H

#include <string>
#include <vector>
#include "vcpu.h"

enum IoResourceType {
  kIoResourceTypePio,
  kIoResourceTypeMmio
};

struct IoResource {
  IoResourceType type;
  uint64_t base;
  uint64_t length;
};

class DeviceManager;
class Device {
 public:
  Device(DeviceManager* manager);
  virtual ~Device();

  virtual void OnRead(uint64_t base, uint8_t* data, uint32_t size);
  virtual void OnWrite(uint64_t base, uint8_t* data, uint32_t size);

  const std::vector<IoResource>& io_resources() const { return io_resources_; }
  const std::string& name() const { return name_; }
 protected:
  void AddIoResource(IoResourceType type, uint64_t base, uint64_t length);

  DeviceManager* manager_;
  std::string name_;
  std::vector<IoResource> io_resources_;
};

#endif // _MVISOR_DEVICE_H
