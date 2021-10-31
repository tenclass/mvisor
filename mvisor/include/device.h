#ifndef _MVISOR_DEVICE_H
#define _MVISOR_DEVICE_H

#include <string>
#include <list>
#include "vcpu.h"

enum IoResourceType {
  kIoResourceTypePio,
  kIoResourceTypeMmio
};

struct IoResource {
  IoResourceType type;
  uint64_t base;
  uint64_t length;
  const char* name;
};

class DeviceManager;
class Device {
 public:
  Device(DeviceManager* manager);
  virtual ~Device();

  virtual void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);

  const std::list<IoResource>& io_resources() const { return io_resources_; }
  const std::string& name() const { return name_; }
 protected:
  void AddIoResource(IoResourceType type, uint64_t base, uint64_t length) {
    AddIoResource(type, base, length, nullptr);
  }
  void AddIoResource(IoResourceType type, uint64_t base, uint64_t length, const char* name);
  void RemoveIoResource(IoResourceType type, const char* name);

  DeviceManager* manager_;
  std::string name_;
  std::list<IoResource> io_resources_;
};

#endif // _MVISOR_DEVICE_H
