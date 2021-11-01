#include "device.h"
#include <cstring>
#include "logger.h"
#include "device_manager.h"

Device::Device(DeviceManager* manager)
  : manager_(manager) {
  manager_->RegisterDevice(this);
}

Device::~Device() {
  for (auto &io_resource : io_resources_) {
    manager_->UnregisterIoHandler(this, io_resource);
  }
  manager_->UnregisterDevice(this);
}

void Device::AddIoResource(IoResourceType type, uint64_t base, uint64_t length, const char* name) {
  IoResource io_resource = {
    .type = type,
    .base = base,
    .length = length,
    .name = name
  };
  io_resources_.push_back(std::move(io_resource));
  manager_->RegisterIoHandler(this, io_resource);
}

void Device::RemoveIoResource(IoResourceType type, const char* name) {
  for (auto it = io_resources_.begin(); it != io_resources_.end(); it++) {
    if (it->type == type &&
        (it->name == name ||
          (name && it->name && strcmp(it->name, name) == 0)
        )
      ) {
      manager_->UnregisterIoHandler(this, *it);
      io_resources_.erase(it);
      break;
    }
  }
}

void Device::Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  MV_PANIC("not implemented %s base=0x%lx offset=0x%lx size=%d",
    name_.c_str(), ir.base, offset, size);
}

void Device::Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  MV_PANIC("not implemented %s base=0x%lx offset=0x%lx size=%d",
    name_.c_str(), ir.base, offset, size);
}
