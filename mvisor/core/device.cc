#include "device.h"
#include <cstring>
#include "logger.h"
#include "utility.h"
#include "device_manager.h"

Device::Device() {
  strcpy(name_, "unknown");
}

Device::~Device() {
  if (connected_) {
    Disconnect();
  }
}

Device* Device::Create(const char* class_name) {
  return dynamic_cast<Device*>(realize_class(class_name));
}

void Device::AddChild(Device* device) {
  device->parent_ = this;
  children_.push_back(device);
}

void Device::Connect() {
  MV_ASSERT(manager_);

  for (auto child : children_) {
    child->manager_ = manager_;
    child->Connect();
  }

  connected_ = true;
  manager_->RegisterDevice(this);
  for (auto ir : io_resources_) {
    manager_->RegisterIoHandler(this, ir);
  }
  if (parent_) {
    MV_LOG("%s <= %s", parent_->name_, name_);
  }
}

void Device::Disconnect() {
  connected_ = false;
  for (auto child : children_) {
    child->Disconnect();
  }

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
  if (connected_) {
    manager_->RegisterIoHandler(this, io_resource);
  }
}

void Device::RemoveIoResource(IoResourceType type, const char* name) {
  for (auto it = io_resources_.begin(); it != io_resources_.end(); it++) {
    if (it->type == type &&
        (it->name == name ||
          (name && it->name && strcmp(it->name, name) == 0)
        )
      ) {
      if (connected_) {
        manager_->UnregisterIoHandler(this, *it);
      }
      io_resources_.erase(it);
      break;
    }
  }
}

void Device::RemoveIoResource(IoResourceType type, uint64_t base) {
  for (auto it = io_resources_.begin(); it != io_resources_.end(); it++) {
    if (it->type == type && it->base == base) {
      if (connected_) {
        manager_->UnregisterIoHandler(this, *it);
      }
      io_resources_.erase(it);
      break;
    }
  }
}

void Device::Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  MV_PANIC("not implemented %s base=0x%lx offset=0x%lx size=%d",
    name_, ir.base, offset, size);
}

void Device::Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  MV_PANIC("not implemented %s base=0x%lx offset=0x%lx size=%d data=0x%lx",
    name_, ir.base, offset, size, *(uint64_t*)data);
}
