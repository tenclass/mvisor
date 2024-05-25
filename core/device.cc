/* 
 * MVisor
 * Copyright (C) 2021 Terrence <terrence@tenclass.com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "device.h"
#include <cstring>
#include "logger.h"
#include "utilities.h"
#include "machine.h"
#include "device_manager.h"

Device::Device() {
  strcpy(name_, "unknown");

  /* If the device is not specified a default parent, attach it to the top system-root */
  set_default_parent_class("SystemRoot");
}

Device::~Device() {
  /* Free resources */
  for (auto resource : io_resources_) {
    delete resource;
  }
}

void Device::Reset() {
  /* Don't add anything here */
}

/* Connect() is called when device manager initialize */
void Device::Connect() {
  MV_ASSERT(manager_);

  for (auto child : children_) {
    auto device = dynamic_cast<Device*>(child);
    if (device) {
      device->manager_ = manager_;
      device->Connect();
    }
  }

  connected_ = true;
  manager_->RegisterDevice(this);
  for (auto resource : io_resources_) {
    SetIoResourceEnabled(resource, true);
  }
  if (parent_ && manager_->machine()->debug()) {
    MV_LOG("%s <= %s", parent_->name(), name_);
  }
}

/* Disconnect() is called when device manager is being destroyed */
void Device::Disconnect() {
  if (!connected_) {
    return;
  }
  connected_ = false;
  for (auto child : children_) {
    auto device = dynamic_cast<Device*>(child);
    if (device) {
      device->Disconnect();
    }
  }

  for (auto resource : io_resources_) {
    if (resource->enabled) {
      SetIoResourceEnabled(resource, false);
    }
  }
  manager_->UnregisterDevice(this);
}

void Device::AddIoResource(IoResourceType type, uint64_t base, uint64_t length, const char* name) {
  AddIoResource(type, base, length, name, nullptr, kIoResourceFlagNone);
}

void Device::AddIoResource(IoResourceType type, uint64_t base, uint64_t length, const char* name, void* host_memory, IoResourceFlag flags) {
  MV_ASSERT(length > 0);
  auto resource = new IoResource {
    .type = type,
    .base = base,
    .length = length,
    .name = name,
    .enabled = false,
    .host_memory = host_memory,
    .mapped_region = nullptr,
    .flags = flags
  };
  io_resources_.push_back(resource);
  if (connected_) {
    SetIoResourceEnabled(resource, true);
  }
}

void Device::RemoveIoResource(IoResourceType type, const char* name) {
  for (auto it = io_resources_.begin(); it != io_resources_.end(); it++) {
    auto resource = *it;
    if (resource->type == type &&
        (resource->name == name || (name && resource->name && strcmp(resource->name, name) == 0))
      ) {
      if (connected_) {
        SetIoResourceEnabled(resource, false);
      }
      io_resources_.erase(it);
      delete resource;
      return;
    }
  }
}

void Device::RemoveIoResource(IoResourceType type, uint64_t base) {
  for (auto it = io_resources_.begin(); it != io_resources_.end(); it++) {
    auto resource = *it;
    if (resource->type == type && resource->base == base) {
      if (connected_) {
        SetIoResourceEnabled(resource, false);
      }
      MV_ASSERT(!resource->enabled);
      io_resources_.erase(it);
      delete resource;
      return;
    }
  }
  MV_PANIC("%s not found type=%d base=0x%lx", name_, type, base);
}

void Device::SetIoResourceEnabled(IoResource* resource, bool enabled) {
  if (enabled) {
    MV_ASSERT(!resource->enabled);
    if (resource->type == kIoResourceTypeRam) {
      MV_ASSERT(resource->host_memory);
      auto mm = manager_->machine()->memory_manager();
      resource->mapped_region = mm->Map(resource->base, resource->length, resource->host_memory, kMemoryTypeRam, resource->name);
    } else {
      manager_->RegisterIoHandler(this, resource);
    }
    resource->enabled = true;
  } else {
    MV_ASSERT(resource->enabled);
    if (resource->type == kIoResourceTypeRam) {
      MV_ASSERT(resource->mapped_region);
      auto mm = manager_->machine()->memory_manager();
      mm->Unmap(&resource->mapped_region);
    } else {
      manager_->UnregisterIoHandler(this, resource);
    }
    resource->enabled = false;
  }
}

void Device::Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  MV_PANIC("not implemented %s base=0x%lx offset=0x%lx size=%d name=%s",
    name_, resource->base, offset, size, resource->name);
  MV_UNUSED(data);
}

void Device::Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  MV_PANIC("not implemented %s base=0x%lx offset=0x%lx size=%d data=0x%lx name=%s",
    name_, resource->base, offset, size, *(uint64_t*)data, resource->name);
}

bool Device::SaveState(MigrationWriter* writer) {
  MV_UNUSED(writer);
  return true;
}

bool Device::LoadState(MigrationReader* reader) {
  MV_UNUSED(reader);
  return true;
}


void Device::Schedule(VoidCallback callback) {
  manager_->io()->Schedule(this, std::move(callback));
}

IoTimer* Device::AddTimer(int64_t interval_ns, bool permanent, VoidCallback callback) {
  return manager_->io()->AddTimer(this, interval_ns, permanent, std::move(callback));
}

void Device::ModifyTimer(IoTimer* timer, int64_t interval_ns) {
  manager_->io()->ModifyTimer(timer, interval_ns);
}

void Device::RemoveTimer(IoTimer** timer) {
  MV_ASSERT(*timer != nullptr);
  manager_->io()->RemoveTimer(timer);
}

void Device::StartPolling(int fd, uint poll_mask, IoCallback callback) {
  manager_->io()->StartPolling(this, fd, poll_mask, std::move(callback));
}

void Device::StopPolling(int fd) {
  manager_->io()->StopPolling(fd);
}

