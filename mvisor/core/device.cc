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
}

Device::~Device() {
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
  for (auto ir : io_resources_) {
    manager_->RegisterIoHandler(this, ir);
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

  for (auto &io_resource : io_resources_) {
    manager_->UnregisterIoHandler(this, io_resource);
  }
  manager_->UnregisterDevice(this);
}

void Device::AddIoResource(IoResourceType type, uint64_t base, uint64_t length, const char* name) {
  AddIoResource(type, base, length, nullptr, name);
}

void Device::AddIoResource(IoResourceType type, uint64_t base, uint64_t length, void* host_memory, const char* name) {
  IoResource io_resource = {
    .type = type,
    .base = base,
    .length = length,
    .name = name,
    .host_memory = host_memory
  };
  io_resources_.push_back(io_resource);
  if (connected_) {
    SetIoResourceEnabled(io_resource, true);
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
        SetIoResourceEnabled(*it, false);
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
        SetIoResourceEnabled(*it, false);
      }
      io_resources_.erase(it);
      break;
    }
  }
}

void Device::SetIoResourceEnabled(IoResource& ir, bool enabled) {
  if (enabled) {
    MV_ASSERT(!ir.enabled);
    if (ir.type == kIoResourceTypeRam) {
      MV_ASSERT(ir.host_memory);
      auto mm = manager_->machine()->memory_manager();
      ir.mapped_region = mm->Map(ir.base, ir.length, ir.host_memory, kMemoryTypeRam, ir.name);
    } else {
      manager_->RegisterIoHandler(this, ir);
    }
    ir.enabled = true;
  } else {
    if (!ir.enabled) {
      return;
    }
    if (ir.type == kIoResourceTypeRam) {
      MV_ASSERT(ir.mapped_region);
      auto mm = manager_->machine()->memory_manager();
      mm->Unmap(&ir.mapped_region);
    } else {
      manager_->UnregisterIoHandler(this, ir);
    }
    ir.enabled = false;
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
