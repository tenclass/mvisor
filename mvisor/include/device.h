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

#ifndef _MVISOR_DEVICE_H
#define _MVISOR_DEVICE_H


#include "utilities.h"
#include "object.h"
#include <string>
#include <list>
#include "vcpu.h"

enum IoResourceType {
  kIoResourceTypePio,
  kIoResourceTypeMmio,
  kIoResourceTypeRam
};

struct MemoryRegion;
struct IoResource {
  IoResourceType      type;
  uint64_t            base;
  uint64_t            length;
  const char*         name;
  bool                enabled;
  void*               host_memory;
  const MemoryRegion* mapped_region;
};

class DeviceManager;
class Device : public Object {
 public:
  Device();
  virtual ~Device();

  virtual void Connect();
  virtual void Disconnect();
  virtual void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void Reset();

  const std::list<IoResource>& io_resources() const { return io_resources_; }
  DeviceManager* manager() { return manager_; }
 protected:
  void AddIoResource(IoResourceType type, uint64_t base, uint64_t length, const char* name);
  void AddIoResource(IoResourceType type, uint64_t base, uint64_t length, void* host_memory, const char* name);
  void RemoveIoResource(IoResourceType type, const char* name);
  void RemoveIoResource(IoResourceType type, uint64_t base);
  void SetIoResourceEnabled(IoResource& ir, bool enabled);

  friend class DeviceManager;
  DeviceManager* manager_;

  std::list<IoResource> io_resources_;
  bool connected_ = false;
};

#endif // _MVISOR_DEVICE_H
