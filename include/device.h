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


#include <string>
#include <list>
#include <mutex>

#include "utilities.h"
#include "object.h"
#include "io_thread.h"
#include "migration.h"


enum IoResourceType {
  kIoResourceTypeUnknown,
  kIoResourceTypePio,
  kIoResourceTypeMmio,
  kIoResourceTypeRam
};

enum IoResourceFlag {
  kIoResourceFlagNone = 0,
  kIoResourceFlagCoalescingMmio = 1
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
  IoResourceFlag      flags;
};

class DeviceManager;
class Device : public Object {
 public:
  Device();
  virtual ~Device();

  virtual void Connect();
  virtual void Disconnect();
  virtual void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void Reset();

  virtual bool SaveState(MigrationWriter* writer);
  virtual bool LoadState(MigrationReader* reader);

  /* Map to IoThread methods, and you don't need to pass the device object */
  IoTimer* AddTimer(int64_t interval_ns, bool permanent, VoidCallback callback);
  void ModifyTimer(IoTimer* timer, int64_t interval_ns);
  void RemoveTimer(IoTimer** timer);
  void Schedule(VoidCallback callback);
  void StartPolling(int fd, uint poll_mask, IoCallback callback);
  void StopPolling(int fd);

  void AddIoResource(IoResourceType type, uint64_t base, uint64_t length, const char* name);
  void AddIoResource(IoResourceType type, uint64_t base, uint64_t length, const char* name, void* host_memory, IoResourceFlag flags = kIoResourceFlagNone);
  void RemoveIoResource(IoResourceType type, const char* name);
  void RemoveIoResource(IoResourceType type, uint64_t base);
  void SetIoResourceEnabled(IoResource* resource, bool enabled);

  inline const std::list<IoResource*>& io_resources() const { return io_resources_; }
  inline DeviceManager* manager() { return manager_; }
  inline std::recursive_mutex& mutex() { return mutex_; }

 protected:
  friend class IoThread;
  friend class DeviceManager;
  DeviceManager* manager_;

  bool                    connected_ = false;
  std::list<IoResource*>  io_resources_;
  std::recursive_mutex    mutex_;
};

#endif // _MVISOR_DEVICE_H
