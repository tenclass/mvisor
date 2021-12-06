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

#ifndef _MVISOR_DEVICE_MANAGER_H
#define _MVISOR_DEVICE_MANAGER_H

#include <set>
#include <string>
#include <deque>
#include <thread>
#include "pci_device.h"
#include "device.h"

struct MemoryRegion;
struct IoHandler {
  IoResource io_resource;
  Device* device;
  const MemoryRegion* memory_region;
};

struct IoEvent {
  Device*         device;
  uint64_t        address;
  uint32_t        length;
  uint64_t        datamatch;
  uint32_t        flags;
  int             fd;
};

/* Currently used for VGA auto refresh */
typedef std::chrono::steady_clock::time_point IoTimePoint;
typedef std::function<void()> VoidCallback;
struct IoTimer {
  Device*       device;
  bool          permanent;
  int           interval_ms;
  IoTimePoint   next_timepoint;
  VoidCallback  callback;
};

class Machine;
class DeviceManager {
 public:
  DeviceManager(Machine* machine, Device* root);
  ~DeviceManager();

  void RegisterDevice(Device* device);
  void UnregisterDevice(Device* device);
  void ResetDevices();

  void RegisterIoHandler(Device* device, const IoResource& io_resource);
  void UnregisterIoHandler(Device* device, const IoResource& io_resource);
  void RegisterIoEvent(Device* device, IoResourceType type, uint64_t address, uint32_t length, uint64_t datamatch);
  void UnregisterIoEvent(Device* device, IoResourceType type, uint64_t address);
  IoTimer* RegisterIoTimer(Device* device, int interval_ms, bool permanent, VoidCallback callback);
  void UnregisterIoTimer(IoTimer* timer);
  void ModifyIoTimer(IoTimer* timer, int interval_ms);

  void PrintDevices();
  Device* LookupDeviceByName(const std::string name);
  PciDevice* LookupPciDevice(uint16_t bus, uint8_t devfn);

  /* call by machine */
  void HandleIo(uint16_t port, uint8_t* data, uint16_t size, int is_write, uint32_t count, bool ioeventfd = false);
  void HandleMmio(uint64_t base, uint8_t* data, uint16_t size, int is_write, bool ioeventfd = false);

  void* TranslateGuestMemory(uint64_t gpa);
  
  void SetIrq(uint32_t irq, uint32_t level);
  void SignalMsi(uint64_t address, uint32_t data);

  Machine* machine() { return machine_; }
  Device* root() { return root_; }

 private:
  void InitializeIoEvent();
  void IoEventLoop();
  int CheckIoTimers();

  Machine*                machine_;
  Device*                 root_;
  std::set<Device*>       registered_devices_;
  std::deque<IoHandler*>  mmio_handlers_;
  std::deque<IoHandler*>  pio_handlers_;
  std::thread             ioevent_thread_;
  std::set<IoEvent*>      ioevents_;
  int                     epoll_fd_ = -1;
  int                     stop_event_fd_ = -1;
  std::set<IoTimer*>      iotimers_;
};

#endif // _MVISOR_DEVICE_MANAGER_H
