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
#include <mutex>
#include <vector>
#include <thread>
#include "pci_device.h"
#include "device.h"
#include "io_thread.h"

struct MemoryRegion;
struct IoHandler {
  const IoResource*   resource;
  Device*             device;
  const MemoryRegion* memory_region;
};

typedef std::function<void()> VoidCallback;
typedef std::function<uint (uint, uint, uint)> PciIrqTranslator; 

enum IoEventType {
  kIoEventPio,
  kIoEventMmio,
  kIoEventFd
};

struct IoEvent {
  IoEventType     type;
  Device*         device;
  uint64_t        address;
  uint32_t        length;
  uint64_t        datamatch;
  uint32_t        flags;
  int             fd;
};

struct IoAccounting {
  IoTimePoint     last_print_time;
  uint            total_pio = 0;
  uint            total_mmio = 0;
};

struct DirtyMemoryRegion {
  uint64_t hva;
  uint64_t begin;
  uint64_t end;
};

struct DirtyMemoryBitmap {
  struct DirtyMemoryRegion region;
  std::string data;
};

class Machine;
class DeviceManager {
 public:
  DeviceManager(Machine* machine, Device* root);
  ~DeviceManager();

  void RegisterDevice(Device* device);
  void UnregisterDevice(Device* device);
  void ResetDevices();

  void RegisterIoHandler(Device* device, const IoResource* resource);
  void UnregisterIoHandler(Device* device, const IoResource* resource);
  IoEvent* RegisterIoEvent(Device* device, IoResourceType type, uint64_t address);
  IoEvent* RegisterIoEvent(Device* device, IoResourceType type, uint64_t address, uint32_t length, uint64_t datamatch);
  void UnregisterIoEvent(Device* device, IoResourceType type, uint64_t address);
  void UnregisterIoEvent(IoEvent* event);
  void SetupCoalescingMmioRing(kvm_coalesced_mmio_ring* ring);
  void FlushCoalescingMmioBuffer();

  void PrintDevices();
  Device* LookupDeviceByClass(const std::string class_name);
  PciDevice* LookupPciDevice(uint16_t bus, uint8_t slot, uint8_t function);

  /* call by machine */
  void HandleIo(uint16_t port, uint8_t* data, uint16_t size, int is_write, uint32_t count, bool ioeventfd = false);
  void HandleMmio(uint64_t addr, uint8_t* data, uint16_t size, int is_write, bool ioeventfd = false);

  void* TranslateGuestMemory(uint64_t gpa);
  void AddDirtyMemory(uint64_t gpa, size_t size = 0);
  bool SaveState(MigrationWriter* writer);
  bool LoadState(MigrationReader* reader);
  
  /* IRQ / MSIs all are GSIs */
  void SetGsiLevel(uint gsi, uint level);
  void SetPciIrqLevel(PciDevice* pci, uint level);
  void SignalMsi(uint64_t address, uint32_t data);
  int AddMsiNotifier(uint64_t address, uint32_t data, int trigger_fd = -1);
  void RemoveMsiNotifier(int gsi, int trigger_fd);
  void SetPciIrqNotifier(PciDevice* pci, int trigger_fd, int unmask_fd = -1, bool assign = true);

  /* Called by PIIX3 or ICH9 LPC */
  void set_pci_irq_translator(PciIrqTranslator translator) { pci_irq_translator_ = translator; }

  inline Machine* machine() { return machine_; }
  inline Device* root() { return root_; }
  IoThread* io();

 private:
  void SetupIrqChip();
  void ResetGsiRoutingTable();
  void UpdateGsiRoutingTable();

 private:
  Machine*                            machine_;
  Device*                             root_;
  std::set<Device*>                   registered_devices_;
  std::deque<IoHandler*>              mmio_handlers_;
  std::deque<IoHandler*>              pio_handlers_;
  std::set<IoEvent*>                  ioevents_;
  std::recursive_mutex                mutex_;
  std::vector<kvm_irq_routing_entry>  gsi_routing_table_;
  int                                 next_gsi_ = 0;
  IoAccounting                        io_accounting_;
  kvm_coalesced_mmio_ring*            coalesced_mmio_ring_ = nullptr;
  std::recursive_mutex                coalesced_mmio_ring_mutex_;
  PciIrqTranslator                    pci_irq_translator_;
  std::unordered_set<uint>            pci_irq_raised_[32];
};

#endif // _MVISOR_DEVICE_MANAGER_H
