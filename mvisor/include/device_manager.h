#ifndef _MVISOR_DEVICE_MANAGER_H
#define _MVISOR_DEVICE_MANAGER_H

#include <set>
#include <string>
#include <deque>
#include "pci_device.h"
#include "device.h"

struct MemoryRegion;
struct IoHandler {
  IoResource io_resource;
  Device* device;
  const MemoryRegion* memory_region;
};

class DeviceManager;
class SystemRoot : public Device {
 public:
  SystemRoot(DeviceManager* manager) {
    manager_ = manager;
  }
};

class Machine;
class DeviceManager {
 public:
  DeviceManager(Machine* machine);
  ~DeviceManager();

  void IntializeQ35(); // called only once

  void RegisterDevice(Device* device);
  void UnregisterDevice(Device* device);

  void RegisterIoHandler(Device* device, const IoResource& io_resource);
  void UnregisterIoHandler(Device* device, const IoResource& io_resource);

  void PrintDevices();
  Device* LookupDeviceByName(const std::string name);
  PciDevice* LookupPciDevice(uint16_t bus, uint8_t devfn);

  /* call by machine */
  void HandleIo(uint16_t port, uint8_t* data, uint16_t size, int is_write, uint32_t count);
  void HandleMmio(uint64_t base, uint8_t* data, uint16_t size, int is_write);

  void* TranslateGuestMemory(uint64_t gpa);
  
  void SetIrq(uint32_t irq, uint32_t level);

  Machine* machine() { return machine_; }

 private:
  Machine* machine_;
  SystemRoot* root_;
  std::set<Device*> registered_devices_;
  std::deque<IoHandler*> mmio_handlers_;
  std::deque<IoHandler*> pio_handlers_;
};

#endif // _MVISOR_DEVICE_MANAGER_H
