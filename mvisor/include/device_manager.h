#ifndef _MVISOR_DEVICE_MANAGER_H
#define _MVISOR_DEVICE_MANAGER_H

#include <vector>
#include <map>
#include <string>
#include "pci_device.h"
#include "device.h"

struct IoHandler {
  IoResource io_resource;
  Device* device;
};

class Machine;
class DeviceManager {
 public:
  DeviceManager(Machine* machine);
  ~DeviceManager();
  void IntializeQ35();

  void RegisterDevice(Device* device);
  void UnregisterDevice(Device* device);
  void PrintDevices();
  Device* LookupDeviceByName(const std::string name);
  PciDevice* LookupPciDevice(uint32_t device_number, uint32_t function_number);

  void HandleIo(uint16_t port, uint8_t* data, uint16_t size, int is_write, uint32_t count);
  void HandleMmio(uint64_t base, uint8_t* data, uint16_t size, int is_write);

  void ReadGuestMemory(uint64_t gpa, uint8_t* data, uint32_t size);
  void WriteGuestMemory(uint64_t gpa, uint8_t* data, uint32_t size);
  void* TranslateGuestMemory(uint64_t gpa);

  Machine* machine() { return machine_; }
 private:
  Machine* machine_;
  std::vector<Device*> devices_;
  std::map<uint64_t, IoHandler> mmio_handlers_;
  std::map<uint64_t, IoHandler> pio_handlers_;
};

#endif // _MVISOR_DEVICE_MANAGER_H
