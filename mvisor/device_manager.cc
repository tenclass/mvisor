#include "device_manager.h"
#include "logger.h"
#include "devices/rtc.h"
#include "devices/ps2_controller.h"
#include "devices/seabios.h"
#include "devices/pci_host_bridge.h"

DeviceManager::DeviceManager(Machine* machine) : machine_(machine) {
  IntializeQ35();
}

DeviceManager::~DeviceManager() {
  for (auto device: devices_) {
    delete device;
  }
}

Device* DeviceManager::LookupDeviceByName(const std::string name) {
  for (auto device : devices_) {
    if (device->name() == name) {
      return device;
    }
  }
  return nullptr;
}

void DeviceManager::IntializeQ35() {
  PciHostBridgeDevice* pci_host_bridge = new PciHostBridgeDevice(this);
  RegisterDevice(pci_host_bridge);
  SeaBiosDevice* seabios = new SeaBiosDevice(this);
  RegisterDevice(seabios);
  RtcDevice* rtc = new RtcDevice(this);
  RegisterDevice(rtc);
  Ps2ControllerDevice* ps2 = new Ps2ControllerDevice(this);
  RegisterDevice(ps2);
}

void DeviceManager::PrintDevices() {
  for (auto device : devices_) {
    MV_LOG("device: %s", device->name().c_str());
    for (auto &io_resource : device->io_resources()) {
      if (io_resource.type == kIoResourceTypePio) {
        MV_LOG("\tio port 0x%lx-0x%lx", io_resource.base, io_resource.base + io_resource.length - 1);
      } else {
        MV_LOG("\tmmio address 0x%016lx-0x016%lx", io_resource.base, io_resource.base + io_resource.length);
      }
    }
  }
}

PciDevice* DeviceManager::LookupPciDevice(uint32_t device_number, uint32_t function_number) {
  for (auto device : devices_) {
    PciDevice* pci_device = dynamic_cast<PciDevice*>(device);
    if (pci_device &&
        pci_device->device_number_ == device_number && 
        pci_device->function_number_ == function_number) {
      return pci_device;
    }
  }
  return nullptr;
}


void DeviceManager::RegisterDevice(Device* device) {
  devices_.push_back(device);
  MV_LOG("device: %s", device->name().c_str());

  for (auto &io_resource : device->io_resources()) {
    if (io_resource.type == kIoResourceTypePio) {
      if (pio_handlers_.find(io_resource.base) != pio_handlers_.end()) {
        MV_PANIC("overlapped pio resource 0x%lx", io_resource.base);
      }
      pio_handlers_[io_resource.base] = IoHandler {
        .io_resource = io_resource,
        .device = device
      };
      MV_LOG("\tio port 0x%lx-0x%lx", io_resource.base, io_resource.base + io_resource.length - 1);
    } else {
      if (mmio_handlers_.find(io_resource.base) != mmio_handlers_.end()) {
        MV_PANIC("overlapped mmio resource 0x%016lx", io_resource.base);
      }
      mmio_handlers_[io_resource.base] = IoHandler {
        .io_resource = io_resource,
        .device = device
      };
      MV_LOG("\tmmio address 0x%016lx-0x016%lx", io_resource.base, io_resource.base + io_resource.length);
    }
  }
}


void DeviceManager::HandleIo(uint16_t port, uint8_t* data, uint16_t size, int is_write, uint32_t count) {
  auto it = pio_handlers_.upper_bound(port);
  if (it != pio_handlers_.begin())
    --it;
  if (it != pio_handlers_.end()) {
    auto &resource = it->second.io_resource;
    if (port >= resource.base && port < resource.base + resource.length) {
      Device* device = it->second.device;
      while (count--) {
        if (is_write) {
          device->OnWrite(port, data, size);
        } else {
          device->OnRead(port, data, size);
        }
        data += size;
      }
      return;
    }
  }
  MV_PANIC("unhandled io %s port: 0x%x size: %x data: %016lx count: %d",
    is_write ? "out" : "in", port, size, *(uint64_t*)data, count);
}

void DeviceManager::HandleMmio(uint64_t base, uint8_t* data, uint16_t size, int is_write) {
  auto it = pio_handlers_.upper_bound(base);
  if (it != pio_handlers_.begin())
    --it;
  if (it != pio_handlers_.end()) {
    auto &resource = it->second.io_resource;
    if (base >= resource.base && base < resource.base + resource.length) {
      Device* device = it->second.device;
      if (is_write) {
        device->OnWrite(base, data, size);
      } else {
        device->OnRead(base, data, size);
      }
      data += size;
      return;
    }
  }
  MV_PANIC("unhandled mmio %s base: 0x%016lx size: %x data: %016lx",
    is_write ? "write" : "read", base, size, *(uint64_t*)data);
}
