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

#include "device_manager.h"
#include <cstring>
#include <algorithm>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/eventfd.h>
#include <signal.h>
#include <poll.h>
#include "logger.h"
#include "memory_manager.h"
#include "machine.h"

#define IOEVENTFD_MAX_EVENTS  1000

/* SystemRoot is a motherboard that holds all the funcational devices */
class SystemRoot : public Device {
 public:
  SystemRoot() {}
 private:
  friend class DeviceManager;
};
DECLARE_DEVICE(SystemRoot);

inline IoThread* DeviceManager::io() {
  return machine_->io_thread_;
}

DeviceManager::DeviceManager(Machine* machine, Device* root) :
  machine_(machine), root_(root)
{
  root_->manager_ = this;
  /* Call Connect() on all devices and do the initialization
   * 1. reset device status
   * 2. register IO handlers
   */
  root_->Connect();

  /* Call Reset() on all devices after Connect() */
  ResetDevices();
}

DeviceManager::~DeviceManager() {
  if (root_) {
    /* Disconnect invoked recursively */
    root_->Disconnect();
  }
}

/* Called when system start or reset */
void DeviceManager::ResetDevices() {
  for (auto device : registered_devices_) {
    device->Reset();
  }
}

/* Used for debugging */
void DeviceManager::PrintDevices() {
  for (auto device : registered_devices_) {
    MV_LOG("Device: %s", device->name());
    for (auto &ir : device->io_resources()) {
      switch (ir.type)
      {
      case kIoResourceTypePio:
        MV_LOG("\tIO port 0x%lx-0x%lx", ir.base, ir.base + ir.length - 1);
        break;
      case kIoResourceTypeMmio:
        MV_LOG("\tMMIO address 0x%016lx-0x016%lx", ir.base, ir.base + ir.length - 1);
      case kIoResourceTypeRam:
        MV_LOG("\tRAM address 0x%016lx-0x016%lx", ir.base, ir.base + ir.length - 1);
        break;
      }
    }
  }
}

Device* DeviceManager::LookupDeviceByName(const std::string name) {
  for (auto device : registered_devices_) {
    if (device->name() == name) {
      return device;
    }
  }
  return nullptr;
}

PciDevice* DeviceManager::LookupPciDevice(uint16_t bus, uint8_t devfn) {
  for (auto device : registered_devices_) {
    PciDevice* pci_device = dynamic_cast<PciDevice*>(device);
    if (pci_device && pci_device->devfn_ == devfn) {
      return pci_device;
    }
  }
  return nullptr;
}


void DeviceManager::RegisterDevice(Device* device) {
  // Check devfn conflicts or reassign it
  PciDevice* pci_device = dynamic_cast<PciDevice*>(device);
  if (pci_device) {
    if (LookupPciDevice(pci_device->bus(), pci_device->devfn())) {
      MV_PANIC("PCI device function %x conflicts", pci_device->devfn());
      return;
    }
  }

  registered_devices_.insert(device);
}

void DeviceManager::UnregisterDevice(Device* device) {
  registered_devices_.erase(device);
}


void DeviceManager::RegisterIoHandler(Device* device, const IoResource& io_resource) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (io_resource.type == kIoResourceTypePio) {
    pio_handlers_.push_back(new IoHandler {
      .io_resource = io_resource,
      .device = device
    });
  } else if (io_resource.type == kIoResourceTypeMmio) {
    // Map the memory to type Device, access these regions will cause MMIO access fault
    const MemoryRegion* region = machine_->memory_manager()->Map(io_resource.base, io_resource.length,
      nullptr, kMemoryTypeDevice, io_resource.name);

    mmio_handlers_.push_back(new IoHandler {
      .io_resource = io_resource,
      .device = device,
      .memory_region = region
    });
  }
}

void DeviceManager::UnregisterIoHandler(Device* device, const IoResource& io_resource) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (io_resource.type == kIoResourceTypePio) {
    for (auto it = pio_handlers_.begin(); it != pio_handlers_.end(); it++) {
      if ((*it)->device == device && (*it)->io_resource.base == io_resource.base) {
        delete *it;
        pio_handlers_.erase(it);
        break;
      }
    }
  } else if (io_resource.type == kIoResourceTypeMmio) {
    for (auto it = mmio_handlers_.begin(); it != mmio_handlers_.end(); it++) {
      if ((*it)->device == device && (*it)->io_resource.base == io_resource.base) {
        delete *it;
        mmio_handlers_.erase(it);
        break;
      }
    }
  }
}

IoEvent* DeviceManager::RegisterIoEvent(Device* device, IoResourceType type, uint64_t address, uint32_t length, uint64_t datamatch) {
  IoEvent* event = new IoEvent {
    .type = kIoEventFd,
    .device = device,
    .address = address,
    .length = length,
    .datamatch = datamatch,
    .flags = length ? KVM_IOEVENTFD_FLAG_DATAMATCH : 0U,
    .fd = eventfd(0, 0)
  };
  if (type == kIoResourceTypePio) {
    event->flags |= KVM_IOEVENTFD_FLAG_PIO;
    event->type = kIoEventPio;
  } else {
    event->type = kIoEventMmio;
  }
  struct kvm_ioeventfd kvm_ioevent = {
    .datamatch = event->datamatch,
    .addr = event->address,
    .len = event->length,
    .fd = event->fd,
    .flags = event->flags
  };
  int ret = ioctl(machine_->vm_fd_, KVM_IOEVENTFD, &kvm_ioevent);
  if (ret < 0) {
    MV_PANIC("failed to register io event, ret=%d", ret);
  }

  event->request = io()->StartPolling(event->fd, POLLIN, [event, this](int events) {
    uint64_t tmp;
    read(event->fd, &tmp, sizeof(tmp));
    if (event->type == kIoEventMmio) {
      HandleMmio(event->address, (uint8_t*)&event->datamatch, event->length, true, true);
    } else if (event->type == kIoEventPio) {
      HandleIo(event->address, (uint8_t*)&event->datamatch, event->length, true, 1, true);
    }
  });

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  ioevents_.insert(event);
  return event;
}

IoEvent* DeviceManager::RegisterIoEvent(Device* device, IoResourceType type, uint64_t address) {
  return RegisterIoEvent(device, type, address, 0, 0);
}

void DeviceManager::UnregisterIoEvent(IoEvent* event) {
  if (event->request) {
    io()->CancelRequest(event->request);
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if (event->type == kIoEventMmio || event->type == kIoEventPio) {
    struct kvm_ioeventfd kvm_ioevent = {
      .datamatch = event->datamatch,
      .addr = event->address,
      .len = event->length,
      .fd = event->fd,
      .flags = event->flags | KVM_IOEVENTFD_FLAG_DEASSIGN
    };
    int ret = ioctl(machine_->vm_fd_, KVM_IOEVENTFD, &kvm_ioevent);
    if (ret < 0) {
      MV_PANIC("failed to unregister io event, ret=%d", ret);
    }
  }

  ioevents_.erase(event);
  delete event;
}

void DeviceManager::UnregisterIoEvent(Device* device, IoResourceType type, uint64_t address) {
  mutex_.lock();
  auto it = std::find_if(ioevents_.begin(), ioevents_.end(), [=](auto &e) {
    return e->device == device && e->address == address &&
      ((type == kIoResourceTypePio) == !!(e->flags & KVM_IOEVENTFD_FLAG_PIO));
  });
  MV_ASSERT(it != ioevents_.end());
  auto event = *it;
  mutex_.unlock();
  UnregisterIoEvent(event);
}

/* IO ports may overlap like MMIO addresses.
 * Use para-virtual drivers instead of IO operations to improve performance.
 * It seems no race condition would happen among vCPUs
 */
void DeviceManager::HandleIo(uint16_t port, uint8_t* data, uint16_t size, int is_write, uint32_t count, bool ioeventfd) {
  int it_count = 0;
  std::deque<IoHandler*>::iterator it;

  mutex_.lock();
  for (it = pio_handlers_.begin(); it != pio_handlers_.end(); it++, it_count++) {
    auto &resource = (*it)->io_resource;
    if (port >= resource.base && port < resource.base + resource.length) {
      Device* device = (*it)->device;
      if (it_count >= 3) {
        // Move to the front for faster access next time
        pio_handlers_.push_front(*it);
        pio_handlers_.erase(it);
        --it;
      }
      mutex_.unlock();

      auto start_time = std::chrono::steady_clock::now();
      uint8_t* ptr = data;
      for (uint32_t i = 0; i < count; i++) {
        if (is_write) {
          device->Write(resource, port - resource.base, ptr, size);
        } else {
          device->Read(resource, port - resource.base, ptr, size);
        }
        ptr += size;
      }

      if (machine_->debug()) {
        auto cost_us = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - start_time).count();
        if ((!ioeventfd && cost_us >= 1000) || (ioeventfd && cost_us >= 10000)) {
          MV_LOG("%s SLOW %sio %s port=0x%x size=%u data=%lx cost=%.3lfms", device->name(), ioeventfd ? "EVENT " : "",
            is_write ? "out" : "in", port, size, *(uint64_t*)data, double(cost_us) / 1000.0);
        }
      }
      return;
    }
  }

  /* Accessing invalid port always returns error */
  mutex_.unlock();
  memset(data, 0xFF, size);
  if (machine_->debug()) {
    /* Not allowed unhandled IO for debugging */
    MV_LOG("unhandled io %s port: 0x%x size: %x data: %016lx count: %d",
      is_write ? "out" : "in", port, size, *(uint64_t*)data, count);
  }
}


/* Use for loop to find MMIO handlers is stupid, unless we are sure addresses not overlapped.
 * But moving the handler to the front works great for now, 99% MMIOs are concentrated on
 * a few devices
 * Race condition could happen among multiple vCPUs, should be handled carefully in Read / Write
 */
void DeviceManager::HandleMmio(uint64_t base, uint8_t* data, uint16_t size, int is_write, bool ioeventfd) {
  std::deque<IoHandler*>::iterator it;
  int it_count = 0;

  mutex_.lock();
  for (it = mmio_handlers_.begin(); it != mmio_handlers_.end(); it++, it_count++) {
    auto &resource = (*it)->io_resource;
    if (base >= resource.base && base < resource.base + resource.length) {
      Device* device = (*it)->device;

      if (it_count >= 3) {
        // Move to the front for faster access next time
        mmio_handlers_.push_front(*it);
        mmio_handlers_.erase(it);
        --it;
      }
      mutex_.unlock();

      auto start_time = std::chrono::steady_clock::now();
      if (is_write) {
        device->Write(resource, base - resource.base, data, size);
      } else {
        device->Read(resource, base - resource.base, data, size);
      }

      if (machine_->debug()) {
        auto cost_us = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - start_time).count();
        if ((!ioeventfd && cost_us >= 1000) || (ioeventfd && cost_us >= 10000)) {
          MV_LOG("%s SLOW %smmio %s addr=0x%lx size=%u data=%lx cost=%.3lfms", device->name(), ioeventfd ? "EVENT " : "",
            is_write ? "out" : "in", base, size, *(uint64_t*)data, double(cost_us) / 1000.0);
        }
      }
      return;
    }
  }
  MV_PANIC("unhandled mmio %s base: 0x%016lx size: %x data: %016lx",
    is_write ? "write" : "read", base, size, *(uint64_t*)data);
}

/* Get the host memory address of a guest physical address */
void* DeviceManager::TranslateGuestMemory(uint64_t gpa) {
  auto memory_manger = machine_->memory_manager();
  void* host = memory_manger->GuestToHostAddress(gpa);
  return host;
}

/* Maybe we should have an IRQ manager or just let KVM do this? */
void DeviceManager::SetIrq(uint32_t irq, uint32_t level) {
  /* Send an IRQ to the guest */
  struct kvm_irq_level irq_level = {
    .irq = irq,
    .level = level
  };
  if (ioctl(machine_->vm_fd_, KVM_IRQ_LINE, &irq_level) != 0) {
    MV_PANIC("KVM_IRQ_LINE failed");
  }
}

void DeviceManager::SignalMsi(uint64_t address, uint32_t data) {
  struct kvm_msi msi = {
    .address_lo = (uint32_t)(address),
    .address_hi = (uint32_t)(address >> 32),
    .data = data
  };
  int ret = ioctl(machine_->vm_fd_, KVM_SIGNAL_MSI, &msi);
  if (ret != 1) {
    MV_PANIC("KVM_SIGNAL_MSI ret=%d", ret);
  }
}

