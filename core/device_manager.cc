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

#include <algorithm>

#include <cstring>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/eventfd.h>
#include <signal.h>

#include "logger.h"
#include "memory_manager.h"
#include "machine.h"

#define IOEVENTFD_MAX_EVENTS  1000

/* SystemRoot is a motherboard that holds all the funcational devices */
class SystemRoot : public Device {
 public:
  SystemRoot() {
    default_parent_classes_.clear();
  }
 private:
  friend class DeviceManager;
};
DECLARE_DEVICE(SystemRoot);

IoThread* DeviceManager::io() {
  return machine_->io_thread_;
}

DeviceManager::DeviceManager(Machine* machine, Device* root) :
  machine_(machine), root_(root)
{
  root_->manager_ = this;

  /* Initialize IRQ chip */
  SetupIrqChip();
  
  /* Initialize GSI routing table */
  ResetGsiRoutingTable();

  /* Call Connect() on all devices and do the initialization
   * 1. reset device status
   * 2. register IO handlers
   */
  root_->Connect();
}

DeviceManager::~DeviceManager() {
  if (root_) {
    /* Disconnect invoked recursively */
    root_->Disconnect();
  }
  
  MV_ASSERT(ioevents_.empty());
}

/* Called when system start or reset */
void DeviceManager::ResetDevices() {
  ResetGsiRoutingTable();

  for (auto device : registered_devices_) {
    device->Reset();
  }
}

/* Used for debugging */
void DeviceManager::PrintDevices() {
  for (auto device : registered_devices_) {
    MV_LOG("Device: %s", device->name());
    for (auto resource : device->io_resources()) {
      switch (resource->type)
      {
      case kIoResourceTypePio:
        MV_LOG("\tIO   port    0x%lx-0x%lx %d", resource->base, resource->base + resource->length - 1, resource->enabled);
        break;
      case kIoResourceTypeMmio:
        MV_LOG("\tMMIO address 0x%016lx-0x016%lx %d", resource->base, resource->base + resource->length - 1, resource->enabled);
        break;
      case kIoResourceTypeRam:
        MV_LOG("\tRAM  address 0x%016lx-0x016%lx %d", resource->base, resource->base + resource->length - 1, resource->enabled);
        break;
      case kIoResourceTypeUnknown:
        MV_PANIC("unknown resource type");
      }
    }
  }
}

Device* DeviceManager::LookupDeviceByClass(const std::string class_name) {
  for (auto device : registered_devices_) {
    if (device->classname() == class_name) {
      return device;
    }
  }
  return nullptr;
}

PciDevice* DeviceManager::LookupPciDevice(uint16_t bus, uint8_t slot, uint8_t function) {
  for (auto device : registered_devices_) {
    PciDevice* pci_device = dynamic_cast<PciDevice*>(device);
    if (pci_device && pci_device->bus_ == bus && pci_device->slot_ == slot && pci_device->function_ == function) {
      return pci_device;
    }
  }
  return nullptr;
}


void DeviceManager::RegisterDevice(Device* device) {
  // Check devfn conflicts or reassign it
  PciDevice* pci = dynamic_cast<PciDevice*>(device);
  if (pci) {
    /* Auto generate pci address */
    if (pci->slot_ == 0xFF) {
      auto parent = dynamic_cast<const PciDevice*>(pci->parent());
      /* Check if Multi-function */
      if (parent && (parent->pci_header_.header_type & 0x80)) {
        pci->slot_ = parent->slot_;
        for (uint i = 0; i < 8; i++) {
          if (!LookupPciDevice(pci->bus_, pci->slot_, i)) {
            pci->function_ = i;
            break;
          }
        }
      } else {
        for (uint i = 1; i < 0x1F; i++) {
          if (!LookupPciDevice(pci->bus_, i, pci->function_)) {
            pci->slot_ = i;
            break;
          }
        }
      }
    }

    if (LookupPciDevice(pci->bus_, pci->slot_, pci->function_)) {
      MV_PANIC("PCI device %x.%x conflicts", pci->slot_, pci->function_);
      return;
    }
    if (machine_->debug_) {
      MV_LOG("register PCI %s at %x:%x.%x", pci->name_, pci->bus_, pci->slot_, pci->function_);
    }
  }

  registered_devices_.insert(device);
}

void DeviceManager::UnregisterDevice(Device* device) {
  registered_devices_.erase(device);
}

void DeviceManager::RegisterIoHandler(Device* device, const IoResource* resource) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  /* Check if resources are overlapped */
  if (resource->type == kIoResourceTypePio) {
    auto found = std::find_if(pio_handlers_.begin(), pio_handlers_.end(), [resource](auto& o) {
      return ranges_overlap(o->resource->base, o->resource->length, resource->base, resource->length);
    });
    if (found != pio_handlers_.end()) {
      MV_PANIC("%s port 0x%x(0x%x) overlapped by %s port 0x%x(0x%x)", device->name(), resource->base,
        resource->length, (*found)->device->name(), (*found)->resource->base, (*found)->resource->length);
    }
  } else if (resource->type == kIoResourceTypeMmio) {
    auto found = std::find_if(pio_handlers_.begin(), pio_handlers_.end(), [resource](auto& o) {
      return ranges_overlap(o->resource->base, o->resource->length, resource->base, resource->length);
    });
    if (found != pio_handlers_.end()) {
      MV_PANIC("%s mmio 0x%x(0x%x) overlapped by %s mmio 0x%x(0x%x)", device->name(), resource->base,
        resource->length, (*found)->device->name(), (*found)->resource->base, (*found)->resource->length);
    }
  }

  if (resource->type == kIoResourceTypePio) {
    pio_handlers_.push_back(new IoHandler {
      .resource = resource,
      .device = device,
      .memory_region = nullptr
    });
  } else if (resource->type == kIoResourceTypeMmio) {
    // Map the memory to type Device. Accessing these regions will cause MMIO access fault
    auto region = machine_->memory_manager()->Map(resource->base, resource->length,
      nullptr, kMemoryTypeDevice, resource->name);

    mmio_handlers_.push_back(new IoHandler {
      .resource = resource,
      .device = device,
      .memory_region = region
    });

    if (resource->flags & kIoResourceFlagCoalescingMmio) {
      kvm_coalesced_mmio_zone zone = {
        .addr = resource->base,
        .size =  (uint32_t)resource->length,
        .pad = 0
      };
      if (ioctl(machine_->vm_fd_, KVM_REGISTER_COALESCED_MMIO, &zone) != 0) {
        MV_PANIC("failed to register coaleascing MMIO");
      }
    }
  }
}

void DeviceManager::UnregisterIoHandler(Device* device, const IoResource* resource) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (resource->type == kIoResourceTypePio) {
    for (auto it = pio_handlers_.begin(); it != pio_handlers_.end(); it++) {
      if ((*it)->device == device && (*it)->resource->base == resource->base) {
        delete *it;
        pio_handlers_.erase(it);
        break;
      }
    }
  } else if (resource->type == kIoResourceTypeMmio) {
    for (auto it = mmio_handlers_.begin(); it != mmio_handlers_.end(); it++) {
      if ((*it)->device == device && (*it)->resource->base == resource->base) {
        machine_->memory_manager()->Unmap(&(*it)->memory_region);
        delete *it;
        mmio_handlers_.erase(it);
        break;
      }
    }

    if (resource->flags & kIoResourceFlagCoalescingMmio) {
      kvm_coalesced_mmio_zone zone = {
        .addr = resource->base,
        .size = (uint32_t)resource->length,
        .pad = 0
      };
      if (ioctl(machine_->vm_fd_, KVM_UNREGISTER_COALESCED_MMIO, &zone) != 0) {
        MV_PANIC("failed to unregister coaleascing MMIO");
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
    .flags = event->flags,
    .pad = {0}
  };
  int ret = ioctl(machine_->vm_fd_, KVM_IOEVENTFD, &kvm_ioevent);
  if (ret < 0) {
    MV_PANIC("%s failed to register io event, ret=%d", device->name(), ret);
  }
  if (machine_->debug_) {
    MV_LOG("%s register IO event address=0x%lx length=%lu fd=%d", device->name(), address, length, event->fd);
  }

  io()->StartPolling(device, event->fd, EPOLLIN, [event, this](int events) {
    if (events & EPOLLIN) {
      uint64_t tmp;
      read(event->fd, &tmp, sizeof(tmp));
      if (event->type == kIoEventMmio) {
        HandleMmio(event->address, (uint8_t*)&event->datamatch, event->length, true, true);
      } else if (event->type == kIoEventPio) {
        HandleIo(event->address, (uint8_t*)&event->datamatch, event->length, true, 1, true);
      }
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
  io()->StopPolling(event->fd);

  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if (event->type == kIoEventMmio || event->type == kIoEventPio) {
    struct kvm_ioeventfd kvm_ioevent = {
      .datamatch = event->datamatch,
      .addr = event->address,
      .len = event->length,
      .fd = event->fd,
      .flags = event->flags | KVM_IOEVENTFD_FLAG_DEASSIGN,
      .pad = {0}
    };
    int ret = ioctl(machine_->vm_fd_, KVM_IOEVENTFD, &kvm_ioevent);
    if (ret < 0) {
      MV_PANIC("failed to unregister io event, ret=%d", ret);
    }
  }

  ioevents_.erase(event);
  safe_close(&event->fd);
  delete event;
}

void DeviceManager::UnregisterIoEvent(Device* device, IoResourceType type, uint64_t address) {
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  auto it = std::find_if(ioevents_.begin(), ioevents_.end(), [=](auto &e) {
    return e->device == device && e->address == address &&
      ((type == kIoResourceTypePio) == !!(e->flags & KVM_IOEVENTFD_FLAG_PIO));
  });
  if (it == ioevents_.end()) {
    return;
  }
  auto event = *it;
  lock.unlock();
  UnregisterIoEvent(event);
}

void DeviceManager::SetupCoalescingMmioRing(kvm_coalesced_mmio_ring* ring) {
  if (coalesced_mmio_ring_ == nullptr) {
    coalesced_mmio_ring_ = ring;
  }
}

void DeviceManager::FlushCoalescingMmioBuffer() {
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  if (!coalesced_mmio_ring_) {
    return;
  }
  if (coalesced_mmio_ring_->first == coalesced_mmio_ring_->last) {
    return;
  }

  uint max_entries = ((PAGE_SIZE - sizeof(struct kvm_coalesced_mmio_ring)) / \
    sizeof(struct kvm_coalesced_mmio));
  while (coalesced_mmio_ring_->first != coalesced_mmio_ring_->last) {
    struct kvm_coalesced_mmio *m = &coalesced_mmio_ring_->coalesced_mmio[coalesced_mmio_ring_->first];
    coalesced_mmio_ring_->first = (coalesced_mmio_ring_->first + 1) % max_entries;

    lock.unlock();
    for (size_t i = m->len; i < sizeof(m->data); i++)
      m->data[i] = 0;
    if (m->pio == 1) {
      machine_->device_manager()->HandleIo(m->phys_addr, m->data, m->len, 1, 1);
    } else {
      machine_->device_manager()->HandleMmio(m->phys_addr, m->data, m->len, 1);
    }
    lock.lock();
  }
}

/* IO ports may overlap like MMIO addresses.
 * Use para-virtual drivers instead of IO operations to improve performance.
 * It seems no race condition would happen among vCPUs
 */
void DeviceManager::HandleIo(uint16_t port, uint8_t* data, uint16_t size, int is_write, uint32_t count, bool ioeventfd) {
  int it_count = 0;
  std::deque<IoHandler*>::iterator it;

  std::unique_lock<std::recursive_mutex> lock(mutex_);
  for (it = pio_handlers_.begin(); it != pio_handlers_.end(); it++, it_count++) {
    auto resource = (*it)->resource;
    if (port >= resource->base && port < resource->base + resource->length) {
      Device* device = (*it)->device;
      if (it_count >= 3) {
        // Move to the front for faster access next time
        pio_handlers_.push_front(*it);
        pio_handlers_.erase(it);
        --it;
      }
      lock.unlock();

      auto start_time = std::chrono::steady_clock::now();
      std::lock_guard<std::recursive_mutex> device_lock(device->mutex_);
      auto ptr = data;
      for (uint32_t i = 0; i < count; i++) {
        if (is_write) {
          device->Write(resource, port - resource->base, ptr, size);
        } else {
          device->Read(resource, port - resource->base, ptr, size);
        }
        ptr += size;
      }

      if (machine_->debug() && !ioeventfd) {
        auto cost_us = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - start_time).count();
        if (cost_us >= 10000) {
          MV_LOG("%s SLOW IO %s port=0x%x size=%u data=%lx cost=%.3lfms", device->name(),
            is_write ? "out" : "in", port, size, *(uint64_t*)data, double(cost_us) / 1000.0);
        }
        ++io_accounting_.total_pio;
        if (start_time - io_accounting_.last_print_time > std::chrono::seconds(1)) {
          MV_LOG("pio count=%u  mmio count=%u", io_accounting_.total_pio, io_accounting_.total_mmio);
          io_accounting_.last_print_time = start_time;
          io_accounting_.total_pio = 0;
          io_accounting_.total_mmio = 0;
        }
        // MV_LOG("%s handle io %s port: 0x%x size: %x data: %016lx count: %d", device->name(),
        //   is_write ? "out" : "in", port, size, *(uint64_t*)data, count);
      }
      return;
    }
  }

  /* Accessing invalid port always returns error */
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
void DeviceManager::HandleMmio(uint64_t addr, uint8_t* data, uint16_t size, int is_write, bool ioeventfd) {
  std::deque<IoHandler*>::iterator it;
  int it_count = 0;

  std::unique_lock<std::recursive_mutex> lock(mutex_);
  for (it = mmio_handlers_.begin(); it != mmio_handlers_.end(); it++, it_count++) {
    auto resource = (*it)->resource;
    if (addr >= resource->base && addr < resource->base + resource->length) {
      Device* device = (*it)->device;

      if (it_count >= 3) {
        // Move to the front for faster access next time
        mmio_handlers_.push_front(*it);
        mmio_handlers_.erase(it);
        --it;
      }
      lock.unlock();

      auto start_time = std::chrono::steady_clock::now();
      std::lock_guard<std::recursive_mutex> device_lock(device->mutex_);
      if (is_write) {
        device->Write(resource, addr - resource->base, data, size);
      } else {
        device->Read(resource, addr - resource->base, data, size);
      }

      if (machine_->debug() && !ioeventfd) {
        auto cost_us = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - start_time).count();
        if (cost_us >= 10000) {
          MV_LOG("%s SLOW MMIO %s addr=0x%lx size=%u data=%lx cost=%.3lfms", device->name(),
            is_write ? "out" : "in", addr, size, *(uint64_t*)data, double(cost_us) / 1000.0);
        }
        ++io_accounting_.total_mmio;
        if (start_time - io_accounting_.last_print_time > std::chrono::seconds(1)) {
          MV_LOG("pio count=%u  mmio count=%u", io_accounting_.total_pio, io_accounting_.total_mmio);
          io_accounting_.last_print_time = start_time;
          io_accounting_.total_pio = 0;
          io_accounting_.total_mmio = 0;
        }
        // MV_LOG("%s handled mmio %s addr: 0x%016lx size: %x data: %016lx", device->name(),
        //   is_write ? "write" : "read", addr, size, *(uint64_t*)data);
      }
      return;
    }
  }

  // Read from unhandled MMIO always returns 0
  if (!is_write) {
    bzero(data, size);
  }
  if (machine_->debug()) {
    MV_ERROR("unhandled mmio %s base: 0x%016lx size: %x data: %016lx",
      is_write ? "write" : "read", addr, size, *(uint64_t*)data);
  }
}

/* 
  Get the host memory address of a guest physical address
  Param 'size' means the length of guest memory you may need to modify, it's important for migration
*/
void* DeviceManager::TranslateGuestMemory(uint64_t gpa) {
  auto memory_manger = machine_->memory_manager();
  return memory_manger->GuestToHostAddress(gpa);
}

void DeviceManager::AddDirtyMemory(uint64_t gpa, size_t size) {
  auto memory_manger = machine_->memory_manager();
  if (size == 0) {
    memory_manger->SetDirtyMemoryRegion(gpa, PAGE_SIZE);
  } else {
    memory_manger->SetDirtyMemoryRegion(gpa, size);
  }
}

/* Sets the level of a GSI input to the interrupt controller model */
void DeviceManager::SetGsiLevel(uint gsi, uint level) {
  struct kvm_irq_level irq_level = {
    .irq = gsi,
    .level = level
  };
  if (ioctl(machine_->vm_fd_, KVM_IRQ_LINE, &irq_level) != 0) {
    MV_PANIC("KVM_IRQ_LINE failed, gsi=%u, level=%u", gsi, level);
  }
}

void DeviceManager::SetPciIrqNotifier(PciDevice* pci, int trigger_fd, int unmask_fd, bool assign) {
  auto gsi = pci->pci_header_.irq_line;
  if (pci_irq_translator_) {
    gsi = pci_irq_translator_(pci->slot_, pci->function_, pci->pci_header_.irq_pin);
  }
  MV_ASSERT(gsi < 32);

  kvm_irqfd irqfd = {
    .fd = (uint)trigger_fd,
    .gsi = (uint)gsi,
    .flags = (uint)(assign ? 0 : KVM_IRQFD_FLAG_DEASSIGN)
  };
  if (unmask_fd != -1) {
    irqfd.flags |= KVM_IRQFD_FLAG_RESAMPLE;
    irqfd.resamplefd = unmask_fd;
  }

  if (machine_->debug_) {
    MV_LOG("%s %s trigger fd=%d resamplefd=%d gsi=%d", pci->name(),
      assign ? "assign" : "deassign", trigger_fd, unmask_fd, gsi);
  }
  if (ioctl(machine_->vm_fd_, KVM_IRQFD, &irqfd) < 0) {
    MV_PANIC("%s failed to assign irqfd=%d to gsi=%d", pci->name(), trigger_fd, gsi);
  }
}

/* Calculate the GSI of a PCI pin and set its level.
 * GSI can be shared, so we count the level */
void DeviceManager::SetPciIrqLevel(PciDevice* pci, uint level) {
  auto gsi = pci->pci_header_.irq_line;
  if (pci_irq_translator_) {
    gsi = pci_irq_translator_(pci->slot_, pci->function_, pci->pci_header_.irq_pin);
  }
  MV_ASSERT(gsi < 32);

  std::unique_lock<std::recursive_mutex> lock(mutex_);
  if (level) {
    pci_irq_raised_[gsi].insert(pci->id_);
  } else {
    pci_irq_raised_[gsi].erase(pci->id_);
  }
  uint set_level = !pci_irq_raised_[gsi].empty();
  lock.unlock();

  SetGsiLevel(gsi, set_level);
}

/* It seems we can signal MSI without seting up routing table */
void DeviceManager::SignalMsi(uint64_t address, uint32_t data) {
  struct kvm_msi msi = {
    .address_lo = (uint32_t)(address),
    .address_hi = (uint32_t)(address >> 32),
    .data = data,
    .flags = 0
  };
  auto ret = ioctl(machine_->vm_fd_, KVM_SIGNAL_MSI, &msi);
  if (ret < 0) {
    MV_PANIC("KVM_SIGNAL_MSI ret=%d", ret);
  }
}

/* Since we cannot read routing table from KVM, we keep a copy and update to KVM if changed */
void DeviceManager::UpdateGsiRoutingTable() {
  uint8_t buffer[sizeof(kvm_irq_routing) + sizeof(kvm_irq_routing_entry) * gsi_routing_table_.size()];
  auto table = (kvm_irq_routing*)buffer;

  mutex_.lock();
  table->nr = gsi_routing_table_.size();
  table->flags = 0;
  std::copy(gsi_routing_table_.begin(), gsi_routing_table_.end(), table->entries);
  mutex_.unlock();

  auto ret = ioctl(machine_->vm_fd_, KVM_SET_GSI_ROUTING, table);
  if (ret) {
    MV_PANIC("KVM_SET_GSI_ROUTING ret=%d", ret);
  }
}

/* Use KVM Irq Chip */
void DeviceManager::SetupIrqChip() {
  // Use Kvm in-kernel IRQChip
  if (ioctl(machine_->vm_fd_, KVM_CREATE_IRQCHIP) < 0) {
    MV_PANIC("failed to create irqchip");
  }

  // Use Kvm in-kernel PITClock
  struct kvm_pit_config pit_config = { .flags = KVM_PIT_SPEAKER_DUMMY, .pad = {0} };
  if (ioctl(machine_->vm_fd_, KVM_CREATE_PIT2, &pit_config) < 0) {
    MV_PANIC("failed to create pit2");
  }
}

/* Although KVM has initialized GSI routing table, we still need to do it again */
void DeviceManager::ResetGsiRoutingTable() {
  gsi_routing_table_.clear();
  auto add_irq_routing = [this](uint gsi, uint chip, uint pin) {
    kvm_irq_routing_entry entry = {
      .gsi = gsi,
      .type = KVM_IRQ_ROUTING_IRQCHIP,
      .flags = 0,
      .pad = 0,
      .u = { .irqchip = { .irqchip = chip, .pin = pin } }
    };
    gsi_routing_table_.push_back(entry);
  };

  /* 8259A Master */
  for (uint i = 0; i < 8; i++) {
    if (i != 2) {
      add_irq_routing(i, 0, i);
    }
  }

  /* 8259A Slave */
  for (uint i = 0; i < 8; i++) {
    add_irq_routing(8 + i, 1, i);
  }

  /* IOAPIC */
  for (uint i = 0; i < 24; i++) {
    if (i == 0) {
      add_irq_routing(i, 2, 2);
    } else if (i != 2) {
      add_irq_routing(i, 2, i);
    }
  }

  next_gsi_ = 24;
  UpdateGsiRoutingTable();
}

/* This GSI is currently used with IRQ fd */
int DeviceManager::AddMsiNotifier(uint64_t address, uint32_t data, int trigger_fd) {
  auto gsi = next_gsi_++;

  kvm_irq_routing_entry entry = {
    .gsi = (uint)gsi,
    .type = KVM_IRQ_ROUTING_MSI,
    .flags = 0,
    .pad = 0,
    .u = { .msi = {
      .address_lo = (uint32_t)address,
      .address_hi = (uint32_t)(address >> 32),
      .data = data,
      .pad = 0
    } }
  };

  mutex_.lock();
  gsi_routing_table_.push_back(entry);
  mutex_.unlock();

  UpdateGsiRoutingTable();
  if (trigger_fd != -1) {
    kvm_irqfd irqfd = {
      .fd = (uint)trigger_fd,
      .gsi = (uint)gsi
    };
    if (ioctl(machine_->vm_fd_, KVM_IRQFD, &irqfd) < 0) {
      MV_PANIC("failed to assign irqfd=%d to gsi=%d", trigger_fd, gsi);
    }
  }

  if (machine_->debug_) {
    MV_LOG("msi route gsi=%d address=0x%lx data=0x%x", gsi, address, data);
  }
  return gsi;
}

/* Setting the address to 0 to remove a MSI route */
void DeviceManager::RemoveMsiNotifier(int gsi, int trigger_fd) {
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  auto it = std::find_if(gsi_routing_table_.begin(), gsi_routing_table_.end(), [gsi](auto &entry) {
    return entry.gsi == (uint)gsi;
  });

  if (it == gsi_routing_table_.end()) {
    MV_PANIC("not found gsi=%d", gsi);
  }

  kvm_irqfd irqfd = {
    .fd = (uint)trigger_fd,
    .gsi = (uint)gsi,
    .flags = KVM_IRQFD_FLAG_DEASSIGN
  };

  /* deassign the irqfd and remove from table */
  if (trigger_fd != -1) {
    irqfd.flags = KVM_IRQFD_FLAG_DEASSIGN;
    if (ioctl(machine_->vm_fd_, KVM_IRQFD, &irqfd) < 0) {
      MV_PANIC("failed to assign irqfd=%d to gsi=%d", trigger_fd, gsi);
    }
  }
  gsi_routing_table_.erase(it);
  lock.unlock();
  
  UpdateGsiRoutingTable();
}

bool DeviceManager::SaveState(MigrationWriter* writer) {
  /* Save states of devices */
  for (auto device : registered_devices_) {
    writer->SetPrefix(device->name());
    if (!device->SaveState(writer)) {
      MV_ERROR("failed to save state of device %s", device->name());
      return false;
    }
  }
  return true;
}

bool DeviceManager::LoadState(MigrationReader* reader) {
  ResetGsiRoutingTable();

  /* Load device states */
  for (auto device : registered_devices_) {
    reader->SetPrefix(device->name());
    if (!device->LoadState(reader)) {
      MV_PANIC("failed to load state of device %s", device->name());
      return false;
    }
  }
  return true;
}
