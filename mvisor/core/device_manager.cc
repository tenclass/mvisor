#include "device_manager.h"
#include <cstring>
#include <algorithm>
#include "logger.h"
#include "memory_manager.h"
#include "machine.h"
#include "devices/cmos.h"
#include "devices/ps2.h"
#include "devices/debug_console.h"
#include "devices/pci_host.h"
#include "devices/firmware_config.h"
#include "devices/dummy.h"
#include "devices/serial_port.h"
#include "devices/floppy.h"
#include "devices/ich9_lpc.h"
#include "devices/isa_dma.h"
#include "devices/vga.h"
#include "devices/ide/ide_storage.h"
#include "devices/ide/ide_controller.h"
#include "devices/ahci/ahci_host.h"
#include "images/raw.h"

#define FLOPPY_DISK_IMAGE   "../assets/msdos710.img"
#define CDROM_IMAGE         "../assets/dostools.iso"

DeviceManager::DeviceManager(Machine* machine) : machine_(machine) {
}

DeviceManager::~DeviceManager() {
  for (auto device: registered_devices_) {
    delete device;
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

void DeviceManager::IntializeQ35() {
  auto lpc = new Ich9LpcDevice();
  lpc->AddChild(new DebugConsoleDevice());
  lpc->AddChild(new CmosDevice());
  lpc->AddChild(new Ps2ControllerDevice());
  lpc->AddChild(new DummyDevice());
  lpc->AddChild(new SerialPortDevice());
  lpc->AddChild(new IsaDmaDevice());
  // lpc->AddChild(new FloppyStorageDevice(new RawDiskImage(FLOPPY_DISK_IMAGE)));

  auto pci_host = new PciHostDevice();
  pci_host->AddChild(lpc);
  pci_host->AddChild(new VgaDevice());

  if (true) {
    auto ide_controller = new IdeControllerDevice();
    auto cd = new IdeCdromStorageDevice(new RawDiskImage(CDROM_IMAGE, true));
    ide_controller->AddChild(cd);
    pci_host->AddChild(ide_controller);
  } else {
    auto ahci_host = new AhciHostDevice();
    auto cd = new IdeCdromStorageDevice(new RawDiskImage(CDROM_IMAGE, true));
    ahci_host->AddChild(cd);
    pci_host->AddChild(ahci_host);
  }

  root_ = new SystemRootDevice(this);
  root_->AddChild(new FirmwareConfigDevice());
  root_->AddChild(pci_host);

  /* Call Connect() on all devices */
  root_->Connect();
}

void DeviceManager::PrintDevices() {
  for (auto device : registered_devices_) {
    for (auto &io_resource : device->io_resources()) {
      if (io_resource.type == kIoResourceTypePio) {
        MV_LOG("\tio port 0x%lx-0x%lx", io_resource.base, io_resource.base + io_resource.length - 1);
      } else {
        MV_LOG("\tmmio address 0x%016lx-0x016%lx", io_resource.base, io_resource.base + io_resource.length);
      }
    }
  }
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
      MV_PANIC("devfn_ %x conflicts", pci_device->devfn());
      return;
    }
  }

  registered_devices_.insert(device);
}

void DeviceManager::UnregisterDevice(Device* device) {
  registered_devices_.erase(device);
}

void DeviceManager::RegisterIoHandler(Device* device, const IoResource& io_resource) {
  if (io_resource.type == kIoResourceTypePio) {
    pio_handlers_.push_back(new IoHandler {
      .io_resource = io_resource,
      .device = device
    });
  } else {
    // Map the memory to type Device, access these regions will cause MMIO access fault
    const MemoryRegion* region = machine_->memory_manager()->Map(io_resource.base, io_resource.length,
      nullptr, kMemoryTypeDevice, io_resource.name);

    mmio_handlers_.push_back(new IoHandler {
      .io_resource = io_resource,
      .device = device,
      .memory_region = region
    });
  }
  MV_LOG("%s register %s 0x%08lx-0x%08lx", device->name().c_str(),
    io_resource.type == kIoResourceTypeMmio ? "mmio" : "pio",
    io_resource.base, io_resource.base + io_resource.length - 1);
}

void DeviceManager::UnregisterIoHandler(Device* device, const IoResource& io_resource) {
  if (io_resource.type == kIoResourceTypePio) {
    for (auto it = pio_handlers_.begin(); it != pio_handlers_.end(); it++) {
      if ((*it)->device == device && (*it)->io_resource.base == io_resource.base) {
        delete *it;
        pio_handlers_.erase(it);
        break;
      }
    }
  } else {
    for (auto it = mmio_handlers_.begin(); it != mmio_handlers_.end(); it++) {
      if ((*it)->device == device && (*it)->io_resource.base == io_resource.base) {
        delete *it;
        mmio_handlers_.erase(it);
        break;
      }
    }
  }
}

void DeviceManager::HandleIo(uint16_t port, uint8_t* data, uint16_t size, int is_write, uint32_t count) {
  int found = 0, it_count = 0;
  std::deque<IoHandler*>::iterator it;
  for (it = pio_handlers_.begin(); it != pio_handlers_.end(); it++, it_count++) {
    auto &resource = (*it)->io_resource;
    if (port >= resource.base && port < resource.base + resource.length) {
      Device* device = (*it)->device;
      uint8_t* ptr = data;
      for (uint32_t i = 0; i < count; i++) {
        if (is_write) {
          device->Write(resource, port - resource.base, ptr, size);
        } else {
          device->Read(resource, port - resource.base, ptr, size);
        }
        ptr += size;
      }
      ++found;
      if (it_count > 2) {
        pio_handlers_.push_front(*it);
        pio_handlers_.erase(it);
        --it;
      }
    }
  }

  if (!found) {
    MV_LOG("unhandled io %s port: 0x%x size: %x data: %016lx count: %d",
      is_write ? "out" : "in", port, size, *(uint64_t*)data, count);
    /* Accessing invalid port always returns error */
    memset(data, 0xFF, size);
  }
}

void DeviceManager::HandleMmio(uint64_t base, uint8_t* data, uint16_t size, int is_write) {
  std::deque<IoHandler*>::iterator it;
  int it_count = 0;
  for (it = mmio_handlers_.begin(); it != mmio_handlers_.end(); it++, it_count++) {
    auto &resource = (*it)->io_resource;
    if (base >= resource.base && base < resource.base + resource.length) {
      Device* device = (*it)->device;
      if (is_write) {
        device->Write(resource, base - resource.base, data, size);
      } else {
        device->Read(resource, base - resource.base, data, size);
      }
      data += size;
      if (it_count > 2) {
        mmio_handlers_.push_front(*it);
        mmio_handlers_.erase(it);
        --it;
      }
      return;
    }
  }
  MV_PANIC("unhandled mmio %s base: 0x%016lx size: %x data: %016lx",
    is_write ? "write" : "read", base, size, *(uint64_t*)data);
}

void* DeviceManager::TranslateGuestMemory(uint64_t gpa) {
  auto memory_manger = machine_->memory_manager();
  void* host = memory_manger->GuestToHostAddress(gpa);
  return host;
}

void DeviceManager::ReadGuestMemory(uint64_t gpa, uint8_t* data, uint32_t size) {
  void* host = TranslateGuestMemory(gpa);
  if (host) {
    memcpy(data, host, size);
  }
}

void DeviceManager::WriteGuestMemory(uint64_t gpa, uint8_t* data, uint32_t size) {
  auto memory_manger = machine_->memory_manager();
  void* host = memory_manger->GuestToHostAddress(gpa);
  if (host) {
    memcpy(host, data, size);
  }
}

void DeviceManager::SetIrq(uint32_t irq, uint32_t level) {
  machine_->Interrupt(irq, level);
}
