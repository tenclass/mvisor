/* 
 * MVisor - AHCI Host Controller
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

#include "ahci_host.h"

#include <sys/ioctl.h>
#include <cstring>

#include "logger.h"
#include "device_manager.h"
#include "ide_storage.h"
#include "ahci_internal.h"
#include "machine.h"
#include "linux/kvm.h"
#include "pb/ahci_host.pb.h"

/* Reference:
 * https://wiki.osdev.org/AHCI
 * https://www.intel.com/content/dam/www/public/us/en/documents/technical-specifications/serial-ata-ahci-spec-rev1_3.pdf
 */

AhciHost::AhciHost() {
  /* FIXME: should gernerated by parent pci device */
  slot_ = 31;
  function_ =  2;
  
  /* PCI config */
  pci_header_.vendor_id = 0x8086;
  pci_header_.device_id = 0x2922;
  pci_header_.class_code = 0x010601;
  pci_header_.revision_id = 2;
  pci_header_.header_type = PCI_MULTI_FUNCTION | PCI_HEADER_TYPE_NORMAL;
  pci_header_.subsys_vendor_id = 0x1AF4;
  pci_header_.subsys_id = 0x1100;
  pci_header_.irq_pin = 1;

  pci_header_.data[0x90] = 1 << 6; /* Address Map Register - AHCI mode */

  /* Add SATA capability */
  const uint8_t sata_cap[] = { 0x10, 0x00, 0x48, 0x00, 0x00, 0x00 };
  AddCapability(0x12, sata_cap, sizeof(sata_cap));
  /* Add MSI */
  AddMsiCapability();

  /* Memory bar */
  AddPciBar(5, 4096, kIoResourceTypeMmio);
}

AhciHost::~AhciHost() {
  for (auto port : ports_) {
    if (port) {
      delete port;
    }
  }
}

void AhciHost::Connect() {
  PciDevice::Connect();

  num_ports_ = 0;
  /* Add storage devices */
  for (size_t i = 0; i < children_.size(); i++) {
    IdeStorageDevice* device = dynamic_cast<IdeStorageDevice*>(children_[i]);
    num_ports_++;
    if (!ports_[i]) {
      ports_[i] = new AhciPort(manager_, this, i);
    }
    ports_[i]->AttachDevice(device);
  }
}

void AhciHost::Disconnect() {
  for (size_t i = 0; i < ports_.size(); i++) {
    if (ports_[i]) {
      delete ports_[i];
      ports_[i] = nullptr;
    }
  }
  PciDevice::Disconnect();
}

void AhciHost::Reset() {
  PciDevice::Reset();
  bzero(&host_control_, sizeof(host_control_));
  host_control_.global_host_control = HOST_CONTROL_AHCI_ENABLE;
  host_control_.capabilities = (num_ports_ > 0 ? num_ports_ - 1 : 0) |
    (AHCI_NUM_COMMAND_SLOTS << 8) |
    (AHCI_SUPPORTED_SPEED_GEN1 << AHCI_SUPPORTED_SPEED) |
    // HOST_CAP_NCQ |
    HOST_CAP_AHCI |
    HOST_CAP_64;
  host_control_.ports_implemented = (1 << num_ports_) - 1;
  host_control_.version = AHCI_VERSION_1_0;
  
  for (int i = 0; i < num_ports_; i++) {
    ports_[i]->Reset();
  }
}

void AhciHost::CheckIrq() {
  host_control_.irq_status = 0;
  for (int i = 0; i < num_ports_; i++) {
    auto &pc = ports_[i]->port_control_;
    if (pc.irq_status & pc.irq_mask) {
      host_control_.irq_status |= (1 << i);
    }
  }
  if (host_control_.irq_status && (host_control_.global_host_control & HOST_CONTROL_IRQ_ENABLE)) {
    if (msi_config_.enabled) {
      SignalMsi();
    } else {
      SetIrq(1);
    }
  } else {
    if (!msi_config_.enabled) {
      SetIrq(0);
    }
  }
}

void AhciHost::Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  MV_ASSERT(size == 4 && resource->type == kIoResourceTypeMmio);

  if (offset >= 0x100) {
    int port = (offset - 0x100) >> 7;
    if (ports_[port]) {
      ports_[port]->Read(offset & 0x7f, (uint32_t*)data);
    }
  } else {
    memcpy(data, (uint8_t*)&host_control_ + offset, size);
  }
}

void AhciHost::Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  MV_ASSERT(size == 4 && resource->type == kIoResourceTypeMmio);
  uint32_t value = *(uint32_t*)data;
  
  if (offset >= 0x100) {
    int port = (offset - 0x100) >> 7;
    if (ports_[port]) {
      ports_[port]->Write(offset & 0x7f, value);
    }
  } else {
    switch (offset / 4)
    {
    case kAhciHostRegCapabilities:
    case kAhciHostRegPortsImplemented:
      /* Why Linux writes these registers ??? */
      break;
    case kAhciHostRegControl:
      if (value & HOST_CONTROL_RESET) {
        Reset();
      } else {
        host_control_.global_host_control = (value & 3) | HOST_CONTROL_AHCI_ENABLE;
        // Maybe irq is enabled now, so call check
        CheckIrq();
      }
      break;
    case kAhciHostRegIrqStatus:
      host_control_.irq_status &= ~value;
      break;
    default:
      MV_PANIC("not implemented %s base=0x%lx offset=0x%lx size=%d data=0x%lx",
        name_, resource->base, offset, size, value);
    }
  }
}

bool AhciHost::SaveState(MigrationWriter* writer) {
  AhciHostState state;
  auto control = state.mutable_control();
  control->set_capabilities(host_control_.capabilities);
  control->set_global_host_control(host_control_.global_host_control);
  control->set_irq_status(host_control_.irq_status);
  control->set_ports_implemented(host_control_.ports_implemented);
  control->set_version(host_control_.version);

  for (int i = 0; i < num_ports_; i++) {
    auto port_state = state.add_ports();
    ports_[i]->SaveState(port_state);
  }
  writer->WriteProtobuf("AHCI", state);
  return PciDevice::SaveState(writer);
}

bool AhciHost::LoadState(MigrationReader* reader) {
  if (!PciDevice::LoadState(reader)) {
    return false;
  }
  AhciHostState state;
  if (!reader->ReadProtobuf("AHCI", state)) {
    return false;
  }
  auto &control = state.control();
  host_control_.capabilities = control.capabilities();
  host_control_.global_host_control = control.global_host_control();
  host_control_.irq_status = control.irq_status();
  host_control_.ports_implemented = control.ports_implemented();
  host_control_.version = control.version();

  for (int i = 0; i < num_ports_; i++) {
    auto &port_state = state.ports(i);
    ports_[i]->LoadState(&port_state);
  }
  return true;
}

DECLARE_DEVICE(AhciHost);
