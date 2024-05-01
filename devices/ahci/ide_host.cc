/* 
 * MVisor - IDE Host Controller
 * Copyright (C) 2022 Terrence <terrence@tenclass.com>
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

#include "ide_host.h"

#include <sys/ioctl.h>
#include <cstring>

#include "logger.h"
#include "device_manager.h"
#include "device_interface.h"
#include "ata_storage.h"
#include "ata_disk.h"

/* Reference:
 * https://wiki.osdev.org/PCI_IDE_Controller
 */

IdeHost::IdeHost() {
  /* IDE controller, primary and secondary channel in PCI native mode */
  pci_header_.class_code = 0x010180;
  pci_header_.header_type = PCI_HEADER_TYPE_NORMAL;
  pci_header_.subsys_vendor_id = 0x1AF4;
  pci_header_.subsys_id = 0x1100;
  pci_header_.status |= PCI_STATUS_FAST_BACK;

  /* IDE Bus master */
  SetupPciBar(4, 16, kIoResourceTypePio);

  /* Legacy IDE ports */
  AddIoResource(kIoResourceTypePio, 0x01F0, 8, "IDE Primary");
  AddIoResource(kIoResourceTypePio, 0x0170, 8, "IDE Secondary");
  AddIoResource(kIoResourceTypePio, 0x03F6, 1, "IDE Primary Control");
  AddIoResource(kIoResourceTypePio, 0x0376, 1, "IDE Secondary Control");
}

IdeHost::~IdeHost() {
}

void IdeHost::Connect() {
  PciDevice::Connect();

  for (size_t i = 0; i < ports_.size(); i++) {
    ports_[i] = new IdePort(manager_, this, i);
  }

  /* Add storage devices.
   * Old OS like Win98 requires the CDRom attached to the slave drive */
  int disk_cables[4][2] = {{ 0, 0 }, { 1, 0 }, { 0, 1 }, { 1, 1 }};
  int cdrom_cables[4][2] = {{ 0, 1 }, { 1, 1 }, { 0, 0 }, { 1, 0 }};
  for (size_t i = 0; i < children_.size(); i++) {
    auto device = dynamic_cast<AtaStorageDevice*>(children_[i]);
    auto cables = device->type() == kAtaStorageTypeCdrom ? cdrom_cables : disk_cables;
    for (size_t j = 0; j < 4; j++) {
      if (ports_[cables[j][0]]->drives()[cables[j][1]] == nullptr) {
        ports_[cables[j][0]]->AttachDevice(device, cables[j][1]);
        break;
      }
    }
  }
}

void IdeHost::Disconnect() {
  for (size_t i = 0; i < ports_.size(); i++) {
    if (ports_[i]) {
      delete ports_[i];
      ports_[i] = nullptr;
    }
  }
  PciDevice::Disconnect();
}

bool IdeHost::SaveState(MigrationWriter* writer) {
  IdeHostState state;
  for (size_t i = 0; i < ports_.size(); i++) {
    auto port_state = state.add_ports();
    ports_[i]->SaveState(port_state);
  }
  writer->WriteProtobuf("IDE_HOST", state);
  return PciDevice::SaveState(writer);
}

bool IdeHost::LoadState(MigrationReader* reader) {
  if (!PciDevice::LoadState(reader)) {
    return false;
  }

  IdeHostState state;
  if (!reader->ReadProtobuf("IDE_HOST", state)) {
    return false;
  }
  for (size_t i = 0; i < ports_.size(); i++) {
    auto &port_state = state.ports(i);
    ports_[i]->LoadState(&port_state);
  }
  return true;
}

void SetCmosDiskInformation(CmosDataInterface* cmos, int type_index, int info_index, AtaStorageDevice* drive) {
  auto disk = dynamic_cast<AtaDisk*>(drive);
  auto &info = disk->geometry();
  cmos->SetData(type_index, 47);
  cmos->SetData(info_index, info.cylinders_per_heads);
  cmos->SetData(info_index + 1, info.cylinders_per_heads >> 8);
  cmos->SetData(info_index + 2, info.heads);
  cmos->SetData(info_index + 3, 0xFF);
  cmos->SetData(info_index + 4, 0xFF);
  cmos->SetData(info_index + 5, 0xC0 | ((info.heads > 8) << 3));
  cmos->SetData(info_index + 6, info.cylinders_per_heads);
  cmos->SetData(info_index + 7, info.cylinders_per_heads >> 8);
  cmos->SetData(info_index + 8, info.sectors_per_cylinder);
}

void IdeHost::Reset() {
  PciDevice::Reset();

  for (size_t i = 0; i < ports_.size(); i++) {
    ports_[i]->Reset();
  }

  /* IDE disk information */
  auto cmos = dynamic_cast<CmosDataInterface*>(manager_->LookupDeviceByClass("Cmos"));
  if (cmos) {
    /* hard disk type and size */
    uint8_t value = 0;
    auto primary_drives = ports_[0]->drives();
    if (primary_drives[0] && primary_drives[0]->type() == kAtaStorageTypeDisk) {
      SetCmosDiskInformation(cmos, 0x19, 0x1B, primary_drives[0]);
      value |= 0xF0;
    }
    if (primary_drives[1] && primary_drives[1]->type() == kAtaStorageTypeDisk) {
      SetCmosDiskInformation(cmos, 0x1A, 0x24, primary_drives[1]);
      value |= 0x0F;
    }
    cmos->SetData(0x12, value);

    /* set ATA translation policy (XP needs this to boot) */
    value = 0;
    for (size_t i = 0; i < 4; i++) {
      auto drive = ports_[i / 2]->drives()[i % 2];
      if (drive && drive->type() == kAtaStorageTypeDisk) {
        value += 1 << (i * 2);
      }
    }
    cmos->SetData(0x39, value);
  }
}

void IdeHost::SetPortIrq(int index, uint level) {
  if (index == 0) {
    manager_->SetGsiLevel(14, level);
  } else if (index == 1) {
    manager_->SetGsiLevel(15, level);
  }
}

void IdeHost::Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  if (resource->base == 0x1F0) {
    ports_[0]->Read(offset, data, size);
  } else if (resource->base == 0x3F6) {
    ports_[0]->ReadControl(offset + 2, data, size);
  } else if (resource->base == 0x170) {
    ports_[1]->Read(offset, data, size);
  } else if (resource->base == 0x376) {
    ports_[1]->ReadControl(offset + 2, data, size);
  } else if (resource->base == pci_bars_[4].address) {
    uint index = offset / 8;
    ports_[index]->ReadBusMaster(offset % 8, data, size);
  } else {
    PciDevice::Read(resource, offset, data, size);
  }
}

void IdeHost::Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  if (resource->base == 0x1F0) {
    ports_[0]->Write(offset, data, size);
  } else if (resource->base == 0x3F6) {
    ports_[0]->WriteControl(offset + 2, data, size);
  } else if (resource->base == 0x170) {
    ports_[1]->Write(offset, data, size);
  } else if (resource->base == 0x376) {
    ports_[1]->WriteControl(offset + 2, data, size);
  } else if (resource->base == pci_bars_[4].address) {
    uint index = offset / 8;
    ports_[index]->WriteBusMaster(offset % 8, data, size);
  } else {
    PciDevice::Write(resource, offset, data, size);
  }
}
