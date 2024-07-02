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

#include "pci_device.h"
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include "logger.h"
#include "machine.h"
#include "pci_device.pb.h"


PciDevice::PciDevice() {
  /* A pci device should be attached to pci bus */
  set_default_parent_class("Q35Host", "I440fxHost");

  bus_ = 0;
  slot_ = 0xFF;
  function_ = 0;
  bzero(&pci_header_, sizeof(pci_header_));
  bzero(&pci_bars_, sizeof(pci_bars_));
  bzero(&pci_rom_, sizeof(pci_rom_));
  bzero(&msi_config_, sizeof(msi_config_));

  next_capability_offset_ = 0x40;
  is_pcie_ = false;
}

PciDevice::~PciDevice() {
  if (pci_rom_.data) {
    free(pci_rom_.data);
  }
}

void PciDevice::Connect() {
  Device::Connect();

  // Save default command and status
  default_pci_header_ = pci_header_;
}

void PciDevice::Disconnect() {
  for (int i = 0; i < PCI_BAR_NUMS; i++) {
    if (pci_bars_[i].active) {
      DeactivatePciBar(i);
    }
  }
  Device::Disconnect();
}

void PciDevice::Reset() {
  Device::Reset();

  // TODO: Restore default command ???
}

/* Some PCI device has ROM file, should we reset ROM data if system reset ??? */
void PciDevice::LoadRomFile(const char* path) {
  /* Load rom file from path */
  if (pci_rom_.data) {
    free(pci_rom_.data);
  }

  auto config = manager_->machine()->configuration();
  int fd = open(config->FindPath(path).c_str(), O_RDONLY);
  MV_ASSERT(fd >= 0);
  struct stat st;
  fstat(fd, &st);
  pci_rom_.size = st.st_size;

  /* 64KB alignment */
  size_t align = 0x10000;
  if (st.st_size % align) {
    pci_rom_.size = (st.st_size / align + 1) * align;
  }
  pci_rom_.data = valloc(pci_rom_.size);
  bzero(pci_rom_.data, pci_rom_.size);

  read(fd, pci_rom_.data, st.st_size);
  safe_close(&fd);
}

uint8_t* PciDevice::AddCapability(uint8_t cap, const uint8_t* data, uint8_t length) {
  PciCapabilityHeader cap_header = { .type = cap, .next = pci_header_.capability };
  uint8_t* ptr = pci_header_.data + next_capability_offset_;
  memcpy(ptr, &cap_header, sizeof(cap_header));
  memcpy(ptr + 2, data, length);
  pci_header_.capability = next_capability_offset_;

  next_capability_offset_ += 2 + length;
  pci_header_.status |= PCI_STATUS_CAP_LIST; /* Has capability */
  return ptr;
}

void PciDevice::AddMsiCapability() {
  MsiCapability64 cap64;
  bzero(&cap64, sizeof(cap64));
  cap64.control = PCI_MSI_FLAGS_64BIT;

  msi_config_.is_64bit = true;
  msi_config_.offset = next_capability_offset_;
  msi_config_.length = sizeof(cap64);
  msi_config_.msi64 = (MsiCapability64*)AddCapability(PCI_CAP_ID_MSI,
    (uint8_t*)&cap64.control, msi_config_.length - 2);
}

void PciDevice::AddMsiXCapability(uint bar, uint16_t table_size, uint64_t space_offset, uint64_t space_size) {
  MV_ASSERT(table_size > 0 && table_size * sizeof(MsiXTableEntry) <= space_size);

  MsiXCapability cap;
  bzero(&cap, sizeof(cap));
  cap.control = table_size - 1;
  cap.table_offset = bar | space_offset;
  cap.pba_offset = bar | (space_offset + space_size / 2);

  msi_config_.is_msix = true;
  msi_config_.is_64bit = true;
  msi_config_.msix_bar = bar;
  msi_config_.msix_table_size = table_size;
  msi_config_.msix_space_offset = space_offset;
  msi_config_.msix_space_size = space_size;
  msi_config_.offset = next_capability_offset_;
  msi_config_.length = sizeof(cap);

  msi_config_.msix = (MsiXCapability*)AddCapability(PCI_CAP_ID_MSIX,
    (uint8_t*)&cap.control, msi_config_.length - 2);
}

void PciDevice::SignalMsi(int vector) {
  if (msi_config_.is_msix) {
    MV_ASSERT(vector < msi_config_.msix_table_size);
    auto &msix = msi_config_.msix_table[vector];
    if (msix.control & 1) {
      return; /* Masked */
    }
    uint64_t address = ((uint64_t)msix.message.address_hi << 32) | msix.message.address_lo;
    manager_->SignalMsi(address, msix.message.data);
  } else if (msi_config_.is_64bit) {
    MV_ASSERT(vector == 0);
    uint64_t address = ((uint64_t)msi_config_.msi64->address1 << 32) | msi_config_.msi64->address0;
    manager_->SignalMsi(address, msi_config_.msi64->data);
  } else {
    MV_PANIC("not implemented 32bit msi");
  }
}

void PciDevice::SetIrq(uint level) {
  MV_ASSERT(level == 0 || level == 1);
  MV_ASSERT(pci_header_.irq_pin);

  if (level) {
    pci_header_.status |= PCI_STATUS_INTERRUPT;
  } else {
    pci_header_.status &= ~PCI_STATUS_INTERRUPT;
  }

  if (pci_header_.command & PCI_COMMAND_INTX_DISABLE) {
    return;
  }

  manager_->SetPciIrqLevel(this, level);
}

void PciDevice::Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  if (msi_config_.is_msix && resource->base == pci_bars_[msi_config_.msix_bar].address &&
    offset >= msi_config_.msix_space_offset &&
    offset + size <= msi_config_.msix_space_offset + msi_config_.msix_space_size
  ) {
    offset -= msi_config_.msix_space_offset;
    MV_ASSERT(offset + size <= sizeof(MsiXTableEntry) * msi_config_.msix_table_size);
    memcpy(data, (uint8_t*)msi_config_.msix_table + offset, size);
  } else {
    Device::Read(resource, offset, data, size);
  }
}

void PciDevice::Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  if (msi_config_.is_msix && resource->base == pci_bars_[msi_config_.msix_bar].address) {
    if ( offset >= msi_config_.msix_space_offset && offset + size <= msi_config_.msix_space_offset + msi_config_.msix_space_size) {
      // set msix table
      offset -= msi_config_.msix_space_offset;
      MV_ASSERT(offset + size <= sizeof(MsiXTableEntry) * msi_config_.msix_table_size);
      memcpy((uint8_t*)msi_config_.msix_table + offset, data, size);
    } 
    return;
  } 

  Device::Write(resource, offset, data, size);
}

void PciDevice::ReadPciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
  if (offset + length > pci_config_size()) {
    bzero(data, length);
    if (debug_) {
      MV_WARN("%s failed read config space at 0x%lx length=%d", name_, offset, length);
    }
    return;
  }
  memcpy(data, pci_header_.data + offset, length);
}

void PciDevice::WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
  if (offset + length > pci_config_size()) {
    if (debug_) {
      MV_WARN("%s failed write config space at 0x%lx length=%d data=0x%x", name_, offset, length, *(uint32_t*)data);
    }
    return;
  }

  if (offset == PCI_COMMAND) {
    if (length == 4) {
      WritePciConfigSpace(offset, data, 2);
      WritePciConfigSpace(offset + 2, data + 2, 2);
    } else {
      MV_ASSERT(length == 2);
      WritePciCommand(*(uint16_t*)data);
    }
    return;
  } else if (offset == PCI_STATUS) {
    MV_ASSERT(length == 2);
    pci_header_.status &= ~(*(uint16_t*)data);
    return;
  }

  uint bar_index = (offset - PCI_BAR_OFFSET(0)) / sizeof(uint32_t);
  if (bar_index < PCI_BAR_NUMS) {
    MV_ASSERT(length == 4);
    auto& bar = pci_bars_[bar_index];

    uint32_t value = *(uint32_t*)data;
    if ((value & 0xfffffff0) == 0xfffffff0) {
      value = ~(bar.size - 1);
      pci_header_.bars[bar_index] = (value & bar.address_mask) | bar.special_bits;
      return;
    }

    uint32_t new_address = value & bar.address_mask;
    if (bar.address == new_address) {
      /* address not changed, recover the original value */
      pci_header_.bars[bar_index] = new_address | bar.special_bits;
      return;
    }

    /* A new address is set, update the bar mapping */
    UpdatePciBarAddress(bar_index, new_address);
    return;
  } else if (bar_index == 8) { /* ROM BAR */
    MV_ASSERT(length == 4);
    uint32_t value = *(uint32_t*)data;
    if ((value & 0xfffff800) == 0xfffff800) {
      uint32_t mask = value;
      pci_header_.rom_bar = ~(pci_rom_.size - 1) & mask;
      return;
    }
    
    UpdateRomBarAddress(value & 0xfffff800);
    return;
  }

  memcpy(pci_header_.data + offset, data, length);

  if (msi_config_.length) {
    /* Toggle MSI/MSI-X control */
    if (ranges_overlap(offset, length, msi_config_.offset + PCI_MSI_FLAGS, 1)) {
      if (msi_config_.is_msix) {
        msi_config_.enabled = msi_config_.msix->control & PCI_MSIX_FLAGS_ENABLE;
      } else if (msi_config_.is_64bit) {
        msi_config_.enabled = msi_config_.msi64->control & PCI_MSI_FLAGS_ENABLE;
      } else {
        msi_config_.enabled = msi_config_.msi32->control & PCI_MSI_FLAGS_ENABLE;
      }
    }
  }
}

void PciDevice::UpdatePciBarAddress(uint index, uint32_t address) {
  pci_header_.bars[index] = address | pci_bars_[index].special_bits;
  bool is64_high = pci_bars_[index].address_mask == 0xFFFFFFFF;

  uint op_index = is64_high ? index - 1 : index;
  
  DeactivatePciBar(op_index);
  
  if (is64_high) {
    pci_bars_[op_index].address64 = pci_header_.bars[op_index + 1] & PCI_BASE_ADDRESS_MEM_MASK;
    pci_bars_[op_index].address64 <<= 32;
    pci_bars_[op_index].address64 |= pci_header_.bars[op_index] & PCI_BASE_ADDRESS_MEM_MASK;
  } else {
    pci_bars_[op_index].address64 = address;
    pci_bars_[op_index].address = address;
  }

  if (pci_bars_[op_index].type == kIoResourceTypePio && !(pci_header_.command & PCI_COMMAND_IO)) {
    return;
  }
  if (pci_bars_[op_index].type != kIoResourceTypePio && !(pci_header_.command & PCI_COMMAND_MEMORY)) {
    return;
  }
  ActivatePciBar(op_index);
}

/* Guest rewrite the ROM address, remmap to new address */
void PciDevice::UpdateRomBarAddress(uint32_t address) {
  pci_header_.rom_bar = address;

  auto mm = manager_->machine()->memory_manager();
  if (pci_rom_.mapped_region) {
    if (pci_rom_.mapped_region->gpa == address) {
      /* unchanged */
      return;
    }
    mm->Unmap(&pci_rom_.mapped_region);
  }

  if (address) {
    pci_rom_.mapped_region = mm->Map(address, pci_rom_.size, pci_rom_.data, kMemoryTypeRom, "PCI ROM");
  }
}

/* Handle IO, MMIO ON or OFF */
void PciDevice::WritePciCommand(uint16_t new_command) {
  uint diff = pci_header_.command ^ new_command;
  uint toggle_io = diff & PCI_COMMAND_IO;
  uint toggle_mem = diff & PCI_COMMAND_MEMORY;
  uint toggle_intx = diff & PCI_COMMAND_INTX_DISABLE;
  pci_header_.command = new_command;

  for (int i = 0; i < PCI_BAR_NUMS; i++) {
    if (!pci_bars_[i].type || !pci_header_.bars[i])
      continue;

    bool bar_is_io = pci_header_.bars[i] & PCI_BASE_ADDRESS_SPACE_IO;
    if (toggle_io && bar_is_io) {
      if (new_command & PCI_COMMAND_IO)
        ActivatePciBar(i);
      else
        DeactivatePciBar(i);
    }

    if (toggle_mem && !bar_is_io) {
      if (new_command & PCI_COMMAND_MEMORY)
        ActivatePciBar(i);
      else
        DeactivatePciBar(i);
    }
  }

  /* If interrupt status is set, something has to be done */
  if (toggle_intx && (pci_header_.status & PCI_STATUS_INTERRUPT)) {
    bool disabled = new_command & PCI_COMMAND_INTX_DISABLE;
    manager_->SetPciIrqLevel(this, disabled ? 0 : 1);
  }
}

/* Call this function in device constructor */
void PciDevice::SetupPciBar(uint index, uint32_t size, IoResourceType type) {
  auto &bar = pci_bars_[index];
  MV_ASSERT(bar.size == 0);
  bar.size = size;
  bar.active = false;
  bar.type = type;
  if (type == kIoResourceTypePio) {
    bar.address_mask = (uint32_t)PCI_BASE_ADDRESS_IO_MASK;
    bar.special_bits = PCI_BASE_ADDRESS_SPACE_IO;
  } else if (type == kIoResourceTypeMmio) {
    bar.address_mask = (uint32_t)PCI_BASE_ADDRESS_MEM_MASK;
    bar.special_bits = 0;
  } else if (type == kIoResourceTypeRam) {
    bar.address_mask = (uint32_t)PCI_BASE_ADDRESS_MEM_MASK;
    bar.special_bits = PCI_BASE_ADDRESS_MEM_PREFETCH;
  }
  pci_header_.bars[index] |= bar.special_bits;
}


void PciDevice::SetupPciBar64(uint index, uint64_t size, IoResourceType type) {
  /* TODO: over 4GB memory size */
  MV_ASSERT(size <= 1ULL << 32);
  SetupPciBar(index, size, type);
  pci_bars_[index].special_bits |= PCI_BASE_ADDRESS_MEM_TYPE_64;
  pci_bars_[index + 1].address_mask = 0xFFFFFFFF;
}

/* Called when an bar is activate by guest BIOS or OS */
bool PciDevice::ActivatePciBar(uint index) {
  auto &bar = pci_bars_[index];
  if (bar.active)
    return true;
  MV_ASSERT(bar.type);

  if (bar.type == kIoResourceTypePio) {
    AddIoResource(kIoResourceTypePio, bar.address64, bar.size, "PCI BAR IO");
  } else if (bar.type == kIoResourceTypeMmio) {
    AddIoResource(kIoResourceTypeMmio, bar.address64, bar.size, "PCI BAR MMIO");
  } else if (bar.type == kIoResourceTypeRam) {
    MV_ASSERT(bar.host_memory != nullptr);
    AddIoResource(kIoResourceTypeRam, bar.address64, bar.size, "PCI BAR RAM", bar.host_memory);
  }
  bar.active = true;
  return true;
}

/* Release resources when resetting the bar address */
bool PciDevice::DeactivatePciBar(uint index) {
  auto &bar = pci_bars_[index];
  if (!bar.active)
    return true;

  if (bar.type == kIoResourceTypePio) {
    RemoveIoResource(kIoResourceTypePio, bar.address64);
  } else if (bar.type == kIoResourceTypeMmio) {
    RemoveIoResource(kIoResourceTypeMmio, bar.address64);
  } else if (bar.type == kIoResourceTypeRam) {
    RemoveIoResource(kIoResourceTypeRam, bar.address64);
  }
  bar.active = false;
  return true;
}

bool PciDevice::SaveState(MigrationWriter* writer) {
  PciDeviceState state;
  state.set_bus(bus_);
  state.set_slot(slot_);
  state.set_function(function_);
  state.set_pcie(is_pcie_);
  state.set_config_space(pci_header_.data, pci_config_size());

  for (int i = 0; i < msi_config_.msix_table_size; i++) {
    auto& msix = msi_config_.msix_table[i];
    auto entry = state.add_msix_entries();
    entry->set_address(((uint64_t)msix.message.address_hi << 32) | msix.message.address_lo);
    entry->set_data(msix.message.data);
    entry->set_control(msix.control);
  }
  writer->WriteProtobuf("PCI", state);
  return Device::SaveState(writer);
}

bool PciDevice::LoadState(MigrationReader* reader) {
  if (!Device::LoadState(reader)) {
    return false;
  }
  PciDeviceState state;
  if (!reader->ReadProtobuf("PCI", state)) {
    return false;
  }
  bus_ = state.bus();
  slot_ = state.slot();
  function_ = state.function();
  is_pcie_ = state.pcie();
  auto& config_space = state.config_space();
  memcpy(pci_header_.data, config_space.data(), pci_config_size());

  /* recover pci bar information */
  for (int i = 0; i < PCI_BAR_NUMS; i++) {
    auto &bar = pci_bars_[i];
    bar.address = pci_header_.bars[i] & bar.address_mask;
    if (bar.special_bits & PCI_BASE_ADDRESS_MEM_TYPE_64) {
      bar.address64 = pci_header_.bars[i + 1] & bar.address_mask;
      bar.address64 <<= 32;
      bar.address64 |= pci_header_.bars[i] & bar.address_mask;
    } else {
      bar.address64 = bar.address;
    }

    if (bar.type == kIoResourceTypePio) {
      if (bar.address64 && (pci_header_.command & PCI_COMMAND_IO)) {
        ActivatePciBar(i);
      }
    } else if (bar.type == kIoResourceTypeMmio || bar.type == kIoResourceTypeRam) {
      if (bar.address64 && (pci_header_.command & PCI_COMMAND_MEMORY)) {
        ActivatePciBar(i);
      }
    }
  }

  /* recover msix table */
  for (int i = 0; i < state.msix_entries_size(); i++) {
    auto& msix = msi_config_.msix_table[i];
    auto& entry = state.msix_entries(i);
    msix.message.address_hi = entry.address() >> 32;
    msix.message.address_lo = (uint32_t)entry.address();
    msix.message.data = entry.data();
    msix.control = entry.control();
  }

  /* enable msi / msix */
  if (msi_config_.length) {
    if (msi_config_.is_msix) {
      msi_config_.enabled = msi_config_.msix->control & PCI_MSIX_FLAGS_ENABLE;
    } else if (msi_config_.is_64bit) {
      msi_config_.enabled = msi_config_.msi64->control & PCI_MSI_FLAGS_ENABLE;
    } else {
      msi_config_.enabled = msi_config_.msi32->control & PCI_MSI_FLAGS_ENABLE;
    }
  }
  return true;
}
