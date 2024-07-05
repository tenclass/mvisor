/* 
 * MVisor VGA/VBE
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

/*
 * Since we want to implement QXL, don't waste time on VGA.
 * It's not recommended to use VGA mode due to performance problems.
 * Reference:
 * https://wiki.osdev.org/Bochs_VBE_Extensions
 * http://osdever.net/FreeVGA/vga/portidx.htm
 * https://wiki.osdev.org/VGA_Hardware
 */

#include "vga.h"
#include <cstring>
#include <sys/mman.h>

#include "logger.h"
#include "vbe.h"
#include "machine.h"
#include "edid.h"


Vga::Vga() {
  default_rom_path_ = "../share/vgabios-stdvga.bin";
  
  /* PCI config */
  pci_header_.vendor_id = 0x1234;
  pci_header_.device_id = 0x1111;
  pci_header_.class_code = 0x030000;
  pci_header_.header_type = PCI_HEADER_TYPE_NORMAL;
  pci_header_.subsys_vendor_id = 0x1AF4;
  pci_header_.subsys_id = 0x1100;

  /* Bar 0: 64MB VRAM */
  vram_size_ = _MB(64);
  SetupPciBar(0, vram_size_, kIoResourceTypeRam);

  /* Bar 2: MMIO BAR */
  SetupPciBar(2, 0x1000, kIoResourceTypeMmio);

  /* VGA registers */
  AddIoResource(kIoResourceTypePio, VGA_PIO_BASE, VGA_PIO_SIZE, "VGA IO");
  AddIoResource(kIoResourceTypePio, VBE_PIO_BASE, VBE_PIO_SIZE, "VBE IO");
  AddIoResource(kIoResourceTypeMmio, VGA_MEMORY_BASE, VGA_MEMORY_SIZE, "VGA MEMORY",
    nullptr, kIoResourceFlagCoalescingMmio);
}

Vga::~Vga() {
  // Make sure the device is disconnected
  MV_ASSERT(vga_render_ == nullptr);
}

void Vga::Reset() {
  PciDevice::Reset();

  delete vga_render_;
  vga_render_ = new VgaRender(this, vram_base_, vram_size_);
}


bool Vga::ActivatePciBar(uint32_t bar_index) {
  bool ret = PciDevice::ActivatePciBar(bar_index);

  if (ret && bar_index == 0) {
    auto region = pci_bars_[0].resource->mapped_region;
    if (region) {
      auto mm = manager_->machine()->memory_manager();
      mm->SetLogDirtyBitmap(region, true);
      vga_render_->SetMemoryRegion(region);
    }
  }
  return ret;
}

bool Vga::DeactivatePciBar(uint32_t bar_index) {
  bool ret = PciDevice::DeactivatePciBar(bar_index);

  if (ret && bar_index == 0 && vga_render_) {
    vga_render_->SetMemoryRegion(nullptr);
  }
  return ret;
}

bool Vga::SaveState(MigrationWriter* writer) {
  vga_render_->SaveState(writer);
  writer->WriteMemoryPages("VRAM", vram_base_, vram_size_);
  return PciDevice::SaveState(writer);
}

bool Vga::LoadState(MigrationReader* reader) {
  if (!PciDevice::LoadState(reader)) {
    return false;
  }

  if (!reader->ReadMemoryPages("VRAM", (void**)&vram_base_, vram_size_)) {
    return false;
  }

  if (!vga_render_->LoadState(reader)) {
    return false;
  }

  display_mode_ = kDisplayModeVga;
  Schedule([this]() {
    NotifyDisplayModeChange();
    vga_render_->Redraw();
  });
  return true;
}

void Vga::Connect() {
  /* Initialize rom data and rom bar size */
  if (!pci_rom_.data) {
    if (has_key("rom")) {
      std::string path = std::get<std::string>(key_values_["rom"]);
      LoadRomFile(path.c_str());
    } else {
      LoadRomFile(default_rom_path_.c_str());
    }
  }

  /* Initialize VRAM */
  if (!vram_base_) {
    if (has_key("vram_size")) {
      uint64_t size = std::get<uint64_t>(key_values_["vram_size"]);
      MV_ASSERT(size >= 16 && size <= 512);
      if (size & (size - 1)) {
        MV_PANIC("vram_size must be power of 2");
      }
      vram_size_ = size << 20;
    }
    vram_base_ = (uint8_t*)mmap(nullptr, vram_size_, PROT_READ | PROT_WRITE,
      MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
    pci_bars_[0].size = vram_size_;
    pci_bars_[0].host_memory = vram_base_;

    MV_ASSERT(madvise(vram_base_, vram_size_, MADV_DONTDUMP) == 0);
  }

  PciDevice::Connect();

  vga_render_ = new VgaRender(this, vram_base_, vram_size_);
  refresh_timer_ = AddTimer(NS_PER_SECOND / VGA_REFRESH_FREQUENCY, true,
    std::bind(&Vga::NotifyDisplayUpdate, this));
}

void Vga::Disconnect() {
  if (refresh_timer_) {
    RemoveTimer(&refresh_timer_);
  }

  delete vga_render_;
  vga_render_ = nullptr;

  if (vram_base_) {
    munmap((void*)vram_base_, vram_size_);
    vram_base_ = nullptr;
  }
  PciDevice::Disconnect();
}

void Vga::GetDisplayMode(int* w, int* h, int* bpp, int* stride) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (vga_render_) {
    vga_render_->GetDisplayMode(w, h, bpp, stride);
  }
}

void Vga::GetPalette(const uint8_t** palette, int* count, bool* dac_8bit) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (vga_render_) {
    vga_render_->GetPalette(palette, count, dac_8bit);
  }
}

bool Vga::GetScreenshot(DisplayUpdate& update) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (display_mode_ == kDisplayModeVga) {
    return vga_render_->GetDisplayUpdate(update);
  }
  return false;
}

void Vga::ReadMmio(uint64_t offset, uint8_t* data, uint32_t size) {
  if (offset >= 0x500 && offset + size <= 0x500 + 36) {
    MV_ASSERT(size == 2);
    uint16_t index = (offset - 0x500 ) / 2;
    vga_render_->VbeWritePort(VBE_PIO_BASE, index);
    vga_render_->VbeReadPort(VBE_PIO_BASE + 1, (uint16_t*)data);
  } else if (offset >= 0x400 && offset + size <= 0x500) {
    offset -= 0x400;
    vga_render_->VgaReadPort(VGA_PIO_BASE + offset, &data[0]);
    for (size_t i = 1; i < size; i++) {
      vga_render_->VgaReadPort(VGA_PIO_BASE + offset + 1, &data[i]);
    }
  } else if (offset + size <= 0x100) {
    memcpy(data, edid_bin + offset, size);
  } else {
    MV_PANIC("VGA MMIO read offset=0x%lx size=%d not implemented", offset, size);
  }
}

void Vga::WriteMmio(uint64_t offset, uint8_t* data, uint32_t size) {
  uint64_t value = 0;
  memcpy(&value, data, size);
  if (offset >= 0x500 && offset + size <= 0x500 + 36) {
    MV_ASSERT(size == 2);
    uint16_t index = (offset - 0x500 ) / 2;
    vga_render_->VbeWritePort(VBE_PIO_BASE, index);
    vga_render_->VbeWritePort(VBE_PIO_BASE + 1, value);
  } else if (offset >= 0x400 && offset + size <= 0x500) {
    offset -= 0x400;
    vga_render_->VgaWritePort(VGA_PIO_BASE + offset, data[0]);
    for (size_t i = 1; i < size; i++) {
      vga_render_->VgaWritePort(VGA_PIO_BASE + offset + 1, data[i]);
    }
  } else {
    MV_PANIC("VGA MMIO write offset=0x%lx size=%d data=0x%lx not implemented", offset, size, value);
  }
}

void Vga::Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  uint64_t port = resource->base + offset;
  if (resource->base == VGA_MEMORY_BASE) {
    for (size_t i = 0; i < size; i++) {
      vga_render_->VgaReadMemory(port + i, &data[i]);
    }
  } else if (resource->base == VGA_PIO_BASE) {
    vga_render_->VgaReadPort(port, &data[0]);
    for (size_t i = 1; i < size; i++) {
      vga_render_->VgaReadPort(port + 1, &data[i]);
    }
  } else if (resource->base == VBE_PIO_BASE) {
    if (size == 2) {
      vga_render_->VbeReadPort(port, (uint16_t*)data);
    }
  } else if (resource->base == pci_bars_[2].address) {
    ReadMmio(offset, data, size);
  } else {
    PciDevice::Read(resource, offset, data, size);
  }
}

void Vga::Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  uint64_t port = resource->base + offset;
  if (resource->base == VGA_MEMORY_BASE) {
    for (size_t i = 0; i < size; i++) {
      vga_render_->VgaWriteMemory(port + i, data[i]);
    }
  } else if (resource->base == VGA_PIO_BASE) {
    vga_render_->VgaWritePort(port, data[0]);
    for (size_t i = 1; i < size; i++) {
      vga_render_->VgaWritePort(port + 1, data[i]);
    }
  } else if (resource->base == VBE_PIO_BASE) {
    MV_ASSERT(size == 2);
    vga_render_->VbeWritePort(port, *(uint16_t*)data);
  } else if (resource->base == pci_bars_[2].address) {
    WriteMmio(offset, data, size);
  } else {
    PciDevice::Write(resource, offset, data, size);
  }

  if (vga_render_->IsModeChanged()) {
    display_mode_ = kDisplayModeVga;
    NotifyDisplayModeChange();
  }
}

void Vga::NotifyDisplayUpdate() {
  std::lock_guard<std::recursive_mutex> lock(display_mutex_);
  if (display_update_listeners_.empty()) {
    return;
  }

  if (display_mode_ == kDisplayModeVga) {
    DisplayUpdate update;
    vga_render_->GetDisplayUpdate(update);

    for (auto &listener : display_update_listeners_) {
      listener(update);
    }
  }
}

void Vga::Refresh() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (display_mode_ == kDisplayModeVga) {
    vga_render_->Redraw();
    NotifyDisplayUpdate();
  }
}

DECLARE_DEVICE(Vga);
