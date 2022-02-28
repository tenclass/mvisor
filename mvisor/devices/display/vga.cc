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
 * Since we want to implement QXL, so don't waste time on VGA.
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
#include "vga.font.inc"

#define VGA_PIO_BASE    0x3C0
#define VGA_PIO_SIZE    0x20
#define VBE_PIO_BASE    0x1CE
#define VBE_PIO_SIZE    2
#define VBE_LINEAR_FRAMEBUFFER_BASE 0xE0000000

// When LFB mode disabled, the tradition VGA video memory address is used
#define VGA_MMIO_BASE   0x000A0000
#define VGA_MMIO_SIZE   0x00020000


Vga::Vga() {
  devfn_ = PCI_MAKE_DEVFN(2, 0);
  
  /* PCI config */
  pci_header_.vendor_id = 0x1234;
  pci_header_.device_id = 0x1111;
  pci_header_.class_code = 0x030000;
  pci_header_.revision_id = 5;
  pci_header_.header_type = PCI_HEADER_TYPE_NORMAL;
  pci_header_.subsys_vendor_id = 0x1AF4;
  pci_header_.subsys_id = 0x1100;
  pci_header_.command = PCI_COMMAND_IO | PCI_COMMAND_MEMORY;
  pci_header_.irq_pin = 1;
  

  /* Bar 0: 256MB VRAM (default total) */
  vga_mem_size_ = _MB(16);
  vram_size_ = _MB(256);

  AddPciBar(0, vram_size_, kIoResourceTypeRam);    /* vgamem */
  /* FIXME: bar 2 should be implemented for stdvga if Qxl is not enabled??? */

  AddIoResource(kIoResourceTypePio, VGA_PIO_BASE, VGA_PIO_SIZE, "VGA IO");
  AddIoResource(kIoResourceTypePio, VBE_PIO_BASE, VBE_PIO_SIZE, "VBE IO");
  AddIoResource(kIoResourceTypeMmio, VGA_MMIO_BASE, VGA_MMIO_SIZE, "VGA MMIO");
}

Vga::~Vga() {
}

void Vga::Reset() {
  PciDevice::Reset();
  bzero(vbe_registers_, sizeof(vbe_registers_));
  bzero(sequence_registers_, sizeof(sequence_registers_));
  bzero(gfx_registers_, sizeof(gfx_registers_));
  bzero(pallete_, sizeof(pallete_));
  bzero(crtc_registers_, sizeof(crtc_registers_));
  bzero(attribute_registers_, sizeof(attribute_registers_));
  bzero(status_registers_, sizeof(status_registers_));
  misc_output_reg_ = 0;

  sequence_index_ = 0;
  gfx_index_ = 0;
  attribute_index_ = 0;
  pallete_read_index_ = 0;
  pallete_write_index_ = 0;
  crtc_index_ = 0;
  vbe_index_ = 0;

  vram_map_select_ = vram_base_;
  vram_read_select_ = vram_base_;
  width_ = 0;
  height_ = 0;
  bpp_ = 0;
  mode_ = kDisplayTextMode;
}

void Vga::Connect() {
  /* Initialize rom data and rom bar size */
  if (!pci_rom_.data && has_key("rom")) {
    std::string path = std::get<std::string>(key_values_["rom"]);
    LoadRomFile(path.c_str());
  }
  if (!vram_base_) {
    if (has_key("vram_size")) {
      uint64_t size = std::get<uint64_t>(key_values_["vram_size"]);
      MV_ASSERT(size >= 32 && size <= 1024);
      vram_size_ = size << 20;
    }
    vram_base_ = (uint8_t*)mmap(nullptr, vram_size_, PROT_READ | PROT_WRITE,
      MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
    pci_bars_[0].host_memory = vram_base_;
  }

  refresh_timer_ = manager_->io()->AddTimer(1000 / 30, true, std::bind(&Vga::OnRefreshTimer, this));

  PciDevice::Connect();
}

void Vga::Disconnect() {
  if (vram_base_) {
    munmap((void*)vram_base_, vram_size_);
    vram_base_ = nullptr;
  }
  manager_->io()->RemoveTimer(refresh_timer_);
  PciDevice::Disconnect();
}

void Vga::GetCursorLocation(uint8_t* x, uint8_t* y, uint8_t* sel_start, uint8_t* sel_end) {
  uint16_t location = (crtc_registers_[0xE] << 8) | (crtc_registers_[0xF]);
  *sel_start = crtc_registers_[0xA] & 0x1F;
  *sel_end = crtc_registers_[0xB] & 0x1F;
  *y = location / 80;
  *x = location % 80;
}

void Vga::GetDisplayMode(uint16_t* w, uint16_t* h, uint16_t* bpp) {
  if ((gfx_registers_[6] & 0x1) == 0) {
    *w = 640;
    *h = 400;
    *bpp = 8;
  } else {
    *w = width_;
    *h = height_;
    *bpp = bpp_;
  }
}

const uint8_t* Vga::GetPallete() const {
  return pallete_;
}

void Vga::Read(const IoResource* ir, uint64_t offset, uint8_t* data, uint32_t size) {
  uint64_t port = ir->base + offset;

  if (ir->base == VGA_MMIO_BASE) {
    memcpy(data, vram_read_select_ + offset, size);
  } else if (ir->base == VGA_PIO_BASE) {
    VgaReadPort(port, data, size);
  } else if (ir->base == VBE_PIO_BASE) {
    MV_ASSERT(size == 2);
    VbeReadPort(port, (uint16_t*)data);
  } else {
    MV_PANIC("unhandled read at base=0x%lx offset=0x%lx data=0x%lx size=%x",
      ir->base, offset, *(uint64_t*)data, size);
  }
}

void Vga::Write(const IoResource* ir, uint64_t offset, uint8_t* data, uint32_t size) {
  uint64_t port = ir->base + offset;
  if (ir->base == VGA_MMIO_BASE) {
    memcpy(vram_read_select_ + offset, data, size);
  } else if (ir->base == VGA_PIO_BASE) {
    VgaWritePort(port, data, size);
  } else if (ir->base == VBE_PIO_BASE) {
    VbeWritePort(port, *(uint16_t*)data);
  } else {
    MV_PANIC("unhandled write at base=0x%lx offset=0x%lx data=0x%lx size=%x",
      ir->base, offset, *(uint64_t*)data, size);
  }
}

void Vga::VbeReadPort(uint64_t port, uint16_t* data) {
  if (port == 0x1CE) {
    *data = vbe_index_;
    return;
  }
  if (vbe_index_ < VBE_DISPI_INDEX_NB) {
    if (vbe_registers_[VBE_DISPI_INDEX_ENABLE] & VBE_DISPI_GETCAPS) {
      /* VBE initialization will enable and get capabilities and then disable */
      const uint16_t max_values[] = {
        0, VBE_DISPI_MAX_XRES, VBE_DISPI_INDEX_YRES, VBE_DISPI_MAX_BPP
      };
      MV_ASSERT(vbe_index_ < sizeof(max_values) / sizeof(uint16_t));
      *data = max_values[vbe_index_];
    } else {
      *data = vbe_registers_[vbe_index_];
    }
  } else if (vbe_index_ == VBE_DISPI_INDEX_VIDEO_MEMORY_64K) {
    *data = vram_size_ >> 16;
  } else {
    *data = 0;
    MV_PANIC("invalid read index = %d", vbe_index_);
  }
}

void Vga::VbeWritePort(uint64_t port, uint16_t value) {
  if (port == 0x1CE) { // index
    if (value > VBE_DISPI_INDEX_NB) {
      MV_PANIC("invalid vbe index 0x%x", value);
    }
    vbe_index_ = value;
  } else if (port == 0x1CF) { // data
    switch (vbe_index_)
    {
    case VBE_DISPI_INDEX_ID:
      vbe_version_ = value;
      break;
    case VBE_DISPI_INDEX_XRES:
      width_ = value;
      break;
    case VBE_DISPI_INDEX_YRES:
      height_ = value;
      break;
    case VBE_DISPI_INDEX_BPP:
      bpp_ = value;
      break;
    case VBE_DISPI_INDEX_ENABLE:
      if (debug_) {
        MV_LOG("set vbe enable %x to %x %dx%d bpp=%d", vbe_registers_[4], value,
          vbe_registers_[1], vbe_registers_[2], vbe_registers_[3]);
      }
      if (value & 1) {
        UpdateDisplayMode();
      }
      break;
    case VBE_DISPI_INDEX_BANK:
      vram_read_select_ = vram_base_ + (value << 16);
      break;
    }
    vbe_registers_[vbe_index_] = value;
  }
}

void Vga::VgaReadPort(uint64_t port, uint8_t* data, uint32_t size) {
  switch (port)
  {
  case 0x3C0:
    MV_ASSERT(size == 1);
    *data = attribute_index_;
    break;
  case 0x3C1:
    MV_ASSERT(size == 1);
    *data = attribute_registers_[attribute_index_ & 0x7F];
    break;
  case 0x3C4:
    MV_ASSERT(size == 1);
    *data = sequence_index_;
    break;
  case 0x3C5:
    MV_ASSERT(size == 1);
    *data = sequence_registers_[sequence_index_];
    break;
  case 0x3C9:
    for (uint32_t i = 0; i < size; i++) {
      *data++ = pallete_[pallete_read_index_++];
    }
    break;
  case 0x3CC:
    MV_ASSERT(size == 1);
    *data = misc_output_reg_;
    break;
  case 0x3CE:
    MV_ASSERT(size == 1);
    *data = gfx_index_;
    break;
  case 0x3CF:
    MV_ASSERT(size == 1);
    *data = gfx_registers_[gfx_index_];
    break;
  case 0x3D5:
    MV_ASSERT(size == 1);
    *data = crtc_registers_[crtc_index_];
    break;
  case 0x3DA:
    MV_ASSERT(size == 1);
    attribute_index_ &= ~0x80; // Clears attribute flip-flop
    status_registers_[1] ^= 9;
    *data = status_registers_[1];
    break;
  default:
    MV_PANIC("not implemented %s port=0x%lx size=%d data=0x%lx",
      name_, port, size, *(uint64_t*)data);
    break;
  }
}

void Vga::VgaWritePort(uint64_t port, uint8_t* data, uint32_t size) {
  uint8_t value = *data;
  switch (port)
  {
  case 0x3C0:
    MV_ASSERT(size == 1);
    if (attribute_index_ & 0x80) { // set data
      attribute_index_ &= ~0x80;
      attribute_registers_[attribute_index_] = value;
    } else { // set index
      attribute_index_ = 0x80 | value;
      if (attribute_index_ & 0x20) {
        // renderer changed event
        UpdateDisplayMode();
      }
    }
    break;
  case 0x3C2:
    MV_ASSERT(size == 1);
    misc_output_reg_ = value & ~0x10;
    break;
  case 0x3C4:
    sequence_index_ = value;
    if (size == 2) {
      sequence_registers_[sequence_index_] = data[1];
      if (sequence_index_ == 4) {
        sequence_registers_[sequence_index_] &= 0b1110;
      }
    }
    break;
  case 0x3C5:
    MV_ASSERT(size == 1);
    sequence_registers_[sequence_index_] = value;
    break;
  case 0x3C6:
    MV_ASSERT(value == 0xFF); // pallete mask
    break;
  case 0x3C7:
    MV_ASSERT(size == 1);
    pallete_read_index_ = value * 3;
    break;
  case 0x3C8:
    MV_ASSERT(size == 1);
    pallete_write_index_ = value * 3;
    break;
  case 0x3C9:
    MV_ASSERT(size == 1);
    pallete_[pallete_write_index_++] = value;
    break;
  case 0x3CE:
    gfx_index_ = value;
    if (size == 2) {
      gfx_registers_[gfx_index_] = data[1];
      if (gfx_index_ == 4 || gfx_index_ == 6) {
        UpdateVRamMemoryMap();
      }
    }
    break;
  case 0x3CF:
    MV_ASSERT(size == 1);
    gfx_registers_[gfx_index_] = value;
    if (gfx_index_ == 4 || gfx_index_ == 6) {
      UpdateVRamMemoryMap();
    }
    break;
  case 0x3D4:
    crtc_index_ = value;
    if (size == 2) {
      crtc_registers_[crtc_index_] = data[1];
    }
    break;
  case 0x3D5:
    MV_ASSERT(size == 1);
    crtc_registers_[crtc_index_] = value;
    break;
  default:
    MV_PANIC("not implemented %s port=0x%lx size=%d data=0x%lx",
      name_, port, size, value);
    break;
  }
}

void Vga::UpdateVRamMemoryMap() {
  const int map_types[][2] = {
    { 0xA0000, 0x20000 }, { 0xA0000, 0x10000 },
    { 0xB0000, 0x08000 }, { 0xB8000, 0x08000 }
  };
  /* Memory map select */
  int index = (gfx_registers_[6] >> 2) & 0b11;
  int read_index = gfx_registers_[4] & 0b11;
  vram_map_select_size_ = map_types[index][1];
  vram_map_select_ = vram_base_ + map_types[index][0] - VGA_MMIO_BASE;
  vram_read_select_ = vram_base_ + vram_map_select_size_ * read_index;
  if (debug_) {
    MV_LOG("map index=%d read_index=%d", index, read_index);
  }
}

void Vga::UpdateDisplayMode() {
  if (vbe_registers_[VBE_DISPI_INDEX_ENABLE] & VBE_DISPI_ENABLED) {
    mode_ = kDisplayVbeMode;
  } else if ((gfx_registers_[6] & 0x1) == 0) {
    mode_ = kDisplayTextMode;
  } else {
    mode_ = kDisplayVgaMode;
  }
  NotifyDisplayModeChange();
}

void Vga::NotifyDisplayModeChange() {
  for (auto listener : display_change_listerners_) {
    listener();
  }
}

void Vga::RegisterDisplayChangeListener(DisplayChangeListener callback) {
  display_change_listerners_.push_back(callback);
}

void Vga::RegisterDisplayRenderer(DisplayRenderCallback draw_callback,
    DisplayCursorUpdateCallback cursor_callback) {
  display_render_callbacks_.push_back(draw_callback);
  display_cursor_callbacks_.push_back(cursor_callback);
}

void Vga::NotifyDisplayRender(DisplayPartialBitmap* partial) {
  for (auto renderer : display_render_callbacks_) {
    renderer(partial);
  }
}

void Vga::NotifyDisplayCursorUpdate(DisplayCursorUpdate* update) {
  for (auto callback : display_cursor_callbacks_) {
    callback(update);
  }
}

void Vga::OnRefreshTimer() {
  if (mode_ == kDisplayTextMode) {
    width_ = 640;
    height_ = 400;
    bpp_ = 8;
    RenderTextMode();
  } else if (mode_ == kDisplayVbeMode) {
    RenderGraphicsMode();
  }
}

void Vga::RenderGraphicsMode() {
  DisplayPartialBitmap* partial = new DisplayPartialBitmap {
    .width = width_,
    .height = height_,
    .x = 0,
    .y = 0
  };
  partial->stride = partial->width * (bpp_ >> 3);
  partial->vector.emplace_back(DisplayPartialData {
    .data = vram_map_select_,
    .size = size_t(partial->stride * partial->height)
  });
  partial->release = [partial]() {
    delete partial;
  };
  NotifyDisplayRender(partial);
}

void Vga::RenderTextMode() {
  DisplayPartialBitmap* partial = new DisplayPartialBitmap {
    .width = width_,
    .height = height_,
    .x = 0,
    .y = 0
  };
  partial->stride = partial->width * (bpp_ >> 3);
  uint8_t* buffer = new uint8_t[partial->stride * partial->height];
  partial->vector.emplace_back(DisplayPartialData {
    .data = buffer,
    .size = size_t(partial->stride * partial->height)
  });
  partial->release = [partial]() {
    delete[] partial->vector[0].data;
    delete partial;
  };

  uint8_t* ptr = vram_map_select_;
  for (int y = 0; y < 25; y++) {
    for (int x = 0; x < 80; x++) {
      int character = *ptr++;
      int attribute = *ptr++;
      DrawCharacter(buffer, partial->stride, x, y, character, attribute);
    }
  }
  
  DrawTextCursor(buffer, partial->stride);
  NotifyDisplayRender(partial);
}

void Vga::DrawCharacter(uint8_t* buffer, int stride, int x, int y, int character, int attribute) {
  uint8_t* font = (uint8_t*)__font8x16;

  buffer += (y * 16) * stride;
  buffer += x * 8;
  
  // Draw the glyph
  uint8_t fore_color = attribute & 0xF;
  uint8_t back_color = (attribute >> 4) & 0xF;
  for (int yy = 0; yy < 16; yy++) {
    for (int xx = 0; xx < 8; xx++) {
      if (font[character * 16 + yy] & (0x80 >> xx)) {
        buffer[xx] = fore_color;
      } else {
        buffer[xx] = back_color;
      }
    }
    buffer += stride;
  }
}

void Vga::DrawTextCursor(uint8_t* buffer, int stride) {
  uint32_t fore_color = 7;
  uint8_t cx, cy, sl_start, sl_end;
  GetCursorLocation(&cx, &cy, &sl_start, &sl_end);

  buffer += (cy * 16) * stride;
  buffer += cx * 8;

  uint8_t* end = buffer + stride * height_;

  buffer += stride * sl_start;

  for (int y = sl_start; y < sl_end; y++) {
    for (int x = 0; x < 8; x++) {
      if (buffer + x < end) {
        buffer[x] = fore_color;
      }
    }
    buffer += stride;
  }
}

DECLARE_DEVICE(Vga);
