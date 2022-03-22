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
#include "states/vga.pb.h"

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
  pci_header_.header_type = PCI_HEADER_TYPE_NORMAL;
  pci_header_.subsys_vendor_id = 0x1AF4;
  pci_header_.subsys_id = 0x1100;
  pci_header_.command = PCI_COMMAND_IO | PCI_COMMAND_MEMORY;
  

  /* Bar 0: 64MB VRAM */
  vga_mem_size_ = _MB(16);
  vram_size_ = _MB(64);

  AddPciBar(0, vram_size_, kIoResourceTypeRam);    /* vgamem */
  /* FIXME: bar 2 should be implemented for stdvga if Qxl is not enabled??? */

  AddIoResource(kIoResourceTypePio, VGA_PIO_BASE, VGA_PIO_SIZE, "VGA IO");
  AddIoResource(kIoResourceTypePio, VBE_PIO_BASE, VBE_PIO_SIZE, "VBE IO");
  AddIoResource(kIoResourceTypeMmio, VGA_MMIO_BASE, VGA_MMIO_SIZE, "VGA MMIO",
    nullptr, kIoResourceFlagCoalescingMmio);
}

Vga::~Vga() {
}

void Vga::Reset() {
  PciDevice::Reset();
  bzero(&vbe_, sizeof(vbe_));
  bzero(&vga_, sizeof(vga_));

  vram_map_select_ = vram_base_;
  vram_read_select_ = vram_base_;
  mode_ = kDisplayTextMode;
  width_ = 640;
  height_ = 400;
  bpp_ = 8;
}


bool Vga::SaveState(MigrationWriter* writer) {
  VgaState state;
  state.set_misc_output(vga_.misc_output);
  state.set_status(*(uint16_t*)vga_.status);
  state.set_sequence_index(vga_.sequence_index);
  state.set_sequence(vga_.sequence, sizeof(vga_.sequence));
  state.set_gfx_index(vga_.gfx_index);
  state.set_gfx(vga_.gfx, sizeof(vga_.gfx));
  state.set_attribute_index(vga_.attribute_index);
  state.set_attribute(vga_.attribute, sizeof(vga_.attribute));
  state.set_crtc_index(vga_.crtc_index);
  state.set_crtc(vga_.crtc, sizeof(vga_.crtc));
  state.set_pallete_read_index(vga_.pallete_read_index);
  state.set_pallete_write_index(vga_.pallete_write_index);
  state.set_pallete(vga_.pallete, sizeof(vga_.pallete));
  state.set_dac_state(vga_.dac_state);
  state.set_feature_control(vga_.feature_control_);
  writer->WriteProtobuf("VGA", state);

  VbeState vbe_state;
  vbe_state.set_version(vbe_.version);
  vbe_state.set_index(vbe_.index);
  vbe_state.set_registers(vbe_.registers, sizeof(vbe_.registers));
  writer->WriteProtobuf("VBE", vbe_state);
  writer->WriteRaw("VRAM", vram_base_, vram_size_);
  return PciDevice::SaveState(writer);
}

bool Vga::LoadState(MigrationReader* reader) {
  if (!PciDevice::LoadState(reader)) {
    return false;
  }
  if (!reader->ReadRaw("VRAM", vram_base_, vram_size_)) {
    return false;
  }
  VgaState state;
  if (!reader->ReadProtobuf("VGA", state)) {
    return false;
  }
  vga_.misc_output = state.misc_output();
  *(uint16_t*)vga_.status = state.status();
  vga_.sequence_index = state.sequence_index();
  memcpy(vga_.sequence, state.sequence().data(), sizeof(vga_.sequence));
  vga_.gfx_index = state.gfx_index();
  memcpy(vga_.gfx, state.gfx().data(), sizeof(vga_.gfx));
  vga_.attribute_index = state.attribute_index();
  memcpy(vga_.attribute, state.attribute().data(), sizeof(vga_.attribute));
  vga_.crtc_index = state.crtc_index();
  memcpy(vga_.crtc, state.crtc().data(), sizeof(vga_.crtc));
  vga_.pallete_read_index = state.pallete_read_index();
  vga_.pallete_write_index = state.pallete_write_index();
  memcpy(vga_.pallete, state.pallete().data(), sizeof(vga_.pallete));
  vga_.dac_state = state.dac_state();
  vga_.feature_control_ = state.feature_control();

  VbeState vbe_state;
  if (!reader->ReadProtobuf("VBE", vbe_state)) {
    return false;
  }
  vbe_.version = vbe_state.version();
  vbe_.index = vbe_state.index();
  memcpy(vbe_.registers, vbe_state.registers().data(), sizeof(vbe_.registers));

  UpdateDisplayMode();
  UpdateVRamMemoryMap();
  return true;
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
  uint16_t location = (vga_.crtc[0xE] << 8) | (vga_.crtc[0xF]);
  *sel_start = vga_.crtc[0xA] & 0x1F;
  *sel_end = vga_.crtc[0xB] & 0x1F;
  *y = (location / 80) % 25;
  *x = location % 80;
}

void Vga::GetDisplayMode(uint16_t* w, uint16_t* h, uint16_t* bpp) {
  *w = width_;
  *h = height_;
  *bpp = bpp_;
}

const uint8_t* Vga::GetPallete() const {
  return vga_.pallete;
}

void Vga::Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  uint64_t port = resource->base + offset;

  if (resource->base == VGA_MMIO_BASE) {
    memcpy(data, vram_read_select_ + offset, size);
  } else if (resource->base == VGA_PIO_BASE) {
    VgaReadPort(port, data, size);
  } else if (resource->base == VBE_PIO_BASE) {
    MV_ASSERT(size == 2);
    VbeReadPort(port, (uint16_t*)data);
  } else {
    MV_PANIC("unhandled read at base=0x%lx offset=0x%lx data=0x%lx size=%x",
      resource->base, offset, *(uint64_t*)data, size);
  }
}

void Vga::Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  uint64_t port = resource->base + offset;
  if (resource->base == VGA_MMIO_BASE) {
    memcpy(vram_read_select_ + offset, data, size);
  } else if (resource->base == VGA_PIO_BASE) {
    VgaWritePort(port, data, size);
  } else if (resource->base == VBE_PIO_BASE) {
    VbeWritePort(port, *(uint16_t*)data);
  } else {
    MV_PANIC("unhandled write at base=0x%lx offset=0x%lx data=0x%lx size=%x",
      resource->base, offset, *(uint64_t*)data, size);
  }
}

void Vga::VbeReadPort(uint64_t port, uint16_t* data) {
  if (port == 0x1CE) {
    *data = vbe_.index;
    return;
  }
  if (vbe_.index < VBE_DISPI_INDEX_NB) {
    if (vbe_.registers[VBE_DISPI_INDEX_ENABLE] & VBE_DISPI_GETCAPS) {
      /* VBE initialization will enable and get capabilities and then disable */
      const uint16_t max_values[] = {
        0, VBE_DISPI_MAX_XRES, VBE_DISPI_MAX_YRES, VBE_DISPI_MAX_BPP
      };
      MV_ASSERT(vbe_.index < sizeof(max_values) / sizeof(uint16_t));
      *data = max_values[vbe_.index];
    } else {
      *data = vbe_.registers[vbe_.index];
    }
  } else if (vbe_.index == VBE_DISPI_INDEX_VIDEO_MEMORY_64K) {
    *data = vram_size_ >> 16;
  } else {
    *data = 0;
    MV_PANIC("invalid read index = %d", vbe_.index);
  }
}

void Vga::VbeWritePort(uint64_t port, uint16_t value) {
  if (port == 0x1CE) { // index
    if (value > VBE_DISPI_INDEX_NB) {
      MV_PANIC("invalid vbe index 0x%x", value);
    }
    vbe_.index = value;
  } else if (port == 0x1CF) { // data
    vbe_.registers[vbe_.index] = value;
    switch (vbe_.index)
    {
    case VBE_DISPI_INDEX_ID:
      vbe_.version = value;
      break;
    case VBE_DISPI_INDEX_ENABLE:
      if (debug_) {
        MV_LOG("set vbe enable to %x %dx%d bpp=%d", value,
          vbe_.registers[1], vbe_.registers[2], vbe_.registers[3]);
      }
      if (value & 1) {
        UpdateDisplayMode();
      }
      break;
    }
    UpdateVRamMemoryMap();
  }
}

void Vga::VgaReadPort(uint64_t port, uint8_t* data, uint32_t size) {
  switch (port)
  {
  case 0x3C0:
    MV_ASSERT(size == 1);
    *data = vga_.attribute_index;
    break;
  case 0x3C1:
    MV_ASSERT(size == 1);
    *data = vga_.attribute[vga_.attribute_index & 0x7F];
    break;
  case 0x3C4:
    MV_ASSERT(size == 1);
    *data = vga_.sequence_index;
    break;
  case 0x3C5:
    MV_ASSERT(size == 1);
    *data = vga_.sequence[vga_.sequence_index];
    break;
  case 0x3C6: 
    *data = 0xFF;
    break;
  case 0x3C7:
    *data = vga_.dac_state;
    break;
  case 0x3C8:
    *data = vga_.pallete_write_index;
    break;
  case 0x3C9:
    for (uint32_t i = 0; i < size; i++) {
      *data++ = vga_.pallete[vga_.pallete_read_index++];
    }
    break;
  case 0x3CA:
    MV_ASSERT(size == 1);
    *data = vga_.feature_control_;
    break;
  case 0x3CC:
    MV_ASSERT(size == 1);
    *data = vga_.misc_output;
    break;
  case 0x3CE:
    MV_ASSERT(size == 1);
    *data = vga_.gfx_index;
    break;
  case 0x3CF:
    MV_ASSERT(size == 1);
    *data = vga_.gfx[vga_.gfx_index];
    break;
  case 0x3D4:
    MV_ASSERT(size == 1);
    *data = vga_.crtc_index;
    break;
  case 0x3D5:
    MV_ASSERT(size == 1);
    *data = vga_.crtc[vga_.crtc_index];
    break;
  case 0x3DA:
    MV_ASSERT(size == 1);
    vga_.attribute_index &= ~0x80; // Clears attribute flip-flop
    vga_.status[1] ^= 9;
    *data = vga_.status[1];
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
    if (vga_.attribute_index & 0x80) { // set data
      vga_.attribute_index &= ~0x80;
      vga_.attribute[vga_.attribute_index] = value;
    } else { // set index
      vga_.attribute_index = 0x80 | value;
      if (vga_.attribute_index & 0x20) {
        // renderer changed event
        UpdateDisplayMode();
      }
    }
    break;
  case 0x3C2:
    MV_ASSERT(size == 1);
    vga_.misc_output = value & ~0x10;
    break;
  case 0x3C4:
    vga_.sequence_index = value;
    if (size == 2) {
      vga_.sequence[vga_.sequence_index] = data[1];
      if (vga_.sequence_index == 4) {
        vga_.sequence[vga_.sequence_index] &= 0b1110;
      }
    }
    break;
  case 0x3C5:
    MV_ASSERT(size == 1);
    vga_.sequence[vga_.sequence_index] = value;
    break;
  case 0x3C6:
    MV_ASSERT(value == 0xFF); // pallete mask
    break;
  case 0x3C7:
    MV_ASSERT(size == 1);
    vga_.pallete_read_index = value * 3;
    vga_.dac_state = 3;
    break;
  case 0x3C8:
    MV_ASSERT(size == 1);
    vga_.pallete_write_index = value * 3;
    vga_.dac_state = 0;
    break;
  case 0x3C9:
    MV_ASSERT(size == 1);
    vga_.pallete[vga_.pallete_write_index++] = value;
    break;
  case 0x3CE:
    vga_.gfx_index = value;
    if (size == 2) {
      vga_.gfx[vga_.gfx_index] = data[1];
      if (vga_.gfx_index == 4 || vga_.gfx_index == 6) {
        UpdateVRamMemoryMap();
      }
    }
    break;
  case 0x3CF:
    MV_ASSERT(size == 1);
    vga_.gfx[vga_.gfx_index] = value;
    if (vga_.gfx_index == 4 || vga_.gfx_index == 6) {
      UpdateVRamMemoryMap();
    }
    break;
  case 0x3D4:
    vga_.crtc_index = value;
    if (size == 2) {
      vga_.crtc[vga_.crtc_index] = data[1];
    }
    break;
  case 0x3D5:
    MV_ASSERT(size == 1);
    vga_.crtc[vga_.crtc_index] = value;
    break;
  default:
    MV_PANIC("not implemented %s port=0x%lx size=%d data=0x%lx",
      name_, port, size, value);
    break;
  }
}

void Vga::UpdateVRamMemoryMap() {
  if (mode_ == kDisplayVbeMode) {
    auto bpp = vbe_.registers[VBE_DISPI_INDEX_BPP];
    uint stride = vbe_.registers[VBE_DISPI_INDEX_VIRT_WIDTH] * bpp / 8;
    uint offset = vbe_.registers[VBE_DISPI_INDEX_X_OFFSET] * bpp / 8;
    offset += vbe_.registers[VBE_DISPI_INDEX_Y_OFFSET] * stride;
  
    vram_map_select_ = vram_base_ + offset;
    vram_map_select_size_ = vram_size_;
    vram_read_select_ = vram_base_ + (vbe_.registers[VBE_DISPI_INDEX_BANK] << 16);
    if (debug_) {
      MV_LOG("VBE map offset=0x%lx, bank offset=0x%lx", offset, (vbe_.registers[VBE_DISPI_INDEX_BANK] << 16));
    }
  
    /* Map / unmap the area as ram to accelerate */
    if (has_mapped_vga_) {
      RemoveIoResource(kIoResourceTypeRam, VGA_MMIO_BASE);
    }
    AddIoResource(kIoResourceTypeRam, VGA_MMIO_BASE, VGA_MMIO_SIZE, "VGA RAM", vram_read_select_);
    has_mapped_vga_ = true;
  } else if (mode_ == kDisplayVgaMode || mode_ == kDisplayTextMode) {
    const size_t map_types[][2] = {
      { 0xA0000, 0x20000 }, { 0xA0000, 0x10000 },
      { 0xB0000, 0x08000 }, { 0xB8000, 0x08000 }
    };
    
    /* Memory map select controls visual area while read select controls IO */
    int index = (vga_.gfx[6] >> 2) & 0b11;
    int read_index = vga_.gfx[4] & 0b11;
    vram_map_select_size_ = map_types[index][1];
    vram_map_select_ = vram_base_ + map_types[index][0] - VGA_MMIO_BASE;
    vram_read_select_ = vram_base_ + vram_map_select_size_ * read_index;

    if (debug_) {
      MV_LOG("VGA map index=%d read_index=%d", index, read_index);
    }
  }
}

void Vga::UpdateDisplayMode() {
  if (vbe_.registers[VBE_DISPI_INDEX_ENABLE] & VBE_DISPI_ENABLED) {
    mode_ = kDisplayVbeMode;
    width_ = vbe_.registers[VBE_DISPI_INDEX_XRES];
    height_ = vbe_.registers[VBE_DISPI_INDEX_YRES];
    bpp_ = vbe_.registers[VBE_DISPI_INDEX_BPP];
  } else if ((vga_.gfx[6] & 0x1) == 0) {
    mode_ = kDisplayTextMode;
    width_ = 640;
    height_ = 400;
    bpp_ = 8;
  } else {
    /* FIXME: width/height/bpp should be calculate from VGA regs */
    mode_ = kDisplayVgaMode;
    width_ = 640;
    height_ = 480;
    bpp_ = 8;
  }
  NotifyDisplayModeChange();
}

void Vga::NotifyDisplayModeChange() {
  for (auto &listener : display_change_listerners_) {
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
  for (auto &renderer : display_render_callbacks_) {
    renderer(partial);
  }
}

void Vga::NotifyDisplayCursorUpdate(DisplayCursorUpdate* update) {
  for (auto &callback : display_cursor_callbacks_) {
    callback(update);
  }
}

void Vga::OnRefreshTimer() {
  if (mode_ == kDisplayTextMode) {
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
  partial->Release = [partial]() {
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
  partial->Release = [partial]() {
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
