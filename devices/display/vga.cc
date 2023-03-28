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
#include "vga.pb.h"
#include "vga_regs.h"
#include "machine.h"

#define VGA_PIO_BASE    0x3C0
#define VGA_PIO_SIZE    0x20
#define VBE_PIO_BASE    0x1CE
#define VBE_PIO_SIZE    2
#define VBE_LINEAR_FRAMEBUFFER_BASE 0xE0000000

// When LFB mode disabled, the tradition VGA video memory address is used
#define VGA_MMIO_BASE   0x000A0000
#define VGA_MMIO_SIZE   0x00020000


Vga::Vga() {
  default_rom_path_ = "../share/vgabios-stdvga.bin";
  
  /* PCI config */
  pci_header_.vendor_id = 0x1234;
  pci_header_.device_id = 0x1111;
  pci_header_.class_code = 0x030000;
  pci_header_.header_type = PCI_HEADER_TYPE_NORMAL;
  pci_header_.subsys_vendor_id = 0x1AF4;
  pci_header_.subsys_id = 0x1100;
  

  /* Bar 0: 256MB VRAM */
  vga_mem_size_ = _MB(16);
  vram_size_ = _MB(256);

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

  vram_map_addr_ = 0;
  vram_map_size_ = 0;
  mode_ = kDisplayUnknownMode;
  width_ = 640;
  height_ = 480;
  bpp_ = 8;
  rows_ = cols_ = 0;
  font_height_ = font_width_ = 0;
  cursor_blink_time_ = std::chrono::steady_clock::now();
  cursor_visible_ = false;
  text_mode_ = false;

  if (vram_rw_mapped_) {
    RemoveIoResource(kIoResourceTypeRam, "VGA RAM");
    vram_rw_mapped_ = nullptr;
  }
}


bool Vga::SaveState(MigrationWriter* writer) {
  VgaState state;
  state.set_misc_output(vga_.misc_output);
  state.set_status(*(uint16_t*)vga_.status);
  state.set_sequencer_index(vga_.sequencer_index);
  state.set_sequencer(vga_.sequencer, sizeof(vga_.sequencer));
  state.set_gfx_index(vga_.gfx_index);
  state.set_gfx(vga_.gfx, sizeof(vga_.gfx));
  state.set_attribute_index(vga_.attribute_index);
  state.set_attribute(vga_.attribute, sizeof(vga_.attribute));
  state.set_crtc_index(vga_.crtc_index);
  state.set_crtc(vga_.crtc, sizeof(vga_.crtc));
  state.set_palette_read_index(vga_.palette_read_index);
  state.set_palette_write_index(vga_.palette_write_index);
  state.set_palette(vga_.palette, sizeof(vga_.palette));
  state.set_dac_state(vga_.dac_state);
  state.set_feature_control(vga_.feature_control);
  state.set_latch(vga_.latch);
  writer->WriteProtobuf("VGA", state);

  VbeState vbe_state;
  vbe_state.set_version(vbe_.version);
  vbe_state.set_index(vbe_.index);
  vbe_state.set_registers(vbe_.registers, sizeof(vbe_.registers));
  writer->WriteProtobuf("VBE", vbe_state);
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

  VgaState state;
  if (!reader->ReadProtobuf("VGA", state)) {
    return false;
  }
  vga_.misc_output = state.misc_output();
  *(uint16_t*)vga_.status = state.status();
  vga_.sequencer_index = state.sequencer_index();
  memcpy(vga_.sequencer, state.sequencer().data(), sizeof(vga_.sequencer));
  vga_.gfx_index = state.gfx_index();
  memcpy(vga_.gfx, state.gfx().data(), sizeof(vga_.gfx));
  vga_.attribute_index = state.attribute_index();
  memcpy(vga_.attribute, state.attribute().data(), sizeof(vga_.attribute));
  vga_.crtc_index = state.crtc_index();
  memcpy(vga_.crtc, state.crtc().data(), sizeof(vga_.crtc));
  vga_.palette_read_index = state.palette_read_index();
  vga_.palette_write_index = state.palette_write_index();
  memcpy(vga_.palette, state.palette().data(), sizeof(vga_.palette));
  vga_.dac_state = state.dac_state();
  vga_.feature_control = state.feature_control();
  vga_.latch = state.latch();

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
  if (!pci_rom_.data) {
    if (has_key("rom")) {
      std::string path = std::get<std::string>(key_values_["rom"]);
      LoadRomFile(path.c_str());
    } else {
      LoadRomFile(default_rom_path_.c_str());
    }
  }
  if (!vram_base_) {
    if (has_key("vram_size")) {
      uint64_t size = std::get<uint64_t>(key_values_["vram_size"]);
      MV_ASSERT(size >= 16 && size <= 512);
      if (size & (size - 1)) {
        MV_PANIC("vram_size must be power of 2");
      }
      vram_size_ = size << 20;
    }
    if (has_key("vga_size")) {
      uint64_t size = std::get<uint64_t>(key_values_["vga_size"]);
      MV_ASSERT(size >= 2 && size <= 256);
      if (size & (size - 1)) {
        MV_PANIC("vga_size must be power of 2");
      }
      vga_mem_size_ = size << 20;
    }
    vram_base_ = (uint8_t*)mmap(nullptr, vram_size_, PROT_READ | PROT_WRITE,
      MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
    pci_bars_[0].size = vram_size_;
    pci_bars_[0].host_memory = vram_base_;

    MV_ASSERT(madvise(vram_base_, vram_size_, MADV_DONTDUMP) == 0);
  }

  if (has_key("debug_io")) {
    debug_io_ = std::get<bool>(key_values_["debug_io"]);
  }

  PciDevice::Connect();

  refresh_timer_ = AddTimer(NS_PER_SECOND / 30, true, std::bind(&Vga::OnRefreshTimer, this));
}

void Vga::Disconnect() {
  if (vram_base_) {
    munmap((void*)vram_base_, vram_size_);
    vram_base_ = nullptr;
  }

  if (refresh_timer_) {
    RemoveTimer(refresh_timer_);
    refresh_timer_ = nullptr;
  }
  mode_ = kDisplayUnknownMode;
  PciDevice::Disconnect();
}

void Vga::OnRefreshTimer() {
  if (mode_ == kDisplayVbeMode || mode_ == kDisplayVgaMode) {
    NotifyDisplayUpdate();
  }
}

void Vga::GetCursorLocation(uint8_t* x, uint8_t* y, uint8_t* sel_start, uint8_t* sel_end) {
  uint16_t location = (vga_.crtc[VGA_CRTC_CURSOR_HI] << 8) | (vga_.crtc[VGA_CRTC_CURSOR_LO]);
  *sel_start = vga_.crtc[VGA_CRTC_CURSOR_START] & 0x1F;
  *sel_end = vga_.crtc[VGA_CRTC_CURSOR_END] & 0x1F;
  *y = (location / 80) % 25;
  *x = location % 80;
}

void Vga::GetDisplayMode(int* w, int* h, int* bpp, int* stride) {
  if (w)
    *w = width_;
  if (h)
    *h = height_;
  if (bpp)
    *bpp = bpp_;
  if (stride)
    *stride = stride_;
}

void Vga::GetPalette(const uint8_t** palette, int* count) {
  int shift_control = (vga_.gfx[VGA_GFX_MODE] >> 5) & 3;
  *count = shift_control >= 2 ? 256 : 16;
  *palette = (const uint8_t*)vga_.palette;
  if (vbe_.registers[VBE_DISPI_ENABLED] & VBE_DISPI_8BIT_DAC) {
    MV_LOG("warning: 8bit DAC is in use");
  }
}

void Vga::Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  uint64_t port = resource->base + offset;

  if (resource->base == VGA_MMIO_BASE) {
    for (size_t i = 0; i < size; i++) {
      VgaReadMemory(port + i, &data[i]);
    }
  } else if (resource->base == VGA_PIO_BASE) {
    VgaReadPort(port, &data[0]);
    for (size_t i = 1; i < size; i++) {
      VgaReadPort(port + 1, &data[i]);
    }
  } else if (resource->base == VBE_PIO_BASE) {
    if (size == 2) {
      VbeReadPort(port, (uint16_t*)data);
    }
  } else {
    PciDevice::Read(resource, offset, data, size);
  }
}

void Vga::Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  uint64_t port = resource->base + offset;
  if (resource->base == VGA_MMIO_BASE) {
    for (size_t i = 0; i < size; i++) {
      VgaWriteMemory(port + i, data[i]);
    }
  } else if (resource->base == VGA_PIO_BASE) {
    VgaWritePort(port, data[0]);
    for (size_t i = 1; i < size; i++) {
      VgaWritePort(port + 1, data[i]);
    }
  } else if (resource->base == VBE_PIO_BASE) {
    MV_ASSERT(size == 2);
    VbeWritePort(port, *(uint16_t*)data);
  } else {
    PciDevice::Write(resource, offset, data, size);
  }
}

void Vga::VbeReadPort(uint64_t port, uint16_t* data) {
  if (port == 0x1CE) {
    *data = vbe_.index;
  } else if (port == 0x1CF) {
    MV_ASSERT(vbe_.index <= VBE_DISPI_INDEX_NB);
    if (vbe_.index == VBE_DISPI_INDEX_VIDEO_MEMORY_64K) {
      *data = vram_size_ >> 16;
    } else {
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
    }
  }
}

void Vga::VbeWritePort(uint64_t port, uint16_t value) {
  if (port == 0x1CE) { // index
    MV_ASSERT(value <= VBE_DISPI_INDEX_NB);
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
        MV_LOG("set vbe enable to %x(mode=%d) %dx%d bpp=%d", value, mode_,
          vbe_.registers[1], vbe_.registers[2], vbe_.registers[3]);
      }
      if (value & VBE_DISPI_ENABLED) {
        vbe_.registers[VBE_DISPI_INDEX_VIRT_WIDTH] = 0;
        vbe_.registers[VBE_DISPI_INDEX_VIRT_HEIGHT] = 0;
        vbe_.registers[VBE_DISPI_INDEX_X_OFFSET] = 0;
        vbe_.registers[VBE_DISPI_INDEX_Y_OFFSET] = 0;
        UpdateDisplayMode();
        if (!(value & VBE_DISPI_NOCLEARMEM)) {
          bzero(vram_base_, height_ * stride_);
        }
      } else {
        vbe_.registers[VBE_DISPI_INDEX_BANK] = 0;
      }
      UpdateVRamMemoryMap();
      break;
    case VBE_DISPI_INDEX_BANK:
      UpdateVRamMemoryMap();
      break;
    }
  }
}

void Vga::VgaReadPort(uint64_t port, uint8_t* data) {
  switch (port)
  {
  case VGA_ATT_W:
    *data = vga_.attribute_index & 0x80 ? 0 : vga_.attribute_index;
    break;
  case VGA_ATT_R:
    *data = vga_.attribute[vga_.attribute_index & 0x1F];
    break;
  case VGA_SEQ_I:
    *data = vga_.sequencer_index;
    break;
  case VGA_SEQ_D:
    *data = vga_.sequencer[vga_.sequencer_index];
    break;
  case VGA_PEL_MSK: 
    *data = 0xFF;
    break;
  case VGA_PEL_IR:
    *data = vga_.dac_state;
    break;
  case VGA_PEL_IW:
    *data = vga_.palette_write_index;
    break;
  case VGA_PEL_D:
    *data = vga_.palette[vga_.palette_read_index++];
    break;
  case VGA_FTC_R:
    *data = vga_.feature_control;
    break;
  case VGA_MIS_R:
    *data = vga_.misc_output;
    break;
  case VGA_GFX_I:
    *data = vga_.gfx_index;
    break;
  case VGA_GFX_D:
    *data = vga_.gfx[vga_.gfx_index];
    break;
  case VGA_CRT_IC:
    *data = vga_.crtc_index;
    break;
  case VGA_CRT_DC:
    *data = vga_.crtc[vga_.crtc_index];
    break;
  case VGA_IS1_RC:
    vga_.attribute_index &= ~0x80; // Clears attribute flip-flop
    vga_.status[1] ^= 9;
    *data = vga_.status[1];
    break;
  default:
    *data = 0xFF;
    MV_ERROR("not implemented %s port=0x%lX data=0x%02X", name_, port, *data);
    break;
  }

  if (debug_io_) {
    MV_LOG("R %s(%lX) data=0x%02X", vga_port_name(port, false), port, *data);
  }
}

void Vga::VgaWritePort(uint64_t port, uint32_t value) {
  if (debug_io_) {
    MV_LOG("W %s(%lX) data=0x%02X", vga_port_name(port, true), port, value);
  }

  switch (port)
  {
  case VGA_ATT_IW:
    if (vga_.attribute_index & 0x80) { // set data
      vga_.attribute_index &= 0x1F;
      switch (vga_.attribute_index)
      {
      case VGA_ATC_PALETTE0 ... VGA_ATC_PALETTEF:
        vga_.attribute[vga_.attribute_index] = value & 0x3F;
        break;
      case VGA_ATC_MODE:
        vga_.attribute[vga_.attribute_index] = value & ~0x10;
        break;
      case VGA_ATC_OVERSCAN:
        vga_.attribute[vga_.attribute_index] = value;
        break;
      case VGA_ATC_PLANE_ENABLE:
        vga_.attribute[vga_.attribute_index] = value & ~0xC0;
        break;
      case VGA_ATC_PEL:
      case VGA_ATC_COLOR_PAGE:
        vga_.attribute[vga_.attribute_index] = value & ~0xF0;
        break;
      default:
        break;
      }
    } else { // set index
      vga_.attribute_index = 0x80 | (value & 0x3F);
      if (vga_.attribute_index & VGA_AR_ENABLE_DISPLAY) {
        // renderer changed event
        UpdateDisplayMode();
      }
    }
    break;
  case VGA_MIS_W:
    vga_.misc_output = value & ~0x10;
    break;
  case VGA_SEQ_I:
    vga_.sequencer_index = value & 7;
    break;
  case VGA_SEQ_D: {
    const uint8_t masks[] = { 0x3, 0x3D, 0xF, 0x3F, 0xE, 0, 0, 0xFF };
    vga_.sequencer[vga_.sequencer_index] = value & masks[vga_.sequencer_index];
    break;
  }
  case VGA_PEL_MSK:
    MV_ASSERT(value == 0xFF); // palette mask
    break;
  case VGA_PEL_IR:
    vga_.palette_read_index = value * 3;
    vga_.dac_state = 3;
    break;
  case VGA_PEL_IW:
    vga_.palette_write_index = value * 3;
    vga_.dac_state = 0;
    break;
  case VGA_PEL_D:
    vga_.palette[vga_.palette_write_index++] = value;
    break;
  case VGA_GFX_I:
    vga_.gfx_index = value & 0xF;
    break;
  case VGA_GFX_D: {
    const uint8_t masks[16] = { 0xF, 0xF, 0xF, 0x1F, 0x3, 0x7B, 0xF, 0xF, 0xFF, 0 };
    vga_.gfx[vga_.gfx_index] = value & masks[vga_.gfx_index];
    if (vga_.gfx_index == VGA_GFX_MISC) {
      UpdateVRamMemoryMap();
    }
    break;
  }
  case VGA_CRT_IC:
    vga_.crtc_index = value;
    break;
  case VGA_CRT_DC:
    vga_.crtc[vga_.crtc_index] = value;
    break;
  default:
    MV_ERROR("not implemented %s port=0x%lx data=0x%x", name_, port, value);
    break;
  }
}

void Vga::VgaReadMemory(uint64_t addr, uint8_t* data) {
  if (addr < vram_map_addr_ || addr >= vram_map_addr_ + vram_map_size_) {
    *data = 0xFF;
    return;
  }

  addr -= vram_map_addr_;

  if (vga_.sequencer[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M) {
    /* chain 4 mode */
    MV_ASSERT(addr < vram_size_);
    *data = vram_base_[addr];
  } else if (vga_.gfx[VGA_GFX_MODE] & 0x10) {
    /* odd / even (text) mode */
    uint plane = (vga_.gfx[VGA_GFX_PLANE_READ] & 2) | (addr & 1);
    addr = ((addr & ~1) << 1) | plane;
    MV_ASSERT(addr < vram_size_);
    *data = vram_base_[addr];
  } else {
    /* standard VGA latched access */
    MV_ASSERT(addr * sizeof(uint32_t) < vram_size_);
    vga_.latch = ((uint32_t*)vram_base_)[addr];

    if (vga_.gfx[VGA_GFX_MODE] & 8) {
      /* read mode 1 */
      uint value = (vga_.latch ^ mask16[vga_.gfx[VGA_GFX_COMPARE_VALUE]]) &
        mask16[vga_.gfx[VGA_GFX_COMPARE_MASK]];
      value |= value >> 16;
      value |= value >> 8;
      *data = (~value) & 0xFF;
    } else {
      /* read mode 0 */
      int plane = vga_.gfx[VGA_GFX_PLANE_READ];
      *data = (vga_.latch >> (plane * 8)) & 0xFF;
    }
  }
}

/* For graphic mode, VGA RAM normally starts at 0xA0000 while text mode at 0xB8000 */
void Vga::VgaWriteMemory(uint64_t addr, uint32_t value) {
  if (addr < vram_map_addr_ || addr >= vram_map_addr_ + vram_map_size_) {
    return;
  }

  addr -= vram_map_addr_;

  if (vga_.sequencer[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M) {
    /* chain 4 mode */
    uint plane = addr & 3;
    uint mask = (1 << plane);
    if (vga_.sequencer[VGA_SEQ_PLANE_WRITE] & mask) {
      vram_base_[addr] = value;
    }
  } else if (vga_.gfx[VGA_GFX_MODE] & 0x10) {
    /* odd / even (text) mode */
    uint plane = (vga_.gfx[VGA_GFX_PLANE_READ] & 2) | (addr & 1);
    uint mask = (1 << plane);
    if (vga_.sequencer[VGA_SEQ_PLANE_WRITE] & mask) {
      addr = ((addr & ~1) << 1) | plane;
      MV_ASSERT(addr < vram_size_);
      vram_base_[addr] = value;
    }
  } else {
    /* standard VGA latched access */
    uint write_mode = vga_.gfx[VGA_GFX_MODE] & 3;
    uint set_mask, bit_mask, write_mask, logical_function, b;

    switch (write_mode)
    {
    case 0:
      /* rotate */
      b = vga_.gfx[VGA_GFX_DATA_ROTATE] & 7;
      value = ((value >> b) | (value << (8 - b))) & 0xFF;
      value |= value << 8;
      value |= value << 16;

      /* apply set / reset mask */
      set_mask = mask16[vga_.gfx[VGA_GFX_SR_ENABLE]];
      value = (value & ~set_mask) | (mask16[vga_.gfx[VGA_GFX_SR_VALUE]] & set_mask);
      bit_mask = vga_.gfx[VGA_GFX_BIT_MASK];
      break;
    case 1:
      value = vga_.latch;
      goto write_value;
    case 2:
      value = mask16[value & 0xF];
      bit_mask = vga_.gfx[VGA_GFX_BIT_MASK];
      break;
    case 3:
      /* rotate */
      b = vga_.gfx[VGA_GFX_DATA_ROTATE] & 7;
      value = (value >> b) | (value << (8 - b));
      
      bit_mask = vga_.gfx[VGA_GFX_BIT_MASK] & value;
      value = mask16[vga_.gfx[VGA_GFX_SR_VALUE]];
      break;
    }

    /* apply logical operation */
    logical_function = vga_.gfx[VGA_GFX_DATA_ROTATE] >> 3;
    switch (logical_function)
    {
    case 1:
      value &= vga_.latch;
      break;
    case 2:
      value |= vga_.latch;
      break;
    case 3:
      value ^= vga_.latch;
      break;
    default:
      break;
    }

    /* apply bit mask */
    bit_mask |= bit_mask << 8;
    bit_mask |= bit_mask << 16;
    value = (value & bit_mask) | (vga_.latch & ~bit_mask);

write_value:
    /* mask data according to plane write */
    write_mask = mask16[vga_.sequencer[VGA_SEQ_PLANE_WRITE]];
    MV_ASSERT(addr * sizeof(uint32_t) < vram_size_);
    auto write_ptr = (uint32_t*)vram_base_;
    write_ptr[addr] = (write_ptr[addr] & ~write_mask) | (value & write_mask);

    // printf("vga: latch: [0x%016lx] mask=0x%08x val=0x%08x\n",
    //         addr * 4, write_mask, value);
  }
}

void Vga::UpdateVRamMemoryMap() {
  uint8_t* map_memory = nullptr;
  if (vbe_.registers[VBE_DISPI_INDEX_ENABLE] & VBE_DISPI_ENABLED) {
    vram_map_addr_ = VGA_MMIO_BASE;
    vram_map_size_ = VGA_MMIO_SIZE;
  
    map_memory = vram_base_ + (vbe_.registers[VBE_DISPI_INDEX_BANK] << 16);
  } else {
    const size_t map_types[][2] = {
      { 0xA0000, 0x20000 }, { 0xA0000, 0x10000 },
      { 0xB0000, 0x08000 }, { 0xB8000, 0x08000 }
    };
    
    /* Memory map select controls visual area while read select controls IO */
    int index = (vga_.gfx[VGA_GFX_MISC] >> 2) & 0b11;
    auto& map_type = map_types[index];
    vram_map_addr_ = map_type[0];
    vram_map_size_ = map_type[1];
  }
  
  /* Map / unmap the area as ram to accelerate */
  if (vram_rw_mapped_ != map_memory) {
    if (vram_rw_mapped_) {
      RemoveIoResource(kIoResourceTypeRam, "VGA RAM");
    }
    if (map_memory) {
      AddIoResource(kIoResourceTypeRam, vram_map_addr_, vram_map_size_, "VGA RAM", map_memory);
      if (debug_) {
        MV_LOG("map plane addr=0x%08x[0x08%x] to 0x%016lx", vram_map_addr_, vram_map_size_, map_memory);
      }

      vram_rw_mapped_ = map_memory;
    }
  }
}

void Vga::UpdateDisplayMode() {
  auto old_mode = mode_;
  auto old_w = width_, old_h = height_, old_bpp = bpp_;

  if (vbe_.registers[VBE_DISPI_INDEX_ENABLE] & VBE_DISPI_ENABLED) {
    /* Read resolution from VBE regsiters */
    mode_ = kDisplayVbeMode;

    auto& r = vbe_.registers;
    r[VBE_DISPI_INDEX_XRES] &= ~3;
    if (r[VBE_DISPI_INDEX_XRES] == 0) {
      r[VBE_DISPI_INDEX_XRES] = 4;
    }
    if (r[VBE_DISPI_INDEX_XRES] > VBE_DISPI_MAX_XRES) {
      r[VBE_DISPI_INDEX_XRES] = VBE_DISPI_MAX_XRES;
    }
    r[VBE_DISPI_INDEX_VIRT_WIDTH] &= ~3;
    if (r[VBE_DISPI_INDEX_VIRT_WIDTH] > VBE_DISPI_MAX_XRES) {
      r[VBE_DISPI_INDEX_VIRT_WIDTH] = VBE_DISPI_MAX_XRES;
    }
    if (r[VBE_DISPI_INDEX_VIRT_WIDTH] < r[VBE_DISPI_INDEX_XRES]) {
      r[VBE_DISPI_INDEX_VIRT_WIDTH] = r[VBE_DISPI_INDEX_XRES];
    }

    if (r[VBE_DISPI_INDEX_YRES] == 0) {
      r[VBE_DISPI_INDEX_YRES] = 1;
    }
    if (r[VBE_DISPI_INDEX_YRES] > VBE_DISPI_MAX_YRES) {
      r[VBE_DISPI_INDEX_YRES] = VBE_DISPI_MAX_YRES;
    }

    if (r[VBE_DISPI_INDEX_X_OFFSET] > VBE_DISPI_MAX_XRES) {
      r[VBE_DISPI_INDEX_X_OFFSET] = VBE_DISPI_MAX_XRES;
    }
    if (r[VBE_DISPI_INDEX_Y_OFFSET] > VBE_DISPI_MAX_YRES) {
      r[VBE_DISPI_INDEX_Y_OFFSET] = VBE_DISPI_MAX_YRES;
    }

    width_ = r[VBE_DISPI_INDEX_XRES];
    height_ = r[VBE_DISPI_INDEX_YRES];
    bpp_ = r[VBE_DISPI_INDEX_BPP];
    stride_ = r[VBE_DISPI_INDEX_VIRT_WIDTH] * bpp_ / 8;
  } else  if (vga_.attribute_index & VGA_AR_ENABLE_DISPLAY) {
    /* Read resolution from VGA registers */
    mode_ = kDisplayVgaMode;
    text_mode_ = !(vga_.gfx[VGA_GFX_MISC] & VGA_GR06_GRAPHICS_MODE);
  
    font_height_ = (vga_.crtc[VGA_CRTC_MAX_SCAN] & 0x1F) + 1;
    font_width_ = 8;
    if (!(vga_.sequencer[VGA_SEQ_CLOCK_MODE] & VGA_SR01_CHAR_CLK_8DOTS)) {
      font_width_ = 9;
    }
    cols_ = vga_.crtc[VGA_CRTC_H_DISP] + 1;
    width_ = cols_ * font_width_;
    height_ = vga_.crtc[VGA_CRTC_V_DISP_END] |
      ((vga_.crtc[VGA_CRTC_OVERFLOW] & 0x02) << 7) |
      ((vga_.crtc[VGA_CRTC_OVERFLOW] & 0x40) << 3);
    height_ += 1;
    rows_ = height_ / font_height_;
    bpp_ = 8;
    stride_ = width_ * bpp_ / 8;
  } else {
    mode_ = kDisplayUnknownMode;
    return;
  }

  if (old_mode != mode_ || old_w != width_ || old_h != height_ || old_bpp != bpp_) {
    NotifyDisplayModeChange();
  }
}

void Vga::NotifyDisplayModeChange() {
  if (debug_) {
    MV_LOG("update mode=%d %dx%dx%d stride=%d", mode_, width_, height_, bpp_, stride_);
  }
  for (auto &listener : display_mode_change_listeners_) {
    listener();
  }
}

void Vga::NotifyDisplayUpdate() {
  for (auto &listener : display_update_listeners_) {
    listener();
  }
}

void Vga::RegisterDisplayModeChangeListener(DisplayModeChangeListener callback) {
  display_mode_change_listeners_.push_back(callback);
}

void Vga::RegisterDisplayUpdateListener(DisplayUpdateListener callback) {
  display_update_listeners_.push_back(callback);
}

/* This interface method is called by UI thread, remember to lock the device */
bool Vga::AcquireUpdate(DisplayUpdate& update, bool redraw) {
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  if (mode_ == kDisplayUnknownMode) {
    return false;
  }
  /* VGA always update the whole surface */
  MV_UNUSED(redraw);

  DisplayPartialBitmap partial;
  partial.stride = stride_;
  partial.bpp = bpp_;
  partial.width = width_;
  partial.height = height_;
  partial.x = 0;
  partial.y = 0;
  partial.palette = vga_.palette;

  if (mode_ == kDisplayVbeMode) {
    size_t offset = vbe_.registers[VBE_DISPI_INDEX_X_OFFSET] * bpp_ / 8;
    offset += vbe_.registers[VBE_DISPI_INDEX_Y_OFFSET] * stride_;
    partial.data = vram_base_ + offset;
  } else if (mode_ == kDisplayVgaMode) {
    size_t data_size = stride_ * height_;
    if (vga_surface_.size() != data_size) {
      vga_surface_.resize(data_size);
    }
    partial.data = (uint8_t*)vga_surface_.data();

    /* FIXME: Should unlock the device but crashed when display mode was changing */
    // lock.unlock();
    if (text_mode_) {
      DrawText(partial.data);

      /* blink cursor */
      auto now = std::chrono::steady_clock::now();
      if (now >= cursor_blink_time_) {
        cursor_blink_time_ = now + std::chrono::milliseconds(250);
        cursor_visible_ = !cursor_visible_;
      }
      if (cursor_visible_) {
        DrawTextCursor(partial.data);
      }
    } else if (mode_ == kDisplayVgaMode) {
      DrawGraphic(partial.data);
    }
  }

  update.partials.emplace_back(std::move(partial));
  update.cursor.visible = false;
  return true;
}

void Vga::ReleaseUpdate() {
}

void Vga::DrawGraphic(uint8_t* buffer) {
  /* shift_control to determine 640x480 16 or 256 color mode */
  int shift_control = (vga_.gfx[VGA_GFX_MODE] >> 5) & 3;
  uint plane_mask = vga_.attribute[VGA_ATC_PLANE_ENABLE];
  uint8_t* dst = buffer;
  uint8_t* src = vram_base_;
  uint src_stride = vga_.crtc[VGA_CRTC_OFFSET] << 3;
  uint start_addr = vga_.crtc[VGA_CRTC_START_LO] | (vga_.crtc[VGA_CRTC_START_HI] << 8);
  src += start_addr * 4;

  for (int y = 0; y < rows_; y++) {
    auto s = src, d = dst;

    if (shift_control >= 2) {
      for (int x = 0; x < cols_; x++) {
        d[0] = d[1] = s[0];
        d[2] = d[3] = s[1];
        d[4] = d[5] = s[2];
        d[6] = d[7] = s[3];
        d += 8;
        s += 4;
      }
    } else {
      for (int x = 0; x < cols_; x++) {
        uint8_t p0 = s[0], p1 = s[1], p2 = s[2], p3 = s[3];
        for (int bit = 0x80; bit >= 1; bit >>= 1) {
          uint8_t pixel = ((p0 & bit) ? 1 : 0) |
                          ((p1 & bit) ? 2 : 0) |
                          ((p2 & bit) ? 4 : 0) |
                          ((p3 & bit) ? 8 : 0);
          pixel &= plane_mask;
          *d++ = vga_.attribute[pixel] & 0xF;
        }
        s += 4;
      }
    }
    src += src_stride;
    dst += stride_;
  }
}

void Vga::DrawText(uint8_t* buffer) {
  uint character_map = vga_.sequencer[VGA_SEQ_CHARACTER_MAP];
  uint8_t* fonts[2];
  fonts[0] = vram_base_ + ((character_map >> 5 & 1) | (character_map >> 1 & 6)) * 0x4000;
  fonts[1] = vram_base_ + ((character_map >> 4 & 1) | (character_map << 1 & 6)) * 0x4000;

  uint line_offset = vga_.crtc[VGA_CRTC_OFFSET] << 3;
  uint start_addr = (vga_.crtc[VGA_CRTC_START_HI] << 8) | (vga_.crtc[VGA_CRTC_START_LO]);
  uint8_t* dst = buffer;
  uint8_t* src = vram_base_ + start_addr * 4;

  for (int y = 0; y < rows_; y++) {
    auto s = src, d = dst;
    for (int x = 0; x < cols_; x++) {
      int character = s[0], attribute = s[1];
      DrawCharacter(d, fonts[(attribute >> 3) & 1], character, attribute);
      s += 4;
      d += font_width_;
    }

    src += line_offset;
    dst += stride_ * font_height_;
  }
}

/* supports font width 8 / 9 */
void Vga::DrawCharacter(uint8_t* dest, uint8_t* font, int character, int attribute) {
  uint8_t fore_color = attribute & 0xF;
  uint8_t back_color = (attribute >> 4) & 0xF;
  font += 2 + character * 32 * 4;
  bool dup9 = (vga_.attribute[VGA_ATC_MODE] & 0x4) && character >= 0xB0 && character <= 0xDF;

  for (int yy = 0; yy < font_height_; yy++) {
    uint value;
    uint font_data = font[0];
    uint8_t* ptr = dest;
    for (uint bit = 0x80; bit >= 1; bit >>= 1) {
      value = font_data & bit ? fore_color : back_color;
      *ptr++ = value;
    }
    if (font_width_ >= 9) {
      *ptr++ = dup9 ? value : back_color;
    }
    dest += stride_;
    font += 4;
  }
}

void Vga::DrawTextCursor(uint8_t* buffer) {
  uint32_t fore_color = 7;
  uint8_t cx, cy, sl_start, sl_end;
  GetCursorLocation(&cx, &cy, &sl_start, &sl_end);

  buffer += (cy * font_height_) * stride_;
  buffer += cx * font_width_;

  uint8_t* end = buffer + stride_ * height_;

  buffer += stride_ * sl_start;

  for (int y = sl_start; y < sl_end; y++) {
    for (int x = 0; x < font_width_; x++) {
      if (buffer + x < end) {
        buffer[x] = fore_color;
      }
    }
    buffer += stride_;
  }
}

DECLARE_DEVICE(Vga);
