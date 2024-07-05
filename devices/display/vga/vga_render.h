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

#ifndef _MVISOR_DEVICES_VGA_RENDER_H
#define _MVISOR_DEVICES_VGA_RENDER_H

#include "device.h"
#include "device_interface.h"


class VgaRender {
 private:
  Device*   device_;

  struct {
    uint8_t   misc_output;
    uint8_t   status[2];
    uint8_t   sequencer_index;
    uint8_t   sequencer[256];
    uint8_t   gfx_index;
    uint8_t   gfx[256];
    uint8_t   attribute_index;
    uint8_t   attribute[0x15];
    uint8_t   crtc_index;
    uint8_t   crtc[0x19];
    uint16_t  palette_read_index;
    uint16_t  palette_write_index;
    uint8_t   palette[256 * 3];
    uint8_t   dac_state;
    uint8_t   feature_control;
    uint32_t  latch;
  } vga_;

  struct {
    uint16_t version;
    uint16_t index;
    uint16_t registers[16];
  } vbe_;

  uint32_t    vram_map_addr_;
  uint32_t    vram_map_size_;
  IoResource* vram_rw_resource_ = nullptr;

  uint8_t*    vram_base_ = nullptr;
  uint32_t    vram_size_;

  /* Render mode */
  bool        vbe_mode_;
  int         width_;
  int         height_;
  int         bpp_;
  int         stride_;
  bool        redraw_ = false;

  /* VGA mode */
  int         rows_;
  int         cols_;
  int         font_width_;
  int         font_height_;
  IoTimePoint cursor_blink_time_;
  bool        cursor_visible_;
  bool        text_mode_;
  bool        mode_changed_;

  std::string vga_surface_;
  size_t      vga_display_buffer_size_ = 0;
  std::string vga_display_buffer_;

  MemoryRegion* region_ = nullptr;

  bool IsVbeEnabled();
  void UpdateDisplayMode();
  void UpdateVRamMemoryMap();
  void DrawGraphic(uint8_t* buffer);
  void DrawText(uint8_t* buffer);
  void DrawCharacter(uint8_t* dest, uint8_t* font, int character, int attribute);
  void DrawTextCursor(uint8_t* buffer);
  void GetCursorLocation(uint8_t* x, uint8_t* y, uint8_t* sel_start, uint8_t* sel_end);
  bool GetVbeDisplayUpdate(DisplayUpdate& update);
  bool GetVgaDisplayUpdate(DisplayUpdate& update);

 public:
  VgaRender(Device* device, uint8_t* vram_base, uint32_t vram_size);
  ~VgaRender();
  void SetMemoryRegion(MemoryRegion* region);

  void SaveState(MigrationWriter* writer);
  bool LoadState(MigrationReader* reader);
  void VbeReadPort(uint64_t port, uint16_t* data);
  void VbeWritePort(uint64_t port, uint16_t value);
  void VgaReadPort(uint64_t port, uint8_t* data);
  void VgaWritePort(uint64_t port, uint32_t value);
  void VgaWriteMemory(uint64_t offset, uint32_t value);
  void VgaReadMemory(uint64_t offset, uint8_t* data);
  void GetDisplayMode(int* w, int* h, int* bpp, int* stride);
  void GetPalette(const uint8_t** palette, int* count, bool* dac_8bit);
  bool IsModeChanged();
  bool GetDisplayUpdate(DisplayUpdate& update);
  void Redraw();
};


#define VGA_PIO_BASE    0x3C0
#define VGA_PIO_SIZE    0x20
#define VBE_PIO_BASE    0x1CE
#define VBE_PIO_SIZE    3

// When LFB mode disabled, the tradition VGA video memory address is used
#define VGA_MEMORY_BASE   0x000A0000
#define VGA_MEMORY_SIZE   0x00020000

#define VGA_REFRESH_FREQUENCY 30


#endif // _MVISOR_DEVICES_VGA_RENDER_H
