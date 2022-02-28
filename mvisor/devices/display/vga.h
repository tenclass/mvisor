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

#ifndef _MVISOR_DEVICES_VGA_H
#define _MVISOR_DEVICES_VGA_H

#include "pci_device.h"
#include "device_interface.h"
#include "device_manager.h"

enum DisplayMode {
  kDisplayUnknownMode,
  kDisplayTextMode,
  kDisplayVgaMode,
  kDisplayVbeMode,
  kDisplayQxlMode
};

class Vga : public PciDevice, public DisplayInterface {
 private:
  uint8_t misc_output_reg_;
  uint8_t sequence_index_;
  uint8_t sequence_registers_[256];
  uint8_t gfx_index_;
  uint8_t gfx_registers_[256];
  uint8_t attribute_index_;
  uint8_t attribute_registers_[0x15];
  uint16_t pallete_read_index_;
  uint16_t pallete_write_index_;
  uint8_t pallete_[256 * 3];
  uint8_t crtc_index_;
  uint8_t crtc_registers_[0x19];
  uint8_t status_registers_[2];

  uint16_t vbe_version_;
  uint16_t vbe_index_;
  uint16_t vbe_registers_[16];

  uint8_t* vram_map_select_;
  uint32_t vram_map_select_size_;
  uint8_t* vram_read_select_;

  std::vector<DisplayChangeListener> display_change_listerners_;
  std::vector<DisplayRenderCallback> display_render_callbacks_;
  std::vector<DisplayCursorUpdateCallback> display_cursor_callbacks_;
  IoTimer*                           refresh_timer_;

  void VbeReadPort(uint64_t port, uint16_t* data);
  void VbeWritePort(uint64_t port, uint16_t value);
  void VgaReadPort(uint64_t port, uint8_t* data, uint32_t size);
  void VgaWritePort(uint64_t port, uint8_t* data, uint32_t size);
  void UpdateVRamMemoryMap();
  void RenderTextMode();
  void RenderGraphicsMode();
  void DrawCharacter(uint8_t* buffer, int stride, int x, int y, int character, int attribute);
  void DrawTextCursor(uint8_t* buffer, int stride);
  void GetCursorLocation(uint8_t* x, uint8_t* y, uint8_t* sel_start, uint8_t* sel_end);

 protected:
  uint32_t  vram_size_;
  uint8_t*  vram_base_;
  uint32_t  vga_mem_size_;
  DisplayMode mode_;
  uint16_t    width_;
  uint16_t    height_;
  uint16_t    bpp_;

  void NotifyDisplayModeChange();
  void NotifyDisplayRender(DisplayPartialBitmap* partial);
  void NotifyDisplayCursorUpdate(DisplayCursorUpdate* update);
  void OnRefreshTimer();
  void UpdateDisplayMode();

 public:
  Vga();
  virtual ~Vga();

  virtual void Reset();
  virtual void Connect();
  virtual void Disconnect();
  virtual void Read(const IoResource* ir, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void Write(const IoResource* ir, uint64_t offset, uint8_t* data, uint32_t size);

  virtual void GetDisplayMode(uint16_t* w, uint16_t* h, uint16_t* bpp);

  const uint8_t* GetPallete() const;
  void RegisterDisplayChangeListener(DisplayChangeListener callback);
  virtual void RegisterDisplayRenderer(DisplayRenderCallback draw_callback,
    DisplayCursorUpdateCallback cursor_callback);
};

#endif // _MVISOR_DEVICES_VGA_H
