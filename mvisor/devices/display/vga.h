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

#include <mutex>

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
  struct {
  uint8_t   misc_output;
  uint8_t   status[2];
  uint8_t   sequence_index;
  uint8_t   sequence[256];
  uint8_t   gfx_index;
  uint8_t   gfx[256];
  uint8_t   attribute_index;
  uint8_t   attribute[0x15];
  uint8_t   crtc_index;
  uint8_t   crtc[0x19];
  uint16_t  pallete_read_index;
  uint16_t  pallete_write_index;
  uint8_t   pallete[256 * 3];
  uint8_t   dac_state;
  uint8_t   feature_control_;
  } vga_;

  struct {
    uint16_t version;
    uint16_t index;
    uint16_t registers[16];
  } vbe_;

  uint8_t* vram_map_select_;
  uint32_t vram_map_select_size_;
  uint8_t* vram_read_select_;
  bool     has_mapped_vga_ = false;

  std::vector<DisplayChangeListener> display_change_listerners_;

  void VbeReadPort(uint64_t port, uint16_t* data);
  void VbeWritePort(uint64_t port, uint16_t value);
  void VgaReadPort(uint64_t port, uint8_t* data, uint32_t size);
  void VgaWritePort(uint64_t port, uint8_t* data, uint32_t size);
  void UpdateVRamMemoryMap();
  void RenderTextMode();
  void RenderGraphicsMode();
  void DrawCharacter(uint8_t* buffer, uint stride, uint x, uint y, int character, int attribute);
  void DrawTextCursor(uint8_t* buffer, uint stride);
  void GetCursorLocation(uint8_t* x, uint8_t* y, uint8_t* sel_start, uint8_t* sel_end);

 protected:
  uint32_t    vram_size_;
  uint8_t*    vram_base_ = nullptr;
  uint32_t    vga_mem_size_;
  DisplayMode mode_;
  uint        width_;
  uint        height_;
  uint        bpp_;
  uint        stride_;
  std::string vga_surface_;
  std::recursive_mutex      mutex_;
  std::string default_rom_path_;

  void NotifyDisplayModeChange();
  virtual void UpdateDisplayMode();

 public:
  Vga();
  virtual ~Vga();

  virtual void Reset();
  virtual void Connect();
  virtual void Disconnect();
  virtual void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);
  
  virtual bool SaveState(MigrationWriter* writer);
  virtual bool LoadState(MigrationReader* reader);

  virtual void GetDisplayMode(uint* w, uint* h, uint* bpp, uint* stride);

  const uint8_t* GetPallete() const;
  void RegisterDisplayChangeListener(DisplayChangeListener callback);
  virtual bool AcquireUpdate(DisplayUpdate& update);
  virtual void ReleaseUpdate();
};

#endif // _MVISOR_DEVICES_VGA_H
