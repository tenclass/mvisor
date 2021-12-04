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

  uint16_t width_;
  uint16_t height_;
  uint16_t bpp_;

  std::vector<DisplayChangeListener> display_change_listerners_;

  void VbeReadPort(uint64_t port, uint16_t* data);
  void VbeWritePort(uint64_t port, uint16_t value);
  void VgaReadPort(uint64_t port, uint8_t* data, uint32_t size);
  void VgaWritePort(uint64_t port, uint8_t* data, uint32_t size);
  void UpdateVRamMemoryMap();

 protected:
  uint32_t  vram_size_;
  uint8_t*  vram_base_;
  uint32_t  vga_mem_size_;

  void UpdateRenderer();

 public:
  Vga();
  virtual ~Vga();

  virtual void Reset();
  virtual void Connect();
  virtual void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);

  virtual void GetDisplayMode(DisplayMode *mode, uint16_t* w, uint16_t* h, uint16_t* b);
  virtual uint8_t* GetVRamHostAddress();

  const uint8_t* GetPallete() const;
  void GetCursorLocation(uint8_t* x, uint8_t* y, uint8_t* sel_start, uint8_t* sel_end);
  void RegisterDisplayChangeListener(DisplayChangeListener callback);
};

#endif // _MVISOR_DEVICES_VGA_H
