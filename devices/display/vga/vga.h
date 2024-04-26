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

#include "vga_render.h"
#include "pci_device.h"
#include "../display.h"


class Vga : public PciDevice, public Display {
 private:
  VgaRender*  vga_render_ = nullptr;

 protected:
  std::string default_rom_path_;
  uint32_t    vram_size_;
  uint8_t*    vram_base_ = nullptr;
  IoTimer*    refresh_timer_ = nullptr;

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

  /* DisplayInterface */
  virtual void GetDisplayMode(int* w, int* h, int* bpp, int* stride);
  virtual void GetPalette(const uint8_t** palette, int* count);
  virtual bool AcquireUpdate(DisplayUpdate& update, bool redraw);
  virtual void ReleaseUpdate();
};

#endif // _MVISOR_DEVICES_VGA_H
