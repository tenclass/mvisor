/* 
 * MVisor QXL
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


#include <cstring>
#include "vga.h"
#include "logger.h"
#include "spice/qxl_dev.h"
#include "qxl.modes.inc"
#include "machine.h"

#define QXL_ROM_PATH    "../share/vgabios-qxl.bin"

#define NUM_MEMSLOTS 8
#define MEMSLOT_GENERATION_BITS 8
#define MEMSLOT_SLOT_BITS 8

class Qxl : public Vga {
 private:
  bool      qxl_on_;
  uint32_t  qxl_rom_size_;
  void*     qxl_rom_base_;
  uint32_t  qxl_vram32_size_;
  uint8_t*  qxl_vram32_base_;

  QXLRom*   qxl_rom_;
  QXLModes* qxl_modes_;
  QXLRam*   qxl_ram_;

  struct guest_slots {
    QXLMemSlot    slot;
    uint64_t      offset;
    uint64_t      size;
    uint64_t      delta;
    bool          active;
    void*         hva; 
  } guest_slots_[NUM_MEMSLOTS];
  
  struct guest_primary {
    QXLSurfaceCreate surface;
    uint32_t       commands;
    uint32_t       resized;
    int32_t        qxl_stride;
    uint32_t       abs_stride;
    uint32_t       bits_pp;
    uint32_t       bytes_pp;
    uint8_t        *data;
  } guest_primary_;

 public:
  Qxl() {
    pci_header_.vendor_id = 0x1B36;
    pci_header_.device_id = 0x0100;

    AddPciBar(1, 8 << 20, kIoResourceTypeRam);      /* QXL VRAM32 8MB */
    AddPciBar(2, 8192, kIoResourceTypeRam);         /* QXL ROM */
    AddPciBar(3, 32, kIoResourceTypePio);           /* QXL PIO */
    
    /* Bar 1: 8MB */
    qxl_vram32_size_ = 8 << 20;
    qxl_vram32_base_ = (uint8_t*)valloc(qxl_vram32_size_);
    pci_bars_[1].host_memory = qxl_vram32_base_;

    /* Bar 2: 8KB ROM */
    qxl_rom_size_ = 8192;
    qxl_rom_base_ = valloc(qxl_rom_size_);
    pci_bars_[2].host_memory = qxl_rom_base_;
    
  }

  void Connect() {
    PciDevice::Connect();
    LoadRomFile(QXL_ROM_PATH);
  }

  void Reset() {
    Vga::Reset();

    IntializeQxlRom();
    IntializeQxlRam();
    
    qxl_on_ = false;
    // Reset cursor
    // Reset surfaces
    bzero(guest_slots_, sizeof(guest_slots_));
    // Create primary surface
  }

  void IntializeQxlRom() {
    bzero(qxl_rom_base_, qxl_rom_size_);
  
    QXLRom* rom = (QXLRom*)qxl_rom_base_;
    QXLModes* modes = (QXLModes*)(rom + 1);
    rom->magic = QXL_ROM_MAGIC;
    rom->id = 0;
    rom->log_level = 1; /* Guest debug on */
    rom->modes_offset = sizeof(QXLRom);

    rom->slot_gen_bits = MEMSLOT_GENERATION_BITS;
    rom->slot_id_bits = MEMSLOT_SLOT_BITS;
    rom->slots_start = 1;
    rom->slots_end = NUM_MEMSLOTS - 1;
    rom->n_surfaces = 1024;

    uint32_t n = 0;
    for (size_t i = 0; i < sizeof(qxl_modes) / sizeof(qxl_modes[0]); i++) {
      size_t size_needed = qxl_modes[i].y_res * qxl_modes[i].stride;
      if (size_needed > vga_mem_size_) {
        continue;
      }
      modes->modes[n] = qxl_modes[i];
      modes->modes[n].id = i;
      n++;
    }
    modes->n_modes = n;

    uint32_t ram_header_size = SPICE_ALIGN(sizeof(QXLRam), 4096);
    uint32_t surface0_area_size = SPICE_ALIGN(vga_mem_size_, 4096);
    uint32_t num_pages = (vram_size_ - ram_header_size - surface0_area_size) / PAGE_SIZE;
    MV_ASSERT(ram_header_size + surface0_area_size <= vram_size_);

    rom->draw_area_offset = 0;
    rom->surface0_area_size = surface0_area_size;
    rom->pages_offset = surface0_area_size;
    rom->num_pages = num_pages;
    rom->ram_header_offset = vram_size_ - ram_header_size;

    qxl_rom_ = rom;
    qxl_modes_ = modes;
  }

  void IntializeQxlRam() {
    qxl_ram_ = (QXLRam*)(vram_base_ + qxl_rom_->ram_header_offset);
    qxl_ram_->magic = QXL_RAM_MAGIC;
    qxl_ram_->int_pending = 0;
    qxl_ram_->int_mask = 0;
    qxl_ram_->update_surface = 0;
    qxl_ram_->monitors_config = 0;
    SPICE_RING_INIT(&qxl_ram_->cmd_ring);
    SPICE_RING_INIT(&qxl_ram_->cursor_ring);
    SPICE_RING_INIT(&qxl_ram_->release_ring);

    QXLReleaseRing* ring = &qxl_ram_->release_ring;
    uint32_t prod = ring->prod & SPICE_RING_INDEX_MASK(ring);
    MV_ASSERT(prod < sizeof(ring->items) / sizeof(ring->items[0]));
    ring->items[prod].el = 0;
  }

  virtual void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
    if (ir.base == pci_bars_[3].address) {
      switch (offset)
      {
      case QXL_IO_NOTIFY_CMD:
        MV_LOG("notify cmd");
        break;
      case QXL_IO_NOTIFY_CURSOR:
        MV_LOG("notify cursor");
        break;
      case QXL_IO_RESET:
        Reset();
        UpdateIrqLevel();
        break;
      case QXL_IO_MEMSLOT_ADD:
        MV_ASSERT(*data < NUM_MEMSLOTS);
        MV_ASSERT(!guest_slots_[*data].active);
        AddMemorySlot(*data, qxl_ram_->mem_slot);
        break;
      case QXL_IO_CREATE_PRIMARY:
        CreatePrimarySurface(qxl_ram_->create_surface);
        break;
      case QXL_IO_DESTROY_PRIMARY:
        DestroyPrimarySurface();
        break;
      default:
        MV_PANIC("unhandled QXL command=0x%lx data=0x%lx size=0x%x",
          offset, *(uint64_t*)data, size);
        break;
      }
    } else {
      Vga::Write(ir, offset, data, size);
    }
  }

  void AddMemorySlot(int slot_id, QXLMemSlot& slot) {
    int bar_index;
    for (bar_index = 0; bar_index < PCI_BAR_NUMS; bar_index++) {
      if (!pci_bars_[bar_index].active)
        continue;
      if (slot.mem_start >= pci_bars_[bar_index].address &&
        slot.mem_end <= pci_bars_[bar_index].address + pci_bars_[bar_index].size) {
        break;
      }
    }
    if (bar_index == PCI_BAR_NUMS) {
      MV_PANIC("Invalid slot %d 0x%lx-0x%lx", slot_id, slot.mem_start, slot.mem_end);
      return;
    }

    guest_slots_[slot_id].slot = slot;
    guest_slots_[slot_id].offset = slot.mem_start - pci_bars_[bar_index].address;
    guest_slots_[slot_id].size = slot.mem_end - slot.mem_start;
    guest_slots_[slot_id].hva = (uint8_t*)pci_bars_[bar_index].host_memory + guest_slots_[slot_id].offset;
    guest_slots_[slot_id].active = true;
  }

  void CreatePrimarySurface(QXLSurfaceCreate& create) {
    guest_primary_.surface = create;
    guest_primary_.qxl_stride = create.stride;
    guest_primary_.abs_stride = abs(create.stride);
    guest_primary_.resized++;
    switch (create.format)
    {
    case SPICE_SURFACE_FMT_32_xRGB:
    case SPICE_SURFACE_FMT_32_ARGB:
      guest_primary_.bytes_pp = 4;
      guest_primary_.bits_pp = 32;
      break;
    case SPICE_SURFACE_FMT_16_565:
      guest_primary_.bytes_pp = 2;
      guest_primary_.bits_pp = 16;
    default:
      MV_PANIC("unknown surface format=0x%x", create.format);
      break;
    }
    qxl_on_ = true;
  }

  void DestroyPrimarySurface() {
    MV_LOG("DestroyPrimarySurface");
  }

  virtual void GetDisplayMode(DisplayMode *mode, uint16_t* w, uint16_t* h, uint16_t* b) {
    if (qxl_on_) {
      *mode = kDisplayQxlMode;
      *w = guest_primary_.surface.width;
      *h = guest_primary_.surface.height;
      *b = guest_primary_.bits_pp;
    } else {
      Vga::GetDisplayMode(mode, w, h, b);
    }
  }

  void UpdateIrqLevel() {
    int level = !!(qxl_ram_->int_pending & qxl_ram_->int_mask);
    if (pci_header_.irq_line) {
      manager_->SetIrq(pci_header_.irq_line, level);
    }
  }
};

DECLARE_DEVICE(Qxl);
