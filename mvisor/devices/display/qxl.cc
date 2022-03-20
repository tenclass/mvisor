/* 
 * MVisor QXL
 * Currently only the latest windows 8/10 guest DOD driver is tested
 * 
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
#include <vector>

#include "vga.h"
#include "logger.h"
#include "spice/qxl_dev.h"
#include "qxl.modes.inc"
#include "machine.h"
#include "states/qxl.pb.h"

#define NUM_MEMSLOTS 8
#define MEMSLOT_GENERATION_BITS 8
#define MEMSLOT_SLOT_BITS 8

class Qxl : public Vga {
 private:
  uint32_t  qxl_rom_size_;
  void*     qxl_rom_base_;
  uint32_t  qxl_vram32_size_;
  uint8_t*  qxl_vram32_base_;

  QXLRom*   qxl_rom_;
  QXLModes* qxl_modes_;
  QXLRam*   qxl_ram_;
  QXLReleaseInfo* last_relealse_info_;

  struct guest_slots {
    QXLMemSlot    slot;
    uint64_t      offset;
    bool          active;
    uint8_t*      hva; 
  } guest_slots_[NUM_MEMSLOTS];
  
  struct guest_primary {
    QXLSurfaceCreate  surface;
    uint32_t          resized;
    int32_t           qxl_stride;
    uint32_t          abs_stride;
    uint32_t          bits_pp;
    uint32_t          bytes_pp;
  } guest_primary_;

 public:
  Qxl() {
    pci_header_.vendor_id = 0x1B36;
    pci_header_.device_id = 0x0100;
    pci_header_.revision_id = 5;
    
    /* Bar 1: 8MB, not used in Windows driver */
    qxl_vram32_size_ = _MB(8);
    qxl_vram32_base_ = (uint8_t*)valloc(qxl_vram32_size_);
    pci_bars_[1].host_memory = qxl_vram32_base_;

    /* Bar 2: 8KB ROM */
    qxl_rom_size_ = 8192;
    qxl_rom_base_ = valloc(qxl_rom_size_);
    pci_bars_[2].host_memory = qxl_rom_base_;

    AddPciBar(1, qxl_vram32_size_, kIoResourceTypeRam); /* QXL VRAM32 8MB */
    AddPciBar(2, 8192, kIoResourceTypeRam);             /* QXL ROM */
    AddPciBar(3, 32, kIoResourceTypePio);               /* QXL PIO */
    
  }

  virtual ~Qxl() {
    if (qxl_vram32_base_) {
      free(qxl_vram32_base_);
    }
  }

  virtual bool ActivatePciBar(uint8_t index) {
    if (index == 3) {
      /* Setup ioeventfd for notify commands */
      manager_->RegisterIoEvent(this, kIoResourceTypePio, pci_bars_[index].address + QXL_IO_NOTIFY_CMD);
      manager_->RegisterIoEvent(this, kIoResourceTypePio, pci_bars_[index].address + QXL_IO_NOTIFY_CURSOR);
    }
    return Vga::ActivatePciBar(index);
  }

  virtual bool DeactivatePciBar(uint8_t index) {
    if (index == 3) {
      manager_->UnregisterIoEvent(this, kIoResourceTypePio, pci_bars_[index].address + QXL_IO_NOTIFY_CMD);
      manager_->UnregisterIoEvent(this, kIoResourceTypePio, pci_bars_[index].address + QXL_IO_NOTIFY_CURSOR);
    }
    return Vga::DeactivatePciBar(index);
  }

  virtual void Reset() {
    /* Vga Reset() resets mode */
    Vga::Reset();

    IntializeQxlRom();
    IntializeQxlRam();

    // Reset cursor
    // Reset surfaces
    bzero(guest_slots_, sizeof(guest_slots_));
    bzero(&guest_primary_, sizeof(guest_primary_));
  }

  virtual bool SaveState(MigrationWriter* writer) {
    QxlState state;
    for (int i = 0; i < NUM_MEMSLOTS; i++) {
      auto slot = state.add_guest_slots();
      slot->set_mem_start(guest_slots_[i].slot.mem_start);
      slot->set_mem_end(guest_slots_[i].slot.mem_end);
      slot->set_active(guest_slots_[i].active);
    }
    state.set_last_release_offset((uint64_t)last_relealse_info_ - (uint64_t)vram_base_);
  
    auto primary = state.mutable_guest_primary();
    auto& surface = guest_primary_.surface;
    primary->set_width(surface.width);
    primary->set_height(surface.height);
    primary->set_stride(surface.stride);
    primary->set_format(surface.format);
    primary->set_position(surface.position);
    primary->set_mouse_mode(surface.mouse_mode);
    primary->set_flags(surface.flags);
    primary->set_type(surface.type);
    primary->set_mem_address(surface.mem);
    writer->WriteProtobuf("QXL", state);
    return Vga::SaveState(writer);
  }

  /* Reset should be called before load state */
  virtual bool LoadState(MigrationReader* reader) {
    if (!Vga::LoadState(reader)) {
      return false;
    }
    QxlState state;
    if (!reader->ReadProtobuf("QXL", state)) {
      return false;
    }
    for (int i = 0; i < NUM_MEMSLOTS && i < state.guest_slots_size(); i++) {
      auto slot = state.guest_slots(i);
      if (slot.active()) {
        QXLMemSlot mem_slot = {
          .mem_start = slot.mem_start(),
          .mem_end = slot.mem_end()
        };
        AddMemSlot(i, mem_slot);
      }
    }
    last_relealse_info_ = (QXLReleaseInfo*)(vram_base_ + state.last_release_offset());

    auto& primary = state.guest_primary();
    QXLSurfaceCreate create = {
      .width = primary.width(),
      .height = primary.height(),
      .stride = primary.stride(),
      .format = primary.format(),
      .position = primary.position(),
      .mouse_mode = primary.mouse_mode(),
      .flags = primary.flags(),
      .type = primary.type(),
      .mem = primary.mem_address()
    };
    if (create.width && create.height) {
      CreatePrimarySurface(create);
    }
    return true;
  }

  void IntializeQxlRom() {
    bzero(qxl_rom_base_, qxl_rom_size_);
  
    QXLRom* rom = (QXLRom*)qxl_rom_base_;
    QXLModes* modes = (QXLModes*)(rom + 1);
    rom->magic = QXL_ROM_MAGIC;
    rom->id = 0;
    rom->log_level = 0; /* Guest debug */
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

  uint64_t TranslateAsyncCommand(uint64_t command, bool* async) {
    *async = true;
    switch (command)
    {
    case QXL_IO_UPDATE_AREA_ASYNC:
      command = QXL_IO_UPDATE_AREA;
      break;
    case QXL_IO_MEMSLOT_ADD_ASYNC:
      command = QXL_IO_MEMSLOT_ADD;
      break;
    case QXL_IO_CREATE_PRIMARY_ASYNC:
      command = QXL_IO_CREATE_PRIMARY;
      break;
    case QXL_IO_DESTROY_PRIMARY_ASYNC:
      command = QXL_IO_DESTROY_PRIMARY;
      break;
    case QXL_IO_DESTROY_SURFACE_ASYNC:
      command = QXL_IO_DESTROY_SURFACE_WAIT;
      break;
    case QXL_IO_DESTROY_ALL_SURFACES_ASYNC:
      command = QXL_IO_DESTROY_ALL_SURFACES;
      break;
    case QXL_IO_FLUSH_SURFACES_ASYNC:
    case QXL_IO_MONITORS_CONFIG_ASYNC:
      break;
    default:
      *async = false;
      break;
    }
    return command;
  }

  virtual void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    if (resource->base == pci_bars_[3].address) {
      bool async;
      uint64_t command = TranslateAsyncCommand(offset, &async);
      switch (command)
      {
      case QXL_IO_NOTIFY_CMD:
        FetchGraphicsCommands();
        break;
      case QXL_IO_NOTIFY_CURSOR:
        FetchCursorCommands();
        break;
      case QXL_IO_RESET:
        Reset();
        UpdateIrqLevel();
        break;
      case QXL_IO_MEMSLOT_ADD:
        MV_ASSERT(*data < NUM_MEMSLOTS);
        MV_ASSERT(!guest_slots_[*data].active);
        AddMemSlot(*data, qxl_ram_->mem_slot);
        break;
      case QXL_IO_CREATE_PRIMARY:
      case QXL_IO_CREATE_PRIMARY_ASYNC:
        CreatePrimarySurface(qxl_ram_->create_surface);
        break;
      case QXL_IO_DESTROY_PRIMARY:
        DestroyPrimarySurface();
        break;
      case QXL_IO_NOTIFY_OOM:
        MV_PANIC("QXL guest OOM");
        break;
      case QXL_IO_UPDATE_IRQ:
        UpdateIrqLevel();
        break;
      default:
        MV_PANIC("unhandled QXL command=0x%lx data=0x%lx size=0x%x",
          offset, *(uint64_t*)data, size);
        break;
      }
      if (async) {
        if (debug_) {
          MV_LOG("complete cmd=0x%x", offset);
        }
        SetInterrupt(QXL_INTERRUPT_IO_CMD);
      }
    } else {
      Vga::Write(resource, offset, data, size);
    }
  }

  void AddMemSlot(int slot_id, QXLMemSlot& slot) {
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
    default:
      MV_PANIC("unsupported surface format=0x%x", create.format);
      break;
    }
    bool changed = (mode_ != kDisplayQxlMode) || (width_ != create.width) ||
      (height_ != create.height) || (bpp_ != guest_primary_.bits_pp);
    if (changed) {
      UpdateDisplayMode();
    }
  }

  void DestroyPrimarySurface() {
    /* Maybe we should notify the viewer??? */
    bzero(&guest_primary_, sizeof(guest_primary_));
  }

  virtual void UpdateDisplayMode() {
    if (guest_primary_.resized) {
      mode_ = kDisplayQxlMode;
      width_ = guest_primary_.surface.width;
      height_ = guest_primary_.surface.height;
      bpp_ = guest_primary_.bits_pp;
      NotifyDisplayModeChange();
    } else {
      Vga::UpdateDisplayMode();
    }
  }

  virtual void GetDisplayMode(uint16_t* w, uint16_t* h, uint16_t* bpp) {
    if (mode_ == kDisplayQxlMode) {
      *w = guest_primary_.surface.width;
      *h = guest_primary_.surface.height;
      *bpp = guest_primary_.bits_pp;
    } else {
      Vga::GetDisplayMode(w, h, bpp);
    }
  }

  void SetInterrupt(uint32_t interrupt) {
    qxl_ram_->int_pending |= interrupt;
    UpdateIrqLevel();
  }

  void UpdateIrqLevel() {
    int level = !!(qxl_ram_->int_pending & qxl_ram_->int_mask);
    if (pci_header_.irq_line) {
      manager_->SetIrq(pci_header_.irq_line, level);
    }
  }

  void* GetMemSlotAddress(uint64_t data) {
    uint64_t slot_id = data >> (64 - qxl_rom_->slot_id_bits);
    MV_ASSERT(slot_id < NUM_MEMSLOTS);
  
    uint64_t generation_mask = ((1UL << qxl_rom_->slot_gen_bits) - 1);
    uint64_t generation_shift = 64 - qxl_rom_->slot_id_bits - qxl_rom_->slot_gen_bits;
    uint64_t generation = (data >> generation_shift) & generation_mask;
    MV_ASSERT(generation == qxl_rom_->slot_generation);

    uint64_t virtual_mask = (1UL << generation_shift) - 1;
    return guest_slots_[slot_id].hva + (data & virtual_mask);
  }

  void GetMemSlotChunkedData(uint64_t data, std::vector<DisplayPartialData>& vector) {
    while (data) {
      QXLDataChunk* chunk = (QXLDataChunk*)GetMemSlotAddress(data);
      vector.emplace_back(DisplayPartialData {
        .data = chunk->data,
        .size = chunk->data_size
      });
      data = chunk->next_chunk;
    }
  }

  void GetMemSlotLinearizedData(QXLDataChunk* chunk, uint8_t** data, size_t* size) {
    std::vector<QXLDataChunk*> chunks;
    *size = 0;
    while (chunk) {
      *size += chunk->data_size;
      chunks.push_back(chunk);
      if (chunk->next_chunk) {
        chunk = (QXLDataChunk*)GetMemSlotAddress(chunk->next_chunk);
      } else {
        chunk = nullptr;
      }
    }
    *data = new uint8_t[*size];
    uint8_t* ptr = *data;
    for (auto chunk : chunks) {
      memcpy(ptr, chunk->data, chunk->data_size);
      ptr += chunk->data_size;
    }
  }

  void ReleaseGuestResource(QXLReleaseInfo* info) {
    if (mode_ != kDisplayQxlMode) {
      return;
    }
    QXLReleaseRing* ring = &qxl_ram_->release_ring;
    uint32_t prod = ring->prod & SPICE_RING_INDEX_MASK(ring);

    /* Careful! Union member info->id and info->next have same guest address */
    auto id = info->id;
    info->next = 0;
    if (ring->items[prod].el == 0) {
      ring->items[prod].el = id;
    } else {
      last_relealse_info_->next = id;
    }
    last_relealse_info_ = info;

    if (ring->prod - ring->cons + 1 == ring->num_items) {
      /* ring full, cannot push */
      return;
    }
    bool should_notify;
    SPICE_RING_PUSH(ring, should_notify);
    if (should_notify) {
      SetInterrupt(QXL_INTERRUPT_DISPLAY);
    }

    ring->items[ring->prod & SPICE_RING_INDEX_MASK(ring)].el = 0;
    last_relealse_info_ = nullptr;
  }

  void FetchGraphicsCommands() {
    QXLCommandRing* ring = &qxl_ram_->cmd_ring;
    while (!SPICE_RING_IS_EMPTY(ring)) {
      QXLCommand command = *SPICE_RING_CONS_ITEM(ring);
      bool should_notify;
      SPICE_RING_POP(ring, should_notify);
      if (should_notify) {
        SetInterrupt(QXL_INTERRUPT_DISPLAY);
      }
      ParseCommand(command);
      bool should_wait;
      SPICE_RING_CONS_WAIT(ring, should_wait);
    }
  }

  void FetchCursorCommands() {
    QXLCursorRing* ring = &qxl_ram_->cursor_ring;
    while (!SPICE_RING_IS_EMPTY(ring)) {
      QXLCommand command = *SPICE_RING_CONS_ITEM(ring);
      bool should_notify;
      SPICE_RING_POP(ring, should_notify);
      if (should_notify) {
        SetInterrupt(QXL_INTERRUPT_CURSOR);
      }
      ParseCommand(command);
      bool should_wait;
      SPICE_RING_CONS_WAIT(ring, should_wait);
    }
  }

  void ParseCommand(QXLCommand& command) {
    if (mode_ != kDisplayQxlMode) {
      return;
    }
    switch (command.type)
    {
    case QXL_CMD_DRAW: {
      QXLDrawable* drawable = (QXLDrawable*)GetMemSlotAddress(command.data);
      ParseDrawable(drawable);
      break;
    }
    case QXL_CMD_CURSOR: {
      QXLCursorCmd* cursor = (QXLCursorCmd*)GetMemSlotAddress(command.data);
      ParseCursor(cursor);
      break;
    }
    default:
      DumpHex(&command, sizeof(command));
      MV_PANIC("unhandled command type=0x%x data=0x%lx", command.type, command.data);
      break;
    }
  }

  void ParseDrawable(QXLDrawable* drawable) {
    DisplayPartialBitmap* partial = new DisplayPartialBitmap {
      .width = drawable->bbox.right - drawable->bbox.left,
      .height = drawable->bbox.bottom - drawable->bbox.top,
      .x = drawable->bbox.left,
      .y = drawable->bbox.top
    };
    MV_ASSERT(drawable->bbox.left >= 0 && drawable->bbox.top >= 0);
    if (drawable->bbox.right > (int32_t)guest_primary_.surface.width ||
      drawable->bbox.bottom > (int32_t)guest_primary_.surface.height) {
      MV_LOG("Invalid draw box %d-%d %d-%d surface %ux%u",
        drawable->bbox.left, drawable->bbox.right, drawable->bbox.top, drawable->bbox.bottom,
        guest_primary_.surface.width, guest_primary_.surface.height);
      return;
    }
  
    switch (drawable->type)
    {
    case QXL_DRAW_COPY: {
      QXLCopy* copy = &drawable->u.copy;
      MV_ASSERT(drawable->effect == QXL_EFFECT_OPAQUE);
      MV_ASSERT(drawable->clip.type == SPICE_CLIP_TYPE_NONE);
      if (drawable->self_bitmap) {
        MV_ASSERT(drawable->self_bitmap_area.left == drawable->bbox.left && drawable->self_bitmap_area.right == drawable->bbox.right);
      }
      MV_ASSERT(copy->src_area.top == 0 && copy->src_area.left == 0);
      MV_ASSERT(copy->rop_descriptor == SPICE_ROPD_OP_PUT);
      QXLImage* image = (QXLImage*)GetMemSlotAddress(copy->src_bitmap);
      MV_ASSERT(image->descriptor.type == SPICE_IMAGE_TYPE_BITMAP);
      MV_ASSERT(image->descriptor.flags == 0);

      QXLBitmap* bitmap = &image->bitmap;
      if (bitmap->format != SPICE_BITMAP_FMT_RGBA) {
        MV_LOG("invalid bitmap format=0x%x", bitmap->format);
        ReleaseGuestResource(&drawable->release_info);
        delete partial;
        break;
      }
      MV_ASSERT(bitmap->palette == 0);
      MV_ASSERT(bitmap->stride == bitmap->x * guest_primary_.bytes_pp);
      MV_ASSERT(partial->width == (int)bitmap->x && partial->height == (int)bitmap->y);
      partial->stride = bitmap->stride;
      partial->flip = !(bitmap->flags & QXL_BITMAP_TOP_DOWN);
      GetMemSlotChunkedData(bitmap->data, partial->vector);
      partial->Release = [=]() {
        ReleaseGuestResource(&drawable->release_info);
        delete partial;
      };
      NotifyDisplayRender(partial);
      break;
    }
    case QXL_DRAW_FILL: {
      QXLFill* fill = &drawable->u.fill;
      MV_ASSERT(fill->rop_descriptor == SPICE_ROPD_OP_PUT);
      QXLBrush* brush = &fill->brush;
      MV_ASSERT(brush->type == SPICE_BRUSH_TYPE_SOLID);
      uint32_t color = brush->u.color;
      partial->stride = partial->width * guest_primary_.bytes_pp;
      size_t size = partial->stride * partial->height;
      uint8_t* data = new uint8_t[size];
      memset(data, color, size);
      partial->vector.emplace_back(DisplayPartialData {
        .data = data,
        .size = size
      });
      partial->Release = [=]() {
        ReleaseGuestResource(&drawable->release_info);
        delete data;
        delete partial;
      };
      NotifyDisplayRender(partial);
      break;
    }
    default:
      DumpHex(drawable, sizeof(*drawable));
      MV_PANIC("unhandled drawable type=%d", drawable->type);
      break;
    }
  }

  void ParseCursor(QXLCursorCmd* cursor) {
    DisplayCursorUpdate* update = new DisplayCursorUpdate;
    switch (cursor->type)
    {
    case QXL_CURSOR_HIDE:
      update->command = kDisplayCursorUpdateHide;
      update->Release = [=]() {
        ReleaseGuestResource(&cursor->release_info);
        delete update;
      };
      break;
    case QXL_CURSOR_MOVE:
      update->command = kDisplayCursorUpdateMove;
      update->move.x = cursor->u.position.x;
      update->move.y = cursor->u.position.y;
      update->Release = [=]() {
        ReleaseGuestResource(&cursor->release_info);
        delete update;
      };
      break;
    case QXL_CURSOR_SET: {
      QXLCursor* shape = (QXLCursor*)GetMemSlotAddress(cursor->u.set.shape);
      update->command = kDisplayCursorUpdateSet;
      update->set.visible = cursor->u.set.visible;
      update->set.x = cursor->u.set.position.x;
      update->set.y = cursor->u.set.position.y;

      update->set.type = shape->header.type;
      update->set.width = shape->header.width;
      update->set.height = shape->header.height;
      update->set.hotspot_x = shape->header.hot_spot_x;
      update->set.hotspot_y = shape->header.hot_spot_y;
      GetMemSlotLinearizedData(&shape->chunk, &update->set.data, &update->set.size);
      update->Release = [=]() {
        ReleaseGuestResource(&cursor->release_info);
        delete[] update->set.data;
        delete update;
      };
      break;
    }
    default:
      DumpHex(cursor, sizeof(*cursor));
      MV_PANIC("unhandled cursor type=%d", cursor->type);
      break;
    }
    NotifyDisplayCursorUpdate(update);
  }
};

DECLARE_DEVICE(Qxl);
