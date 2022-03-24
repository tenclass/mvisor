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
#include <map>
#include <mutex>

#include <zlib.h>

#include "vga.h"
#include "logger.h"
#include "spice/qxl_dev.h"
#include "qxl.modes.inc"
#include "machine.h"
#include "states/qxl.pb.h"

#define NUM_MEMSLOTS 8
#define MEMSLOT_GENERATION_BITS 8
#define MEMSLOT_SLOT_BITS 8

struct Surface {
  uint            id;
  QXLReleaseInfo* release_info;
};
  
struct PrimarySurface {
  QXLSurfaceCreate  surface;
  uint32_t          resized;
  int32_t           qxl_stride;
  uint32_t          abs_stride;
  uint32_t          bits_pp;
  uint32_t          bytes_pp;
};

class Qxl : public Vga, public DisplayResizeInterface {
 private:
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
    bool          active;
    uint8_t*      hva; 
  } guest_slots_[NUM_MEMSLOTS];

  PrimarySurface*           primary_surface_ = nullptr;
  std::map<uint, Surface*>  surfaces_;
  std::recursive_mutex      mutex_;
  std::vector<QXLReleaseInfo*> free_resources_;

 public:
  Qxl() {
    pci_header_.vendor_id = 0x1B36;
    pci_header_.device_id = 0x0100;
    pci_header_.revision_id = 5;
    pci_header_.irq_pin = 1;
    
    /* Bar 1: 8MB, not used in Windows driver, but Linux driver 
     * https://www.spice-space.org/multiple-monitors.html
     */
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

  virtual void Reset() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    /* Vga Reset() resets mode */
    Vga::Reset();

    IntializeQxlRom();
    IntializeQxlRam();

    bzero(guest_slots_, sizeof(guest_slots_));
    // Reset cursor
    // Reset surfaces
    if (primary_surface_) {
      DestroyPrimarySurface();
    }
    for (auto it = surfaces_.begin(); it != surfaces_.end(); it++) {
      delete it->second;
    }
    surfaces_.clear();
    free_resources_.clear();
  }

  virtual bool SaveState(MigrationWriter* writer) {
    QxlState state;
    for (int i = 0; i < NUM_MEMSLOTS; i++) {
      auto slot = state.add_guest_slots();
      slot->set_mem_start(guest_slots_[i].slot.mem_start);
      slot->set_mem_end(guest_slots_[i].slot.mem_end);
      slot->set_active(guest_slots_[i].active);
    }
  
    if (primary_surface_) {
      auto primary = state.mutable_guest_primary();
      auto& surface = primary_surface_->surface;
      primary->set_width(surface.width);
      primary->set_height(surface.height);
      primary->set_stride(surface.stride);
      primary->set_format(surface.format);
      primary->set_position(surface.position);
      primary->set_mouse_mode(surface.mouse_mode);
      primary->set_flags(surface.flags);
      primary->set_type(surface.type);
      primary->set_mem_address(surface.mem);
    }
    for (auto info : free_resources_) {
      state.add_free_resources((uint64_t)info - (uint64_t)vram_base_);
    }
    writer->WriteProtobuf("QXL", state);
    writer->WriteRaw("VRAM32", qxl_vram32_base_, qxl_vram32_size_);
    return Vga::SaveState(writer);
  }

  /* Reset should be called before load state */
  virtual bool LoadState(MigrationReader* reader) {
    if (!Vga::LoadState(reader)) {
      return false;
    }
    if (!reader->ReadRaw("VRAM32", qxl_vram32_base_, qxl_vram32_size_)) {
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

    auto& primary = state.guest_primary();
    if (primary.width() && primary.height()) {
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
      CreatePrimarySurface(create);
    }

    for (int i = 0; i < state.free_resources_size(); i++) {
      auto info = (QXLReleaseInfo*)((uint64_t)vram_base_ + state.free_resources(i));
      free_resources_.push_back(info);
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
  }

  virtual void OnRefreshTimer() {
    if (mode_ == kDisplayQxlMode) {
      if (primary_surface_) {
        FetchGraphicsCommands();
        FetchCursorCommands();
      }
    } else {
      Vga::OnRefreshTimer();
    }
  }

  virtual void UpdateDisplayMode() {
    if (primary_surface_ && primary_surface_->resized) {
      mode_ = kDisplayQxlMode;
      width_ = primary_surface_->surface.width;
      height_ = primary_surface_->surface.height;
      bpp_ = primary_surface_->bits_pp;
      stride_ = primary_surface_->abs_stride;
      NotifyDisplayModeChange();
    } else {
      Vga::UpdateDisplayMode();
    }
  }

  virtual void GetDisplayMode(uint16_t* w, uint16_t* h, uint16_t* bpp, uint16_t* stride) {
    if (mode_ == kDisplayQxlMode) {
      *w = primary_surface_->surface.width;
      *h = primary_surface_->surface.height;
      *bpp = primary_surface_->bits_pp;
      *stride = primary_surface_->abs_stride;
    } else {
      Vga::GetDisplayMode(w, h, bpp);
    }
  }

  void SetInterrupt(uint32_t interrupt) {
    auto old_pending = __atomic_fetch_or(&qxl_ram_->int_pending, interrupt, __ATOMIC_SEQ_CST);
    if (old_pending & interrupt)
      return;
    UpdateIrqLevel();
  }

  void UpdateIrqLevel() {
    int level = !!(qxl_ram_->int_pending & qxl_ram_->int_mask);
    SetIrq(level);
  }

  /* Display resize interface */
  virtual bool Resize(uint width, uint height) {
    if (!(qxl_ram_->int_mask & QXL_INTERRUPT_CLIENT_MONITORS_CONFIG)) {
      return false;
    }

    if (!width || !height) {
      return true;
    }

    bzero(&qxl_rom_->client_monitors_config, sizeof(qxl_rom_->client_monitors_config));
    qxl_rom_->client_monitors_config.count = 1;
    auto& head = qxl_rom_->client_monitors_config.heads[0];
    head.left = 0;
    head.top = 0;
    head.right = head.left + width;
    head.bottom = head.top + height;
    qxl_rom_->client_monitors_config_crc = crc32(0xFFFFFFFF,
      (uint8_t*)&qxl_rom_->client_monitors_config, sizeof(qxl_rom_->client_monitors_config)) ^ 0xFFFFFFFF;
    
    SetInterrupt(QXL_INTERRUPT_CLIENT_MONITORS_CONFIG);
    return true;
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
        manager_->io()->Schedule([this]() {
          FetchGraphicsCommands();
        });
        break;
      case QXL_IO_NOTIFY_CURSOR:
        manager_->io()->Schedule([this]() {
          FetchCursorCommands();
        });
        break;
      case QXL_IO_UPDATE_AREA:
        // MV_LOG("QXL_IO_UPDATE_AREA not implemented");
        break;
      case QXL_IO_RESET:
        Reset();
        UpdateIrqLevel();
        break;
      case QXL_IO_MEMSLOT_ADD:
        MV_ASSERT(data[0] < NUM_MEMSLOTS);
        MV_ASSERT(!guest_slots_[data[0]].active);
        AddMemSlot(data[0], qxl_ram_->mem_slot);
        break;
      case QXL_IO_CREATE_PRIMARY:
        CreatePrimarySurface(qxl_ram_->create_surface);
        break;
      case QXL_IO_DESTROY_PRIMARY:
        FetchGraphicsCommands();
        FetchCursorCommands();
        DestroyPrimarySurface();
        break;
      case QXL_IO_NOTIFY_OOM:
        // MV_LOG("QXL guest OOM");
        FetchGraphicsCommands();
        FetchCursorCommands();
        FreeGuestResources();
        break;
      case QXL_IO_UPDATE_IRQ:
        UpdateIrqLevel();
        break;
      case QXL_IO_MONITORS_CONFIG_ASYNC:
        break;
      default:
        MV_PANIC("unhandled QXL command=0x%lx", command);
        break;
      }
      if (async) {
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
    if (debug_) {
      MV_LOG("add QXL memslot %d bar=%d 0x%lx-0x%lx", slot_id, bar_index, slot.mem_start, slot.mem_end);
    }
  }

  void CreatePrimarySurface(QXLSurfaceCreate& create) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
  
    MV_ASSERT(primary_surface_ == nullptr);
    primary_surface_= new PrimarySurface;
    primary_surface_->surface = create;
    primary_surface_->qxl_stride = create.stride;
    primary_surface_->abs_stride = abs(create.stride);
    primary_surface_->resized++;
    switch (create.format)
    {
    case SPICE_SURFACE_FMT_32_xRGB:
    case SPICE_SURFACE_FMT_32_ARGB:
      primary_surface_->bytes_pp = 4;
      primary_surface_->bits_pp = 32;
      break;
    default:
      MV_PANIC("unsupported surface format=0x%x", create.format);
      break;
    }
    if (debug_) {
      MV_LOG("create primary %dx%d", create.width, create.height);
    }
    bool changed = (mode_ != kDisplayQxlMode) || (width_ != create.width) ||
      (height_ != create.height) || (bpp_ != primary_surface_->bits_pp);
    if (changed) {
      UpdateDisplayMode();
    }
  }

  void DestroyPrimarySurface() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
  
    /* Maybe we should notify the viewer??? */
    delete primary_surface_;
    primary_surface_ = nullptr;
    mode_ = kDisplayUnknownMode;
    if (debug_)
      MV_LOG("destroy primary");
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
    MV_ASSERT(*size);
    *data = new uint8_t[*size];
    uint8_t* ptr = *data;
    for (auto chunk : chunks) {
      memcpy(ptr, chunk->data, chunk->data_size);
      ptr += chunk->data_size;
    }
  }

  void FreeGuestResources() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (free_resources_.empty()) {
      return;
    }
  
    QXLReleaseRing* ring = &qxl_ram_->release_ring;
    if (SPICE_RING_IS_FULL(ring)) {
      /* ring full, cannot push */
      MV_LOG("ring is full, failed to push item");
      return;
    }
    auto *el = SPICE_RING_PROD_ITEM(ring);
    /* Careful! Union member info->id and info->next have same guest address */
    *el = free_resources_.front()->id;
    for (auto it = free_resources_.begin(); it != free_resources_.end(); it++) {
      if (it + 1 == free_resources_.end()) {
        (*it)->next = 0;
      } else {
        (*it)->next = (*(it + 1))->id;
      }
    }
  
    bool should_notify;
    SPICE_RING_PUSH(ring, should_notify);
    if (should_notify) {
      SetInterrupt(QXL_INTERRUPT_DISPLAY);
    }
    free_resources_.clear();
  }

  void ReleaseGuestResource(QXLReleaseInfo* info) {
    if (mode_ != kDisplayQxlMode) {
      return;
    }

    std::lock_guard<std::recursive_mutex> lock(mutex_);
    free_resources_.push_back(info);
    if (free_resources_.size() >= 32) {
      FreeGuestResources();
    }
  }

  void FetchGraphicsCommands() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
  
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
    std::lock_guard<std::recursive_mutex> lock(mutex_);
  
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
      ParseCursorCommand(cursor);
      break;
    }
    case QXL_CMD_SURFACE: {
      QXLSurfaceCmd* surface = (QXLSurfaceCmd*)GetMemSlotAddress(command.data);
      ParseSurfaceCommand(surface);
      break;
    }
    default:
      DumpHex(&command, sizeof(command));
      MV_PANIC("unhandled command type=0x%x data=0x%lx", command.type, command.data);
      break;
    }
  }

  void ParseSurfaceCommand(QXLSurfaceCmd* surface_cmd) {
    switch (surface_cmd->type)
    {
    case QXL_SURFACE_CMD_CREATE: {
      auto& create = surface_cmd->u.surface_create;
      auto surface = new Surface;
      surface->id = surface_cmd->surface_id;
      surface->release_info = &surface_cmd->release_info;
      surfaces_[surface->id] = surface;
      if (debug_)
        MV_LOG("create surface id=%d %dx%d format=%d", surface_cmd->surface_id, create.width, create.height, create.format);
      break;
    }
    case QXL_SURFACE_CMD_DESTROY: {
      auto it = surfaces_.find(surface_cmd->surface_id);
      if (it == surfaces_.end()) {
        MV_LOG("surface %d not found", surface_cmd->surface_id);
        break;
      }
      auto surface = it->second;
      if (debug_)
        MV_LOG("destroy surface id=%d", surface->id);

      ReleaseGuestResource(surface->release_info);
      ReleaseGuestResource(&surface_cmd->release_info);
      surfaces_.erase(it);
      delete surface;
      break;
    }
    default:
      MV_PANIC("invalid cmd=%d", surface_cmd->type);
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
    if (drawable->bbox.right > (int32_t)primary_surface_->surface.width ||
      drawable->bbox.bottom > (int32_t)primary_surface_->surface.height) {
      if (debug_) {
        MV_LOG("Invalid draw box %d-%d %d-%d surface %ux%u",
          drawable->bbox.left, drawable->bbox.right, drawable->bbox.top, drawable->bbox.bottom,
          primary_surface_->surface.width, primary_surface_->surface.height);
      }
      ReleaseGuestResource(&drawable->release_info);
      delete partial;
      return;
    }
    MV_ASSERT(drawable->surface_id == 0);
  
    switch (drawable->type)
    {
    case QXL_DRAW_COPY: {
      QXLCopy* copy = &drawable->u.copy;
      MV_ASSERT(drawable->effect == QXL_EFFECT_OPAQUE);
      if (drawable->self_bitmap) {
        MV_ASSERT(drawable->self_bitmap_area.left == drawable->bbox.left && drawable->self_bitmap_area.right == drawable->bbox.right);
      }
      MV_ASSERT(copy->src_area.top == 0 && copy->src_area.left == 0);
      MV_ASSERT(copy->rop_descriptor == SPICE_ROPD_OP_PUT);
      QXLImage* image = (QXLImage*)GetMemSlotAddress(copy->src_bitmap);
      if (image->descriptor.type != SPICE_IMAGE_TYPE_BITMAP) {
        MV_PANIC("image->descriptor.type=0x%x", image->descriptor.type);
        ReleaseGuestResource(&drawable->release_info);
        delete partial;
        break;
      }
      if (image->descriptor.flags != 0) {
        // MV_LOG("image->descriptor.flags=0x%x", image->descriptor.flags);
      }

      QXLBitmap* bitmap = &image->bitmap;
      MV_ASSERT(bitmap->format == SPICE_BITMAP_FMT_RGBA || bitmap->format == SPICE_BITMAP_FMT_32BIT);
      MV_ASSERT(bitmap->palette == 0);
      MV_ASSERT(partial->width == (int)bitmap->x && partial->height == (int)bitmap->y);
      partial->stride = bitmap->stride;
      partial->flip = !(bitmap->flags & QXL_BITMAP_TOP_DOWN);

      if (drawable->clip.type == SPICE_CLIP_TYPE_RECTS) {
        /* FIXME: handle clips */
      }
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
      partial->stride = partial->width * primary_surface_->bytes_pp;
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

  void ParseCursorCommand(QXLCursorCmd* cursor) {
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
