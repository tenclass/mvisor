/* 
 * MVisor QXL
 * Support latest Linux & Windows QXLDoD
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
#include <algorithm>

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
  
struct PrimarySurface {
  QXLSurfaceCreate  create;
  uint32_t          resized;
  int32_t           qxl_stride;
  uint32_t          abs_stride;
  uint32_t          bits_pp;
  uint32_t          bytes_pp;
  bool              active;
};

struct Surface {
  uint            id;
  uint64_t        slot_address;
  QXLSurfaceCmd*  qxl_surface_cmd;
};

struct Drawable
{
  uint64_t        slot_address;
  QXLDrawable*    qxl_drawable;
  bool            drawed;
  bool            drawing;
};

class Qxl : public Vga, public DisplayResizeInterface {
 private:
  uint32_t  qxl_rom_size_;
  void*     qxl_rom_base_ = nullptr;
  uint32_t  qxl_vram32_size_;
  uint8_t*  qxl_vram32_base_ = nullptr;

  QXLRom*   qxl_rom_;
  QXLModes* qxl_modes_;
  QXLRam*   qxl_ram_;

  struct guest_slots {
    QXLMemSlot    slot;
    uint64_t      offset;
    bool          active;
    uint8_t*      hva; 
  } guest_slots_[NUM_MEMSLOTS];

  PrimarySurface                primary_surface_;
  std::map<uint, Surface>       surfaces_;
  std::vector<QXLReleaseInfo*>  free_resources_;
  DisplayMouseCursor            current_cursor_;
  std::list<Drawable>           drawables_;

 public:
  Qxl() {
    default_rom_path_ = "../share/vgabios-qxl.bin";

    pci_header_.vendor_id = 0x1B36;
    pci_header_.device_id = 0x0100;
    pci_header_.revision_id = 5;
    pci_header_.irq_pin = 1;
    
    /* Bar 1: Windows driver uses this block of memory as a normal memslot
     * Linux driver named it surface RAM */
    qxl_vram32_size_ = _MB(16);
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
    primary_surface_.active = false;
    current_cursor_.visible = false;
    surfaces_.clear();
    free_resources_.clear();
    drawables_.clear();
  }

  virtual bool SaveState(MigrationWriter* writer) {
    /* Add all free_sources to release ring */
    FreeGuestResources();
  
    QxlState state;
    for (int i = 0; i < NUM_MEMSLOTS; i++) {
      auto slot = state.add_guest_slots();
      slot->set_mem_start(guest_slots_[i].slot.mem_start);
      slot->set_mem_end(guest_slots_[i].slot.mem_end);
      slot->set_active(guest_slots_[i].active);
    }
  
    if (primary_surface_.active) {
      auto primary = state.mutable_guest_primary();
      auto& surface_create = primary_surface_.create;
      primary->set_width(surface_create.width);
      primary->set_height(surface_create.height);
      primary->set_stride(surface_create.stride);
      primary->set_format(surface_create.format);
      primary->set_position(surface_create.position);
      primary->set_mouse_mode(surface_create.mouse_mode);
      primary->set_flags(surface_create.flags);
      primary->set_type(surface_create.type);
      primary->set_mem_address(surface_create.mem);
    }
  
    /* surfaces are not used now, but save it any way */
    for (auto it = surfaces_.begin(); it != surfaces_.end(); it++) {
      auto& surface = it->second;
      auto sf = state.add_surfaces();
      sf->set_id(surface.id);
      sf->set_slot_address(surface.slot_address);
    }

    for (auto& drawable: drawables_) {
      auto dr = state.add_drawbles();
      dr->set_slot_address(drawable.slot_address);
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

    if (state.has_guest_primary()) {
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
      CreatePrimarySurface(create);
    }

    for (int i = 0; i < state.surfaces_size(); i++) {
      auto& sf = state.surfaces(i);
      surfaces_[sf.id()] = Surface {
        .id = sf.id(),
        .slot_address = sf.slot_address(),
        .qxl_surface_cmd = (QXLSurfaceCmd*)GetMemSlotAddress(sf.slot_address())
      };
    }

    for (int i = 0; i < state.drawbles_size(); i++) {
      auto& dr = state.drawbles(i);
      drawables_.push_back(Drawable {
        .slot_address = dr.slot_address(),
        .qxl_drawable = (QXLDrawable*)GetMemSlotAddress(dr.slot_address())
      });
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

  /* Setup ioeventfd for notify commands, reduce IO operations */
  virtual bool ActivatePciBar(uint8_t index) {
    if (index == 3) {
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

  virtual void UpdateDisplayMode() {
    /* Prevent from calling AcquireUpdate() when setting mode */
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (primary_surface_.active) {
      mode_ = kDisplayQxlMode;
      width_ = primary_surface_.create.width;
      height_ = primary_surface_.create.height;
      bpp_ = primary_surface_.bits_pp;
      stride_ = primary_surface_.abs_stride;
      NotifyDisplayModeChange();
    } else {
      Vga::UpdateDisplayMode();
    }
  }

  virtual void GetDisplayMode(uint* w, uint* h, uint* bpp, uint* stride) {
    if (mode_ == kDisplayQxlMode) {
      *w = primary_surface_.create.width;
      *h = primary_surface_.create.height;
      *bpp = primary_surface_.bits_pp;
      *stride = primary_surface_.abs_stride;
    } else {
      Vga::GetDisplayMode(w, h, bpp, stride);
    }
  }

  void SetInterrupt(uint32_t interrupt) {
    auto old_pending = __atomic_fetch_or(&qxl_ram_->int_pending, interrupt, __ATOMIC_SEQ_CST);
    if (old_pending & interrupt)
      return;
    /* This might be called by UI thread, make sure io thread handles IRQ */
    manager_->io()->Schedule([this]() {
      UpdateIrqLevel();
    });
  }

  void UpdateIrqLevel() {
    int level = !!(qxl_ram_->int_pending & qxl_ram_->int_mask);
    SetIrq(level);
  }

  /* Display resize interface */
  virtual bool Resize(uint width, uint height) {
    if (mode_ != kDisplayQxlMode) {
      return false;
    }
  
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
      case QXL_IO_NOTIFY_CURSOR:
        FetchCommands();
        break;
      case QXL_IO_UPDATE_AREA:
        if (debug_) {
          MV_LOG("QXL_IO_UPDATE_AREA not implemented");
        }
        break;
      case QXL_IO_UPDATE_IRQ:
        UpdateIrqLevel();
        break;
      case QXL_IO_NOTIFY_OOM:
        MV_LOG("QXL_IO_NOTIFY_OOM drawables=%lu", drawables_.size());
        FlushCommandsAndResources(false);
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
        FlushCommandsAndResources(true);
        DestroyPrimarySurface();
        break;
      case QXL_IO_MONITORS_CONFIG_ASYNC:
        break;
      default:
        MV_PANIC("unhandled QXL command=0x%lx", command);
        break;
      }

      if (async) {
        /* Do it like a real async command */
        manager_->io()->Schedule([this]() {
          SetInterrupt(QXL_INTERRUPT_IO_CMD);
        });
      }
    } else {
      Vga::Write(resource, offset, data, size);
    }
  }

  void FlushCommandsAndResources(bool destroy_primary = false) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    /* Fetch any commands on ring */
    FetchCommands();

    /* Remove all drawed objects */
    uint released = 0;
    for (auto it = drawables_.begin(); it != drawables_.end();) {
      if (it->drawed || (destroy_primary && !it->drawing)) {
        ReleaseGuestResource(&it->qxl_drawable->release_info);
        it = drawables_.erase(it);
        ++released;
      } else {
        ++it;
      }
    }

    /* Free released resources */
    FreeGuestResources();

    if (debug_) {
      MV_LOG("drawables released=%u current=%lu", released, drawables_.size());
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
  
    MV_ASSERT(!primary_surface_.active);
    primary_surface_.create = create;
    primary_surface_.qxl_stride = create.stride;
    primary_surface_.abs_stride = abs(create.stride);
    primary_surface_.resized++;
    primary_surface_.active = true;

    switch (create.format)
    {
    case SPICE_SURFACE_FMT_32_xRGB:
    case SPICE_SURFACE_FMT_32_ARGB:
      primary_surface_.bytes_pp = 4;
      primary_surface_.bits_pp = 32;
      break;
    default:
      MV_PANIC("unsupported surface format=0x%x", create.format);
      break;
    }
    if (debug_) {
      MV_LOG("create primary %dx%d", create.width, create.height);
    }
    bool changed = (mode_ != kDisplayQxlMode) || (width_ != create.width) ||
      (height_ != create.height) || (bpp_ != primary_surface_.bits_pp);
    if (changed) {
      UpdateDisplayMode();
    }
  }

  void DestroyPrimarySurface() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
  
    /* Maybe we should notify the viewer??? */
    primary_surface_.active = false;
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

  void GetMemSlotChunkedData(uint64_t data, std::vector<iovec>& vector) {
    while (data) {
      QXLDataChunk* chunk = (QXLDataChunk*)GetMemSlotAddress(data);
      vector.emplace_back(iovec {
        .iov_base = chunk->data,
        .iov_len = chunk->data_size
      });
      data = chunk->next_chunk;
    }
  }

  void GetMemSlotChunkedData(QXLDataChunk* chunk, std::vector<iovec>& vector) {
    while (chunk) {
      vector.emplace_back(iovec {
        .iov_base = chunk->data,
        .iov_len = chunk->data_size
      });
      if (chunk->next_chunk) {
        chunk = (QXLDataChunk*)GetMemSlotAddress(chunk->next_chunk);
      } else {
        chunk = nullptr;
      }
    }
  }

  void FreeGuestResources() {
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
    free_resources_.push_back(info);

    if (free_resources_.size() >= 32) {
      FreeGuestResources();
    }
  }

  /* Lock the drawables and translate to display partial bitmaps */
  virtual bool AcquireUpdate(DisplayUpdate& update) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (mode_ != kDisplayQxlMode) {
      return Vga::AcquireUpdate(update);
    }
    
    FetchCommands();

    for (auto& drawable : drawables_) {
      if (drawable.drawed)
        continue;
      
      drawable.drawing = true;
      ParseDrawble(drawable, update.partials);
    }
    update.cursor = current_cursor_;
    return true;
  }

  /* Update the drawing drawables to drawed */
  virtual void ReleaseUpdate() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (mode_ != kDisplayQxlMode) {
      return Vga::ReleaseUpdate();
    }

    for (auto& drawable : drawables_) {
      if (drawable.drawing) {
        drawable.drawing = false;
        drawable.drawed = true;
      }
    }
  }

  void FetchCommands() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    QXLCommand command;
    while (FetchGraphicsCommand(command)) {
      switch (command.type)
      {
      case QXL_CMD_SURFACE:
        ParseSurfaceCommand(command.data);
        break;
      case QXL_CMD_DRAW:
        ParseDrawCommand(command.data);
        break;
      default:
        MV_PANIC("unhandled command type=0x%x data=0x%lx", command.type, command.data);
        break;
      }
    }
    RemoveInvisibleDrawables();

    while (FetchCursorCommand(command)) {
      MV_ASSERT(command.type == QXL_CMD_CURSOR);
      ParseCursorCommand(command.data);
    }
  }

  bool FetchGraphicsCommand(QXLCommand& command) {
    QXLCommandRing* ring = &qxl_ram_->cmd_ring;
    if (!SPICE_RING_IS_EMPTY(ring)) {
      command = *SPICE_RING_CONS_ITEM(ring);
  
      bool should_notify;
      SPICE_RING_POP(ring, should_notify);
      if (should_notify) {
        SetInterrupt(QXL_INTERRUPT_DISPLAY);
      }
      bool should_wait;
      SPICE_RING_CONS_WAIT(ring, should_wait);
      return true;
    }
    return false;
  }

  bool FetchCursorCommand(QXLCommand& command) {
    QXLCursorRing* ring = &qxl_ram_->cursor_ring;
    if (!SPICE_RING_IS_EMPTY(ring)) {
      command = *SPICE_RING_CONS_ITEM(ring);
      bool should_notify;
      SPICE_RING_POP(ring, should_notify);
      if (should_notify) {
        SetInterrupt(QXL_INTERRUPT_CURSOR);
      }
      bool should_wait;
      SPICE_RING_CONS_WAIT(ring, should_wait);
      return true;
    }
    return false;
  }

  void ParseSurfaceCommand(uint64_t slot_address) {
    auto surface_cmd = (QXLSurfaceCmd*)GetMemSlotAddress(slot_address);
    switch (surface_cmd->type)
    {
    case QXL_SURFACE_CMD_CREATE: {
      auto& create = surface_cmd->u.surface_create;
      surfaces_[surface_cmd->surface_id] = Surface {
        .id = surface_cmd->surface_id,
        .slot_address = slot_address,
        .qxl_surface_cmd = surface_cmd
      };
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
      auto& surface = it->second;
      if (debug_)
        MV_LOG("destroy surface id=%d", surface.id);

      ReleaseGuestResource(&surface.qxl_surface_cmd->release_info);
      ReleaseGuestResource(&surface_cmd->release_info);
      surfaces_.erase(it);
      break;
    }
    default:
      MV_PANIC("invalid cmd=%d", surface_cmd->type);
      break;
    }
  }

  void PushNonOverlappingRects(std::vector<QXLRect>& rects, int left, int top, int right, int bottom) {
    if (left >= right || top >= bottom) {
      /* Invalid */
      return;
    }

    // 扫描是否跟已有矩形有交集
    for (size_t i = 0; i < rects.size(); i++) {
      auto r = rects[i];
      if (r.left < right && r.right > left && r.top < bottom && r.bottom > top) {
        if (r.left > left)
          PushNonOverlappingRects(rects, left, std::max(top, r.top), r.left, std::min(bottom, r.bottom));
        if (r.right < right)
          PushNonOverlappingRects(rects, r.right, std::max(top, r.top), right, std::min(bottom, r.bottom));
        if (r.top > top)
          PushNonOverlappingRects(rects, left, top, right, r.top);
        if (r.bottom < bottom)
          PushNonOverlappingRects(rects, left, r.bottom, right, bottom);
        return;
      }
    }
  
    // 无交集则增加新矩形
    rects.emplace_back(QXLRect {
      .top = top, .left = left, .bottom = bottom, .right = right
    });
  }

  void RemoveInvisibleDrawables() {
    std::vector<QXLRect> draw_rects;
    size_t last_rects_count = 0;

    /* Iterate the drawables from top to bottm, and add non-overlapping rectangle to draw_rects,
     * If no rects are added, that drawable is discardable */
    for (auto it = drawables_.rbegin(); it != drawables_.rend();) {
      auto qxl_drawable = it->qxl_drawable;
      auto& bbox = qxl_drawable->bbox;

      PushNonOverlappingRects(draw_rects, bbox.left, bbox.top, bbox.right, bbox.bottom);
      if (draw_rects.size() == last_rects_count && !it->drawing) {
        ReleaseGuestResource(&qxl_drawable->release_info);
        it = decltype(it)(drawables_.erase(std::next(it).base()));
      } else {
        last_rects_count = draw_rects.size();
        ++it;
      }
    }
  }

  void ParseDrawCommand(uint64_t slot_address) {
    auto qxl_drawable = (QXLDrawable*)GetMemSlotAddress(slot_address);
    drawables_.push_back(Drawable {
      .slot_address = slot_address,
      .qxl_drawable = qxl_drawable
    });
  }

  void ParseCursorCommand(uint64_t slot_address) {
    auto cursor_cmd = (QXLCursorCmd*)GetMemSlotAddress(slot_address);
    switch (cursor_cmd->type)
    {
    case QXL_CURSOR_HIDE:
      current_cursor_.visible = false;
      ++current_cursor_.update_timestamp;
      break;
    case QXL_CURSOR_MOVE:
      current_cursor_.x = cursor_cmd->u.position.x;
      current_cursor_.y = cursor_cmd->u.position.y;
      ++current_cursor_.update_timestamp;
      break;
    case QXL_CURSOR_SET: {
      QXLCursor* cursor = (QXLCursor*)GetMemSlotAddress(cursor_cmd->u.set.shape);
      current_cursor_.visible = cursor_cmd->u.set.visible;
      current_cursor_.x = cursor_cmd->u.set.position.x;
      current_cursor_.y = cursor_cmd->u.set.position.y;

      auto& shape = current_cursor_.shape;
      shape.type = cursor->header.type;
      shape.width = cursor->header.width;
      shape.height = cursor->header.height;
      shape.hotspot_x = cursor->header.hot_spot_x;
      shape.hotspot_y = cursor->header.hot_spot_y;
      shape.vector.clear();
      GetMemSlotChunkedData(&cursor->chunk, shape.vector);
      MV_ASSERT(shape.vector.size() > 0);
      ++shape.id;
      ++current_cursor_.update_timestamp;
      break;
    }
    default:
      DumpHex(cursor_cmd, sizeof(*cursor_cmd));
      MV_PANIC("unhandled cursor type=%d", cursor_cmd->type);
      break;
    }

    ReleaseGuestResource(&cursor_cmd->release_info);
  }

  void ParseDrawble(Drawable& drawable, std::vector<DisplayPartialBitmap>& partials) {
    auto qxl_drawable = drawable.qxl_drawable;
    auto partial = DisplayPartialBitmap {
      .width = uint(qxl_drawable->bbox.right - qxl_drawable->bbox.left),
      .height = uint(qxl_drawable->bbox.bottom - qxl_drawable->bbox.top),
      .x = uint(qxl_drawable->bbox.left),
      .y = uint(qxl_drawable->bbox.top)
    };
    MV_ASSERT(qxl_drawable->bbox.left >= 0 && qxl_drawable->bbox.top >= 0);
    if (qxl_drawable->bbox.right > (int32_t)primary_surface_.create.width ||
      qxl_drawable->bbox.bottom > (int32_t)primary_surface_.create.height) {
      if (debug_) {
        MV_LOG("Invalid draw box %d-%d %d-%d surface %ux%u",
          qxl_drawable->bbox.left, qxl_drawable->bbox.right, qxl_drawable->bbox.top, qxl_drawable->bbox.bottom,
          primary_surface_.create.width, primary_surface_.create.height);
      }
      return;
    }
    MV_ASSERT(qxl_drawable->surface_id == 0);
  
    switch (qxl_drawable->type)
    {
    case QXL_DRAW_COPY: {
      MV_ASSERT(qxl_drawable->effect == QXL_EFFECT_OPAQUE);
      QXLCopy* copy = &qxl_drawable->u.copy;
      MV_ASSERT(copy->src_area.top == 0 && copy->src_area.left == 0);
      MV_ASSERT(copy->rop_descriptor == SPICE_ROPD_OP_PUT);
      QXLImage* image = (QXLImage*)GetMemSlotAddress(copy->src_bitmap);
      if (image->descriptor.type != SPICE_IMAGE_TYPE_BITMAP) {
        MV_PANIC("image->descriptor.type=0x%x", image->descriptor.type);
        break;
      }
      if (image->descriptor.flags != 0) {
        /* FIXME: CACHE_ME flag */
      }

      QXLBitmap* bitmap = &image->bitmap;
      MV_ASSERT(bitmap->format == SPICE_BITMAP_FMT_RGBA || bitmap->format == SPICE_BITMAP_FMT_32BIT);
      MV_ASSERT(bitmap->palette == 0);
      MV_ASSERT(partial.width == bitmap->x && partial.height == bitmap->y);
      partial.stride = bitmap->stride;
      partial.flip = !(bitmap->flags & QXL_BITMAP_TOP_DOWN);

      if (qxl_drawable->clip.type == SPICE_CLIP_TYPE_RECTS) {
        /* FIXME: handle clips */
      }
      GetMemSlotChunkedData(bitmap->data, partial.vector);
      partials.emplace_back(std::move(partial));
      break;
    }
    case QXL_DRAW_FILL: {
      QXLFill* fill = &qxl_drawable->u.fill;
      MV_ASSERT(fill->rop_descriptor == SPICE_ROPD_OP_PUT);
      QXLBrush* brush = &fill->brush;
      MV_ASSERT(brush->type == SPICE_BRUSH_TYPE_SOLID);
      partial.stride = partial.width * primary_surface_.bytes_pp;
      uint32_t color = brush->u.color;
      size_t size = partial.stride * partial.height;
      vga_surface_.resize(size);
      memset(vga_surface_.data(), color, size);
      partial.vector.emplace_back(iovec {
        .iov_base = vga_surface_.data(),
        .iov_len = size
      });
      partials.emplace_back(std::move(partial));
      break;
    }
    default:
      DumpHex(qxl_drawable, sizeof(*qxl_drawable));
      MV_PANIC("unhandled drawable type=%d", qxl_drawable->type);
      break;
    }
  }
};

DECLARE_DEVICE(Qxl);
