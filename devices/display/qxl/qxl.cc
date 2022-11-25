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
#include <algorithm>

#include <zlib.h>
#include <sys/mman.h>

#include "qxl.h"
#include "qxl_render.h"
#include "qxl_modes.h"
#include "qxl.pb.h"
#include "logger.h"


Qxl::Qxl() {
  default_rom_path_ = "../share/vgabios-qxl.bin";
  pci_header_.vendor_id = 0x1B36;
  pci_header_.device_id = 0x0100;
  pci_header_.revision_id = 5;
  pci_header_.irq_pin = 1;
  
  /* Bar 1: Windows driver uses this block of memory as a normal memslot
    * Linux driver named it surface RAM */
  qxl_vram32_size_ = _MB(16);
  qxl_vram32_base_ = (uint8_t*)mmap(nullptr, qxl_vram32_size_, PROT_READ | PROT_WRITE,
    MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
  pci_bars_[1].host_memory = qxl_vram32_base_;

  MV_ASSERT(madvise(qxl_vram32_base_, qxl_vram32_size_, MADV_DONTDUMP) == 0);

  /* Bar 2: 8KB ROM */
  qxl_rom_size_ = 8192;
  qxl_rom_base_ = valloc(qxl_rom_size_);
  pci_bars_[2].host_memory = qxl_rom_base_;

  AddPciBar(1, qxl_vram32_size_, kIoResourceTypeRam); /* QXL VRAM32 8MB */
  AddPciBar(2, 8192, kIoResourceTypeRam);             /* QXL ROM */
  AddPciBar(3, 32, kIoResourceTypePio);               /* QXL PIO */

  render_ = new QxlRender(this);
}

Qxl::~Qxl() {
  /* Destroy renderer */
  delete render_;

  if (qxl_vram32_base_) {
    munmap(qxl_vram32_base_, qxl_vram32_size_);
  }
  if (qxl_rom_base_) {
    free(qxl_rom_base_);
  }
}

void Qxl::Connect() {
  Vga::Connect();

  /* Push all free resources to the ring before saving VM */
  auto machine = manager_->machine();
  state_change_listener_ = machine->RegisterStateChangeListener([=]() {
    if (machine->IsPaused()) {
      FreeGuestResources();
    }
  });
}

void Qxl::Disconnect() {
  manager_->machine()->UnregisterStateChangeListener(&state_change_listener_);

  Vga::Disconnect();
}

void Qxl::Reset() {
  std::lock_guard<std::recursive_mutex> render_lock(render_mutex_);

  /* Vga Reset() resets mode */
  Vga::Reset();

  IntializeQxlRom();
  IntializeQxlRam();

  delete render_;
  render_ = new QxlRender(this);

  bzero(guest_slots_, sizeof(guest_slots_));
  primary_surface_.active = false;
  current_cursor_.visible = false;
  free_resources_.clear();
}

bool Qxl::SaveState(MigrationWriter* writer) {
  /* We have added all free resources to release ring when VM pause */
  MV_ASSERT(free_resources_.empty());

  QxlState state;
  for (int i = 0; i < NUM_MEMSLOTS; i++) {
    auto slot = state.add_guest_slots();
    slot->set_mem_start(guest_slots_[i].slot.mem_start);
    slot->set_mem_end(guest_slots_[i].slot.mem_end);
    slot->set_active(guest_slots_[i].active);
  }

  if (primary_surface_.active) {
    auto primary = state.mutable_guest_primary();
    auto& surface_create = qxl_ram_->create_surface;
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
  auto& surfaces = render_->surfaces();
  for (auto it = surfaces.begin(); it != surfaces.end(); it++) {
    if (it->first == 0)
      continue;
    auto& surface = it->second;
    auto sf = state.add_surfaces();
    sf->set_id(surface.id);
    sf->set_slot_address(surface.slot_address);
  }

  writer->WriteProtobuf("QXL", state);
  writer->WriteMemoryPages("VRAM32", qxl_vram32_base_, qxl_vram32_size_);
  return Vga::SaveState(writer);
}

/* Reset should be called before load state */
bool Qxl::LoadState(MigrationReader* reader) {
  if (!Vga::LoadState(reader)) {
    return false;
  }

  if (qxl_vram32_base_) {
    munmap(qxl_vram32_base_, qxl_vram32_size_);
  }
  if (!reader->ReadMemoryPages("VRAM32", (void**)&qxl_vram32_base_, qxl_vram32_size_)) {
    return false;
  }

  QxlState state;
  if (!reader->ReadProtobuf("QXL", state)) {
    return false;
  }
  for (int i = 0; i < NUM_MEMSLOTS && i < state.guest_slots_size(); i++) {
    auto& slot = state.guest_slots(i);
    if (slot.active()) {
      QXLMemSlot mem_slot = {
        .mem_start = slot.mem_start(),
        .mem_end = slot.mem_end()
      };
      AddMemSlot(i, mem_slot);
    }
  }

  for (int i = 0; i < state.surfaces_size(); i++) {
    auto& sf = state.surfaces(i);
    if (sf.id() == 0) {
      MV_ERROR("invalid surface id %u", sf.id());
      continue;
    }
    render_->ParseSurfaceCommand(sf.slot_address());
  }

  if (state.has_guest_primary()) {
    primary_surface_.active = true;
    render_->CreatePrimarySurface(qxl_ram_->create_surface, false);
    UpdateDisplayMode();

    for (int i = 0; i < state.drawbles_size(); i++) {
      auto& dr = state.drawbles(i);
      render_->ParseDrawCommand(dr.slot_address());
    }
  
    /* push update event after loaded */
    Schedule([this]() {
      NotifyDisplayUpdate();
    });
  }
  return true;
}

void Qxl::IntializeQxlRom() {
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

void Qxl::IntializeQxlRam() {
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
bool Qxl::ActivatePciBar(uint8_t index) {
  if (index == 3) {
    manager_->RegisterIoEvent(this, kIoResourceTypePio, pci_bars_[index].address + QXL_IO_NOTIFY_CMD);
    manager_->RegisterIoEvent(this, kIoResourceTypePio, pci_bars_[index].address + QXL_IO_NOTIFY_CURSOR);
  }
  return Vga::ActivatePciBar(index);
}

bool Qxl::DeactivatePciBar(uint8_t index) {
  if (index == 3) {
    manager_->UnregisterIoEvent(this, kIoResourceTypePio, pci_bars_[index].address + QXL_IO_NOTIFY_CMD);
    manager_->UnregisterIoEvent(this, kIoResourceTypePio, pci_bars_[index].address + QXL_IO_NOTIFY_CURSOR);
  }
  return Vga::DeactivatePciBar(index);
}

void Qxl::UpdateDisplayMode() {
  /* Prevent from calling AcquireUpdate() when setting mode */
  if (primary_surface_.active) {
    auto& create = qxl_ram_->create_surface;
    if (mode_ == kDisplayQxlMode && width_ == (int)create.width && height_ == (int)create.height &&
      bpp_ == GetBitsPerPixelByFormat(create.format)) {
      return;
    }
    mode_ = kDisplayQxlMode;
    width_ = create.width;
    height_ = create.height;
    stride_ = create.stride;
    bpp_ = GetBitsPerPixelByFormat(create.format);

    NotifyDisplayModeChange();
  } else {
    Vga::UpdateDisplayMode();
  }
}

void Qxl::SetInterrupt(uint32_t interrupt) {
  auto old_pending = __atomic_fetch_or(&qxl_ram_->int_pending, interrupt, __ATOMIC_SEQ_CST);
  if (old_pending & interrupt)
    return;
  UpdateIrqLevel();
}

void Qxl::UpdateIrqLevel() {
  int level = !!(qxl_ram_->int_pending & qxl_ram_->int_mask);
  SetIrq(level);
}

/* Display resize interface */
bool Qxl::Resize(int width, int height) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  /*
    * Older windows drivers set int_mask to 0 when their ISR is called,
    * then later set it to ~0. So it doesn't relate to the actual interrupts
    * handled. However, they are old, so clearly they don't support this
    * interrupt
    */
  if (qxl_ram_->int_mask == 0 || qxl_ram_->int_mask == ~0u ||
    !(qxl_ram_->int_mask & QXL_INTERRUPT_CLIENT_MONITORS_CONFIG)) {
    return false;
  }

  /* This should return true if QXL is capable of sending interrupts to config monitors */
  if (mode_ != kDisplayQxlMode || !width || !height) {
    return true;
  }

  Schedule([=]() {
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
  });

  return true;
}

uint64_t Qxl::TranslateAsyncCommand(uint64_t command, bool* async) {
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

void Qxl::ParseControlCommand(uint64_t command, uint32_t argument) {
  std::lock_guard<std::recursive_mutex> render_lock(render_mutex_);
  switch (command) {
  case QXL_IO_UPDATE_AREA:
    FetchCommands();
    render_->UpdateArea(qxl_ram_->update_area, qxl_ram_->update_surface);
    NotifyDisplayUpdate();
    break;
  case QXL_IO_NOTIFY_OOM:
    MV_LOG("QXL_IO_NOTIFY_OOM");
    FlushCommandsAndResources();
    break;
  case QXL_IO_RESET:
    Reset();
    UpdateIrqLevel();
    break;
  case QXL_IO_LOG:
    if (!qxl_ram_->log_buf[0]) {
      MV_LOG("QXL_IO_LOG", qxl_ram_->log_buf);
    }
    break;
  case QXL_IO_MEMSLOT_ADD:
    MV_ASSERT(argument < NUM_MEMSLOTS);
    MV_ASSERT(!guest_slots_[argument].active);
    AddMemSlot(argument, qxl_ram_->mem_slot);
    break;
  case QXL_IO_MEMSLOT_DEL:
    DeleteMemSlot(argument);
    break;
  case QXL_IO_CREATE_PRIMARY:
    render_->CreatePrimarySurface(qxl_ram_->create_surface, true);
    primary_surface_.active = true;
    UpdateDisplayMode();
    break;
  case QXL_IO_DESTROY_PRIMARY:
    FlushCommandsAndResources();
    if (primary_surface_.active) {
      render_->DestroyPrimarySurface();
      primary_surface_.active = false;
      mode_ = kDisplayUnknownMode;
    }
    break;
  case QXL_IO_DESTROY_ALL_SURFACES:
    FlushCommandsAndResources();
    delete render_;
    render_ = new QxlRender(this);
    break;
  case QXL_IO_FLUSH_SURFACES_ASYNC:
    if (debug_) {
      MV_LOG("QXL_IO_FLUSH_SURFACES_ASYNC");
    }
    break;
  case QXL_IO_MONITORS_CONFIG_ASYNC:
    if (debug_) {
      MV_LOG("QXL_IO_MONITORS_CONFIG_ASYNC");
    }
    break;
  case QXL_IO_FLUSH_RELEASE:
    FreeGuestResources();
    break;
  default:
    MV_PANIC("unhandled QXL command=0x%lx", command);
    break;
  }
}

void Qxl::Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  if (resource->base == pci_bars_[3].address) {
    /* ignore all reads to the QXL io ports */
    memset(data, 0xFF, size);
  } else {
    Vga::Read(resource, offset, data, size);
  }
}

void Qxl::Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  if (resource->base == pci_bars_[3].address) {
    bool async;
    uint64_t command = TranslateAsyncCommand(offset, &async);
    switch (command)
    {
    case QXL_IO_NOTIFY_CMD:
    case QXL_IO_NOTIFY_CURSOR:
      NotifyDisplayUpdate();
      break;
    case QXL_IO_UPDATE_IRQ:
      UpdateIrqLevel();
      break;
    default:
      ParseControlCommand(command, data[0]);
    }

    if (async) {
      /* Async command needs interrupt */
      Schedule([this]() {
        SetInterrupt(QXL_INTERRUPT_IO_CMD);
      });
    }
  } else {
    Vga::Write(resource, offset, data, size);
  }
}

void Qxl::FlushCommandsAndResources() {
  /* Fetch any commands on ring */
  flushing_commands_ = true;
  FetchCommands();
  flushing_commands_ = false;

  /* Free released resources */
  FreeGuestResources();
}

void Qxl::AddMemSlot(int slot_id, QXLMemSlot& slot) {
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

void Qxl::DeleteMemSlot(int slot_id) {
  guest_slots_[slot_id].active = false;
}

int Qxl::GetBitsPerPixelByFormat(uint32_t format) {
  switch (format)
  {
  case SPICE_SURFACE_FMT_32_xRGB:
  case SPICE_SURFACE_FMT_32_ARGB:
    return 32;
  default:
    MV_ERROR("unsupported format=0x%x", format);
    return 0;
  }
}

void* Qxl::GetMemSlotAddress(uint64_t data) {
  uint64_t slot_id = data >> (64 - qxl_rom_->slot_id_bits);
  MV_ASSERT(slot_id < NUM_MEMSLOTS);

  uint64_t generation_mask = ((1UL << qxl_rom_->slot_gen_bits) - 1);
  uint64_t generation_shift = 64 - qxl_rom_->slot_id_bits - qxl_rom_->slot_gen_bits;
  uint64_t generation = (data >> generation_shift) & generation_mask;
  MV_ASSERT(generation == qxl_rom_->slot_generation);

  uint64_t virtual_mask = (1UL << generation_shift) - 1;
  return guest_slots_[slot_id].hva + (data & virtual_mask);
}

size_t Qxl::GetMemSlotChunkData(uint64_t data, std::vector<iovec>& vector) {
  size_t size = 0;
  while (data) {
    QXLDataChunk* chunk = (QXLDataChunk*)GetMemSlotAddress(data);
    vector.emplace_back(iovec {
      .iov_base = chunk->data,
      .iov_len = chunk->data_size
    });
    size += chunk->data_size;
    data = chunk->next_chunk;
  }
  return size;
}

size_t Qxl::GetMemSlotChunkData(QXLDataChunk* chunk, std::vector<iovec>& vector) {
  size_t size = 0;
  while (chunk) {
    vector.emplace_back(iovec {
      .iov_base = chunk->data,
      .iov_len = chunk->data_size
    });
    size += chunk->data_size;
    if (chunk->next_chunk) {
      chunk = (QXLDataChunk*)GetMemSlotAddress(chunk->next_chunk);
    } else {
      chunk = nullptr;
    }
  }
  return size;
}

void Qxl::GetMemSlotLinearizedChunkData(QXLDataChunk* chunk, std::string& data) {
  std::vector<iovec> v;
  size_t size = GetMemSlotChunkData(chunk, v);
  data.resize(size);
  size_t offset = 0;
  for (auto& iov : v) {
    memcpy(data.data() + offset, iov.iov_base, iov.iov_len);
    offset += iov.iov_len;
  }
}

void Qxl::FreeGuestResources() {
  if (flushing_commands_ || free_resources_.empty()) {
    return;
  }

  QXLReleaseRing* ring = &qxl_ram_->release_ring;
  if (ring->prod - ring->cons == ring->num_items - 1) {
    return; // ring is full
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

void Qxl::ReleaseGuestResource(QXLReleaseInfo* info) {
  free_resources_.push_back(info);

  if (free_resources_.size() >= 32) {
    FreeGuestResources();
  }
}

/* Lock the drawables and translate to display partial bitmaps */
bool Qxl::AcquireUpdate(DisplayUpdate& update, bool redraw) {
  std::unique_lock<std::recursive_mutex>  lock(mutex_);
  std::lock_guard<std::recursive_mutex>   render_lock(render_mutex_);
  if (mode_ != kDisplayQxlMode) {
    return Vga::AcquireUpdate(update, redraw);
  }

  /* We cannot hold the device lock for so long, here switch to render lock */
  lock.unlock();

  if (!manager_->machine()->IsPaused()) {
    FetchCommands();
  }

  if (redraw) {
    render_->Redraw();
  }
  render_->GetUpdatePartials(update.partials);
  update.cursor = current_cursor_;
  return true;
}

/* Update the drawing drawables to drawed */
void Qxl::ReleaseUpdate() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (mode_ != kDisplayQxlMode) {
    return Vga::ReleaseUpdate();
  }
}

void Qxl::FetchCommands() {
  QXLCommand command;
  while (FetchGraphicsCommand(command)) {
    switch (command.type)
    {
    case QXL_CMD_SURFACE:
      render_->ParseSurfaceCommand(command.data);
      break;
    case QXL_CMD_DRAW:
      render_->ParseDrawCommand(command.data);
      break;
    default:
      MV_PANIC("unhandled command type=0x%x data=0x%lx", command.type, command.data);
      break;
    }
  }

  while (FetchCursorCommand(command)) {
    MV_ASSERT(command.type == QXL_CMD_CURSOR);
    ParseCursorCommand(command.data);
  }
}

bool Qxl::FetchGraphicsCommand(QXLCommand& command) {
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

bool Qxl::FetchCursorCommand(QXLCommand& command) {
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

void Qxl::ParseCursorCommand(uint64_t slot_address) {
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
  
    /* Linearize chunked data */
    GetMemSlotLinearizedChunkData(&cursor->chunk, shape.data);

    ++shape.id;
    ++current_cursor_.update_timestamp;
    break;
  }
  default:
    MV_HEXDUMP("unhandled cursor", cursor_cmd, sizeof(*cursor_cmd));
    break;
  }

  ReleaseGuestResource(&cursor_cmd->release_info);
}


DECLARE_DEVICE(Qxl);
