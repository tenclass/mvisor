/* 
 * MVisor
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


#include "memory_manager.h"

#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <linux/kvm.h>

#include "dirty_memory.pb.h"
#include "machine.h"
#include "logger.h"


#define X86_EPT_IDENTITY_BASE 0xFEFFC000
static const char* type_strings[] = { "Reserved", "RAM", "Device", "ROM", "Unknown" };


MemoryManager::MemoryManager(const Machine* machine)
    : machine_(machine) {
  
  /* Get the number of slots we can allocate */
  uint max_slots = ioctl(machine_->kvm_fd_, KVM_CHECK_EXTENSION, KVM_CAP_NR_MEMSLOTS);
  for (uint i = 0; i < max_slots; i++) {
    free_slots_.insert(i);
  }

  /* Enable KVM_CAP_MANUAL_DIRTY_LOG_PROTECT2 for migration */
  MV_ASSERT(ioctl(machine_->kvm_fd_, KVM_CHECK_EXTENSION, KVM_CAP_MANUAL_DIRTY_LOG_PROTECT2));
  struct kvm_enable_cap enable_cap = {0};
  enable_cap.cap = KVM_CAP_MANUAL_DIRTY_LOG_PROTECT2;
  enable_cap.args[0] = KVM_DIRTY_LOG_MANUAL_PROTECT_ENABLE | KVM_DIRTY_LOG_INITIALLY_SET;
  MV_ASSERT(ioctl(machine_->vm_fd_, KVM_ENABLE_CAP, &enable_cap) == 0);

  /* Setup the memory slot to be traced */
  trace_slot_names_.insert("System");
  trace_slot_names_.insert("SeaBIOS");

  /* Setup the system memory map for BIOS to run */
  InitializeReservedMemory();
  InitializeSystemRam();
  LoadBiosFile();
}

MemoryManager::~MemoryManager() {
  for (auto listener: memory_listeners_) {
    delete listener;
  }
  for (auto listener: dirty_memory_listeners_) {
    delete listener;
  }
  for (auto it = kvm_slots_.begin(); it != kvm_slots_.end(); it++) {
    delete it->second;
  }
  for (auto region : regions_) {
    delete region;
  }
  munmap(ram_host_, machine_->ram_size_);
  dirty_memory_regions_.clear();
  if (bios_data_)
    free(bios_data_);
  if (bios_backup_)
    free(bios_backup_);
}

/* Allocate system ram for guest */
void MemoryManager::InitializeSystemRam() {
  if (machine_->debug_)
    MV_LOG("RAM size: %lu MB", machine_->ram_size_ >> 20);

  ram_host_ = mmap(nullptr, machine_->ram_size_, PROT_READ | PROT_WRITE,
    MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
  MV_ASSERT(ram_host_ != MAP_FAILED);

  /* Make host RAM mergeable (for KSM) */
  MV_ASSERT(madvise(ram_host_, machine_->ram_size_, MADV_MERGEABLE) == 0);
  MV_ASSERT(madvise(ram_host_, machine_->ram_size_, MADV_DONTDUMP) == 0);

  /* Don't map MMIO region */
  const uint64_t low_ram_upper_bound = 2 * (1LL << 30);
  const uint64_t high_ram_lower_bound = 1LL << 32;
  if (machine_->ram_size_ <= low_ram_upper_bound) {
    Map(0, machine_->ram_size_, ram_host_, kMemoryTypeRam, "System");
  } else {
    // Split the ram to two segments leaving a hole in the GPA
    Map(0, low_ram_upper_bound, ram_host_, kMemoryTypeRam, "System");
    // Skip the hole and map the rest
    Map(high_ram_lower_bound, machine_->ram_size_ - low_ram_upper_bound,
      (uint8_t*)ram_host_ + low_ram_upper_bound, kMemoryTypeRam, "System");
  }
}

/*
  * On older Intel CPUs, KVM uses vm86 mode to emulate 16-bit code directly.
  * In order to use vm86 mode, an EPT identity map and a TSS  are needed.
  * Since these must be part of guest physical memory, we need to allocate
  * them, both by setting their start addresses in the kernel and by
  * creating a corresponding e820 entry. We need 4 pages before the BIOS.
  *
  * Older KVM versions may not support setting the identity map base. In
  * that case we need to stick with the default, i.e. a 256K maximum BIOS
  * size.
  */
void MemoryManager::InitializeReservedMemory() {
  /* Allows up to 16M BIOSes. */
  uint64_t identity_base = X86_EPT_IDENTITY_BASE;
  if (ioctl(machine_->vm_fd_, KVM_SET_IDENTITY_MAP_ADDR, &identity_base) < 0) {
    MV_PANIC("failed to set identity map address");
  }

  if (ioctl(machine_->vm_fd_, KVM_SET_TSS_ADDR, identity_base + 0x1000) < 0) {
    MV_PANIC("failed to set tss");
  }
  
  /* Map these addresses as reserved so the guest never touch it */
  Map(X86_EPT_IDENTITY_BASE, 4 * PAGE_SIZE, nullptr, kMemoryTypeReserved, "EPT+TSS");
}

/* SeaBIOS is loaded into the end of 1MB and the end of 4GB */
void MemoryManager::LoadBiosFile() {
  auto& bios_path = machine_->config_->bios_path();
  // Read BIOS data from path to bios_data
  int fd = open(bios_path.c_str(), O_RDONLY);
  MV_ASSERT(fd > 0);  
  struct stat st;
  fstat(fd, &st);

  bios_size_ = st.st_size;
  bios_backup_ = malloc(bios_size_);
  read(fd, bios_backup_, bios_size_);
  safe_close(&fd);

  bios_data_ = valloc(bios_size_);
  memcpy(bios_data_, bios_backup_, bios_size_);
  // Map BIOS file to memory
  Map(0x100000 - bios_size_, bios_size_, bios_data_, kMemoryTypeRam, "SeaBIOS");
  Map(0x100000000 - bios_size_, bios_size_, bios_data_, kMemoryTypeRam, "SeaBIOS");
}

void MemoryManager::Reset() {
  /* Reset BIOS data */
  memcpy(bios_data_, bios_backup_, bios_size_);
  /* Reset 64KB low memory, or Windows 11 complains about some data at 0x6D80 when reboots */
  bzero(ram_host_, 0x10000);
}

/* The number of KVM slots is limited, try not to use out */
uint MemoryManager::AllocateSlotId() {
  MV_ASSERT(!free_slots_.empty());
  auto it = free_slots_.begin();
  uint slot_id = *it;
  free_slots_.erase(it);
  return slot_id;
}

void MemoryManager::UpdateKvmSlot(MemorySlot* slot, bool remove) {
  kvm_userspace_memory_region mr {
    .slot = slot->id,
    .flags = slot->flags,
    .guest_phys_addr = slot->begin,
    .memory_size = remove ? 0 : slot->end - slot->begin,
    .userspace_addr = slot->hva
  };

  if (ioctl(machine_->vm_fd_, KVM_SET_USER_MEMORY_REGION, &mr) < 0) {
    MV_PANIC("failed to set user memory region slot=%d gpa=%016lx size=%016lx hva=%016lx flags=%d",
      mr.slot, mr.guest_phys_addr, mr.memory_size, mr.userspace_addr, mr.flags);
  }
}

/* Don't call this funciton, use Map and Unmap */
void MemoryManager::AddMemoryRegion(MemoryRegion* region) {
  std::unordered_set<MemorySlot*> pending_add;
  std::unordered_set<MemorySlot*> pending_remove;
  
  /* Lock the global mutex while handling insert or remove */
  std::unique_lock lock(mutex_);

  MemorySlot* slot = new MemorySlot;
  slot->id = AllocateSlotId();
  slot->region = region;
  slot->type = region->type;
  slot->begin = region->gpa;
  slot->end = region->gpa + region->size;
  slot->hva = reinterpret_cast<uint64_t>(region->host);
  slot->flags = region->flags;
  pending_add.insert(slot);

  // Find all overlapped slots, split them, remove the old ones (resizing is not supported by KVM)
  // Maybe later we should support region priorities
  for (auto it = kvm_slots_.begin(); it != kvm_slots_.end() && it->second->begin < slot->end; ) {
    if (it->second->begin < slot->end && slot->begin < it->second->end) {
      MemorySlot *hit = it->second;
      MemorySlot *left = nullptr, *right = nullptr;
  
      // Collision found, split the slot
      if (hit->begin < slot->begin) {
        // Left collision
        left = new MemorySlot(*hit);
        left->end = slot->begin;
        left->id = AllocateSlotId();
      }
      if (slot->end < hit->end) {
        // Right collision
        right = new MemorySlot(*hit);
        right->begin = slot->end;
        right->hva = reinterpret_cast<uint64_t>(right->region->host) + (right->begin - right->region->gpa);
        right->id = AllocateSlotId();
      }
      // Move the iterator before we change kvm_slots
      if (!left && !right) {
        it = kvm_slots_.erase(it);
      } else {
        ++it;
        // Replace the hit slot with left and right
        if (left) {
          pending_add.insert(left);
          kvm_slots_[left->begin] = left;
        }
        if (right) {
          pending_add.insert(right);
          kvm_slots_[right->begin] = right;
        }
      }
      
      if (pending_add.find(hit) != pending_add.end()) {
        // Second collision happened, just remove the previous created
        pending_add.erase(hit);
        delete hit;
      } else {
        pending_remove.insert(hit);
      }
    } else {
      ++it;
    }
  }

  // Finally add the new slot
  kvm_slots_[slot->begin] = slot;
  regions_.insert(region);

  // Commit the pending slots to KVM
  for (auto slot : pending_remove) {
    if (slot->type == kMemoryTypeRam || slot->type == kMemoryTypeRom) {
      UpdateKvmSlot(slot, true);
      free_slots_.insert(slot->id);
    }
    // tell listeners we removed a slot
    for (auto listener : memory_listeners_) {
      listener->callback(slot, true);
    }
    delete slot;
  }
  for (auto slot : pending_add) {
    if (slot->type == kMemoryTypeRam || slot->type == kMemoryTypeRom) {
      UpdateKvmSlot(slot, false);
    }
    // tell listeners we have new slots
    for (auto listener : memory_listeners_) {
      listener->callback(slot, false);
    }
  }
}

/* Mapping a memory region in the guest address space */
const MemoryRegion* MemoryManager::Map(uint64_t gpa, uint64_t size, void* host, MemoryType type, const char* name) {
  MV_ASSERT(size > 0);
  MemoryRegion* region = new MemoryRegion;
  region->gpa = gpa;
  region->host = host;
  region->size = size;
  region->type = type;
  region->flags = type == kMemoryTypeRom ? KVM_MEM_READONLY : 0;
  if (type == kMemoryTypeRam && trace_slot_names_.find(name) != trace_slot_names_.end()) {
    region->flags |= KVM_MEM_LOG_DIRTY_PAGES;
  }
  strncpy(region->name, name, 20 - 1);

  if (machine_->debug_)  {
    MV_LOG("map region %s gpa=0x%lx size=0x%lx type=%s", region->name,
      region->gpa, region->size, type_strings[region->type]);
  }

  AddMemoryRegion(region);
  return region;
}

/* TODO: should merge the slots after unmap */
void MemoryManager::Unmap(const MemoryRegion** pregion) {
  auto region = (MemoryRegion*)*pregion;
  if (machine_->debug_) {
    MV_LOG("unmap region %s gpa=0x%lx size=%lx type=%s", region->name,
      region->gpa, region->size, type_strings[region->type]);
  }

  std::unique_lock lock(mutex_);
  // Remove KVM slots
  for (auto it = kvm_slots_.begin(); it != kvm_slots_.end(); ) {
    auto slot = it->second;
    if (slot->region == region) {
      if (slot->type == kMemoryTypeRam || slot->type == kMemoryTypeRom) {
        UpdateKvmSlot(slot, true);
        free_slots_.insert(slot->id);
      }
      // tell listeners we removed a slot
      for (auto listener : memory_listeners_) {
        listener->callback(slot, true);
      }
      delete slot;
      it = kvm_slots_.erase(it);
    } else {
      ++it;
    }
  }

  // Remove region
  if (regions_.erase(region)) {
    delete region;
    *pregion = nullptr;
  }
}

/* StartTrackingDirtyMemory must be called in paused state to 
 * make sure that all dirty memory was updated completely */
void MemoryManager::StartTrackingDirtyMemory() {
  // flush dirty memory from kvm
  for (auto& slot : GetSlotsByNames(trace_slot_names_)) {
    size_t slot_size = slot.end - slot.begin;
    size_t bitmap_size = ALIGN(slot_size / PAGE_SIZE, 64) / 8;
    auto dirty_bitmap = new uint8_t[bitmap_size];
    if (GetDirtyBitmapFromKvm(slot.id, dirty_bitmap)) {
      kvm_clear_dirty_log clear_dirty = {
        .slot = slot.id,
        .num_pages = (uint32_t)(slot_size / PAGE_SIZE),
        .first_page = 0,
        .dirty_bitmap = dirty_bitmap
      };
      MV_ASSERT(ioctl(machine_->vm_fd_, KVM_CLEAR_DIRTY_LOG, &clear_dirty) == 0);
    }
    delete[] dirty_bitmap;
  }

  // tell listeners start tracking dirty memory
  for (auto listener : dirty_memory_listeners_) {
    listener->callback(kStartTrackingDirtyMemory);
  }

  // flush dirty memory from dirty_memory_regions_
  dirty_memory_regions_.clear();

  // open track memory switch
  track_dirty_memory_ = true;
}

void MemoryManager::StopTrackingDirtyMemory() {
  // tell listeners stop tracking dirty memory
  for (auto listener : dirty_memory_listeners_) {
    listener->callback(kStopTrackingDirtyMemory);
  }

  // close track memory switch
  track_dirty_memory_ = false;
}

// Update dirty memory map for migration
void MemoryManager::SetDirtyMemoryRegion(uint64_t gpa, size_t size) {
  if (!track_dirty_memory_) {
    return;
  }

  std::lock_guard<std::mutex> lock(dirty_memory_region_mutex_);
  auto item = dirty_memory_regions_.find(gpa);
  if (item != dirty_memory_regions_.end()) {
    item->second = std::max(item->second, size);
  } else {
    dirty_memory_regions_.insert(std::pair<uint64_t, size_t>(gpa, size));
  }
}

/* Since slots is a flat view without overlaps,
 * we simply use upper_bound to locate the slot in O(logN)
 */
void* MemoryManager::GuestToHostAddress(uint64_t gpa) {
  std::shared_lock lock(mutex_);

  // Find the first slot whose begin is smaller than gpa
  auto it = kvm_slots_.upper_bound(gpa);
  if (it != kvm_slots_.begin()) {
    --it;
  }
  MV_ASSERT(it != kvm_slots_.end());

  MemorySlot* slot = it->second;
  if (gpa >= slot->begin && gpa < slot->end) {
    uint64_t address = slot->hva + gpa - slot->begin;
    return reinterpret_cast<void*>(address);
  }

  // should never reach here
  PrintMemoryScope();
  MV_PANIC("failed to translate guest physical address 0x%016lx", gpa);
  return nullptr;
}

// WARN: low performance
uint64_t MemoryManager::HostToGuestAddress(void* host) {
  std::shared_lock lock(mutex_);
  for (auto it = kvm_slots_.begin(); it != kvm_slots_.end(); it++) { 
    auto slot = it->second;
    if (slot->type != kMemoryTypeRam) {
      continue;
    }

    auto hva = reinterpret_cast<uint64_t>(host);
    auto slot_size = slot->end - slot->begin;
    if (hva >= slot->hva && hva < slot->hva + slot_size) {
      return slot->begin + (hva - slot->hva);
    }
  }

  // should never reach here
  PrintMemoryScope();
  MV_PANIC("failed to translate host address to guest 0x%016lx", host);
  return 0;
}

bool MemoryManager::GetDirtyBitmapFromKvm(uint32_t slot, void* bitmap) {
  kvm_dirty_log dirty = {0};
  dirty.slot = slot;
  dirty.dirty_bitmap = bitmap;

  // dirty_bitmap containing any pages dirtied since the last call to this ioctl
  auto ret = ioctl(machine_->vm_fd_, KVM_GET_DIRTY_LOG, &dirty);
  if (ret != 0) {
    if (errno != ENOENT) {
      MV_PANIC("KVM_GET_DIRTY_LOG failed");
    }
    return false;
  }
  return true;
}

bool MemoryManager::HandleBitmap(const char* bitmap, size_t size, DirtyBitmapCallback callback) {
  for (size_t i = 0; i < size; i++) {
    if (bitmap[i] == 0) {
      continue;
    }

    for (size_t j = 0; j < 8; j++) {
      if ((bitmap[i] & (1 << j)) == 0) {
        continue;
      }

      if (!callback(i * 8 + j)) {
        MV_ERROR("failed to handle bitmap");
        return false;
      }
    }
  }
  return true;
}

bool MemoryManager::SaveDirtyMemory(MigrationNetworkWriter* writer, DirtyMemoryType type) {
  MV_ASSERT(track_dirty_memory_);
  size_t dirty_memory_size = 0;
  DirtyMemoryDescriptor memory_descriptor;

  switch (type) {
    case kDirtyMemoryTypeKvm: {
      memory_descriptor.set_size(PAGE_SIZE);
      for (auto& slot : GetSlotsByNames(trace_slot_names_)) {
        // kvm memory dirty bitmap is 64-page aligned
        size_t bitmap_size = ALIGN((slot.end - slot.begin) / PAGE_SIZE, 64) / 8;
        std::string dirty_bitmap(bitmap_size, '\0');
        if (!GetDirtyBitmapFromKvm(slot.id, dirty_bitmap.data())) {
          continue;
        }

        auto ret = HandleBitmap(dirty_bitmap.data(), bitmap_size, [&](auto offset) {
          memory_descriptor.set_gpa(slot.begin + offset * PAGE_SIZE);
          if (memory_descriptor.gpa() + memory_descriptor.size() > slot.end) {
            MV_PANIC("Guest phyical address is out of current slot, gpa=0x%lx slot=%s", memory_descriptor.gpa(), slot.region->name);
          }
          if (!writer->WriteProtobuf("DIRTY_MEMORY_DESCRIPTOR", memory_descriptor)) {
            return false;
          }
          uint64_t hva = slot.hva + offset * PAGE_SIZE;
          if (!writer->WriteRaw("DIRTY_MEMORY", reinterpret_cast<void*>(hva), memory_descriptor.size())) {
            return false;
          }
          dirty_memory_size += memory_descriptor.size();
          return true;
        });

        if (!ret) {
          return false;
        }
      }
      break;
    }
    case kDirtyMemoryTypeListener: {
      memory_descriptor.set_size(PAGE_SIZE);
      for (auto listener : dirty_memory_listeners_) {
        auto dirty_bitmaps = listener->callback(kGetDirtyMemoryBitmap);
        for (auto& bitmap : dirty_bitmaps) {
          auto ret = HandleBitmap(bitmap.data.data(), bitmap.data.size(), [&](auto offset) {
            memory_descriptor.set_gpa(bitmap.region.begin + offset * PAGE_SIZE);
            if (memory_descriptor.gpa() + memory_descriptor.size() > bitmap.region.end) {
              MV_PANIC("Guest phyical address is out of current bitmap region, gpa=0x%lx", memory_descriptor.gpa());
            }
            if (!writer->WriteProtobuf("DIRTY_MEMORY_DESCRIPTOR", memory_descriptor)) {
              return false;
            }
            uint64_t hva = bitmap.region.hva + offset * PAGE_SIZE;
            if (!writer->WriteRaw("DIRTY_MEMORY", reinterpret_cast<void*>(hva), memory_descriptor.size())) {
              return false;
            }
            dirty_memory_size += memory_descriptor.size();
            return true;
          });

          if (!ret) {
            return false;
          }
        }
      }
      break;
    }
    case kDirtyMemoryTypeDma: {
      for (auto it = dirty_memory_regions_.begin(); it != dirty_memory_regions_.end(); ++it) {
        memory_descriptor.set_gpa(it->first);
        memory_descriptor.set_size(it->second);
        if (!writer->WriteProtobuf("DIRTY_MEMORY_DESCRIPTOR", memory_descriptor)) {
          return false;
        }
        auto hva = GuestToHostAddress(memory_descriptor.gpa());
        if (!writer->WriteRaw("DIRTY_MEMORY", hva, memory_descriptor.size())) {
          return false;
        }
        dirty_memory_size += memory_descriptor.size();
      }
      break;
    }
    default:
      MV_PANIC("not implemented");
      break;
  }

  // send finish header
  memory_descriptor.set_size(0);
  if (!writer->WriteProtobuf("DIRTY_MEMORY_DESCRIPTOR", memory_descriptor)) {
    return false;
  }

  MV_LOG("Save dirty memory type=%d size=%ldMB", type, dirty_memory_size >> 20);
  return true;
}

bool MemoryManager::LoadDirtyMemory(MigrationNetworkReader* reader, DirtyMemoryType type) {
  size_t dirty_memory_size = 0;
  DirtyMemoryDescriptor memory_descriptor;

  while (true) {
    reader->ReadProtobuf("DIRTY_MEMORY_DESCRIPTOR", memory_descriptor);
    if (memory_descriptor.size() == 0) {
      break;
    }

    auto hva = GuestToHostAddress(memory_descriptor.gpa());
    reader->ReadRaw("DIRTY_MEMORY", hva, memory_descriptor.size());
    dirty_memory_size += memory_descriptor.size();
  }

  MV_LOG("Load dirty memory type=%d size=%ldMB", type, dirty_memory_size >> 20);
  return true;
}

/* Used for debugging */
void MemoryManager::PrintMemoryScope() {
  std::shared_lock lock(mutex_);
  MV_LOG("%lu memory slots", kvm_slots_.size());
  for (auto it = kvm_slots_.begin(); it != kvm_slots_.end(); it++) {
    MemorySlot* slot = it->second;
    MV_LOG("Slot%3d %016lx-%016lx hva=%016lx %-10s %-10s",
      slot->id, slot->begin, slot->end, slot->hva,
      slot->region ? type_strings[slot->region->type] : "Unknown",
      slot->region ? slot->region->name : "(nil)");
  }
}

/* Used to build E820 table */
std::vector<const MemorySlot*> MemoryManager::GetMemoryFlatView() {
  std::vector<const MemorySlot*> slots;
  std::shared_lock lock(mutex_);
  for (auto it = kvm_slots_.begin(); it != kvm_slots_.end(); it++) {
    slots.push_back(it->second);
  }
  return slots;
}

/* Vfio device tracks the memory map */
const MemoryListener* MemoryManager::RegisterMemoryListener(MemoryListenerCallback callback) {
  auto listener = new MemoryListener {
    .callback = callback
  };
  std::unique_lock lock(mutex_);
  memory_listeners_.insert(listener);
  return listener;
}

void MemoryManager::UnregisterMemoryListener(const MemoryListener** plistener) {
  std::unique_lock lock(mutex_);
  if (memory_listeners_.erase(*plistener)) {
    delete *plistener;
    *plistener = nullptr;
  }
}

/* Vfio device tracks dirty memory */
const DirtyMemoryListener* MemoryManager::RegisterDirtyMemoryListener(DirtyMemoryListenerCallback callback) {
  auto listener = new DirtyMemoryListener {
    .callback = callback
  };
  std::unique_lock lock(mutex_);
  dirty_memory_listeners_.insert(listener);
  return listener;
}

void MemoryManager::UnregisterDirtyMemoryListener(const DirtyMemoryListener** plistener) {
  std::unique_lock lock(mutex_);
  if (dirty_memory_listeners_.erase(*plistener)) {
    delete *plistener;
    *plistener = nullptr;
  }
}

std::vector<MemorySlot> MemoryManager::GetSlotsByNames(std::unordered_set<std::string> names) {
  std::vector<MemorySlot> slots;
  for (auto it = kvm_slots_.begin(); it != kvm_slots_.end(); ++it) {
    auto slot = it->second;
    if (names.find(slot->region->name) != names.end()) {
      slots.push_back(*slot);
    }
  }
  return slots;
}

/* Save memory to migration */
bool MemoryManager::SaveState(MigrationWriter* writer) {
  writer->SetPrefix("memory");
  writer->WriteRaw("BIOS", bios_data_, bios_size_);
  writer->WriteMemoryPages("RAM", ram_host_, machine_->ram_size_);
  return true;
}

/* Reading memory data from migration */
bool MemoryManager::LoadState(MigrationReader* reader) {
  reader->SetPrefix("memory");
  if (!reader->ReadRaw("BIOS", bios_data_, bios_size_)) {
    return false;
  }

  // get all system memory region from kvm_slots_
  auto system_slots = GetSlotsByNames({"System"});

  // unmap all system memory region
  for (auto it = system_slots.begin(); it != system_slots.end(); ++it) {
    Unmap((const MemoryRegion **)&it->region);
  }

  /* Map the RAM file as copy on write memory */
  if (!reader->ReadMemoryPages("RAM", &ram_host_, machine_->ram_size_)) {
    return false;
  }

  // reset system memory region
  for (auto it = system_slots.begin(); it != system_slots.end(); ++it) {
    Map(it->begin, it->end - it->begin, (void*)it->hva, kMemoryTypeRam, "System");
  }

  return true;
}
