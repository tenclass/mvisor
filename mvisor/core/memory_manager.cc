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

#include <unordered_set>

#include "machine.h"
#include "logger.h"


#define X86_EPT_IDENTITY_BASE 0xFEFFC000


MemoryManager::MemoryManager(const Machine* machine)
    : machine_(machine) {
  
  /* Get the number of slots we can allocate */
  uint max_slots = ioctl(machine_->kvm_fd_, KVM_CHECK_EXTENSION, KVM_CAP_NR_MEMSLOTS);
  for (uint i = 0; i < max_slots; i++) {
    free_slots_.insert(i);
  }

  /* Setup the system memory map for BIOS to run */
  InitializeSystemRam();
  InitializeReservedMemory();
  LoadBiosFile();
}

MemoryManager::~MemoryManager() {
  munmap(ram_host_, machine_->ram_size_);
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
    MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
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
  // Read BIOS data from path to bios_data
  int fd = open(machine_->config_->bios_path().c_str(), O_RDONLY);
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
      
      if (pending_add.find(hit) != pending_add.end()) {
        // Second collision happened, just remove the previous created
        pending_add.erase(hit);
        delete hit;
      } else {
        hit->region = nullptr;
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
    for (auto listener : listeners_) {
      listener->callback(slot, true);
    }
    delete slot;
  }
  for (auto slot : pending_add) {
    if (slot->type == kMemoryTypeRam || slot->type == kMemoryTypeRom) {
      UpdateKvmSlot(slot, false);
    }
    // tell listeners we have new slots
    for (auto listener : listeners_) {
      listener->callback(slot, false);
    }
  }
}

/* Mapping a memory region in the guest address space */
const MemoryRegion* MemoryManager::Map(uint64_t gpa, uint64_t size, void* host, MemoryType type, const char* name) {
  MemoryRegion* region = new MemoryRegion;
  region->gpa = gpa;
  region->host = host;
  region->size = size;
  region->type = type;
  region->flags = type == kMemoryTypeRom ? KVM_MEM_READONLY : 0;
  strncpy(region->name, name, 20 - 1);

  if (machine_->debug_) {
    MV_LOG("map region %s gpa=0x%lx size=0x%lx type=0x%x", region->name,
      region->gpa, region->size, region->type);
  }
  
  AddMemoryRegion(region);
  return region;
}

/* TODO: should merge the slots after unmap */
void MemoryManager::Unmap(const MemoryRegion** pregion) {
  auto region = (MemoryRegion*)*pregion;
  if (machine_->debug_) {
    MV_LOG("unmap region %s gpa=0x%lx size=%lx type=%x", region->name,
      region->gpa, region->size, region->type);
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
      for (auto listener : listeners_) {
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

/* Not used yet */
uint64_t MemoryManager::HostToGuestAddress(void* host) {
  MV_PANIC("not implemented");
  return 0;
}

/* Used for debugging */
void MemoryManager::PrintMemoryScope() {
  static const char* type_strings[] = { "Reserved", "RAM", "Device", "ROM", "Unknown" };
  std::shared_lock lock(mutex_);
  MV_LOG("%lu memory slots", kvm_slots_.size());
  for (auto it = kvm_slots_.begin(); it != kvm_slots_.end(); it++) {
    MemorySlot* slot = it->second;
    MV_LOG("Slot%3d %016lx-%016lx hva=%016lx %-10s %-10s",
      slot->id, slot->begin, slot->end, slot->hva, type_strings[slot->region->type],
      slot->region->name);
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
  listeners_.insert(listener);
  return listener;
}

void MemoryManager::UnregisterMemoryListener(const MemoryListener** plistener) {
  if (listeners_.erase(*plistener)) {
    delete *plistener;
    *plistener = nullptr;
  }
}

/* Save memory to file */
bool MemoryManager::SaveState(MigrationWriter* writer) {
  writer->SetPrefix("memory");
  writer->WriteRaw("BIOS", bios_data_, bios_size_);

  /* Write RAM to sparse file */
  int fd = writer->BeginWrite("RAM");
  ftruncate(fd, machine_->ram_size_);

  auto ptr = (uint8_t*)ram_host_;
  for (size_t pos = 0; pos < machine_->ram_size_; pos += PAGE_SIZE) {
    if (!test_zero(ptr, PAGE_SIZE)) {
      pwrite(fd, ptr, PAGE_SIZE, pos);
    }
    ptr += PAGE_SIZE;
  }
  writer->EndWrite("RAM");
  return true;
}

/* Reading memory data from file */
bool MemoryManager::LoadState(MigrationReader* reader) {
  reader->SetPrefix("memory");
  if (!reader->ReadRaw("BIOS", bios_data_, bios_size_)) {
    return false;
  }

  /* Unmap the preallocated */
  munmap(ram_host_, machine_->ram_size_);

  /* Map the RAM file as copy on write memory */
  int fd = reader->BeginRead("RAM");
  ram_host_ = mmap(nullptr, machine_->ram_size_, PROT_READ | PROT_WRITE, MAP_PRIVATE, fd, 0);
  if (ram_host_ == MAP_FAILED) {
    MV_PANIC("failed to map memory");
  }
  reader->EndRead("RAM");

  /* Make host RAM DONTDUMP */
  madvise(ram_host_, machine_->ram_size_, MADV_DONTDUMP);
  return true;
}
