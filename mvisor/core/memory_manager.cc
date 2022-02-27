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
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cstring>
#include <linux/kvm.h>
#include <unordered_set>
#include "machine.h"
#include "logger.h"

static uint32_t _new_slot_id = 0;
static inline uint32_t get_new_slot_id() {
  return _new_slot_id++;
}

static void kvm_set_user_memory_region(int vm_fd, uint32_t slot, uint64_t gpa,
    uint64_t size, uint64_t hva, uint32_t flags) {

  struct kvm_userspace_memory_region memreg;
  memreg.slot = slot;
  memreg.flags = flags;
  memreg.guest_phys_addr = gpa;
  memreg.memory_size = size;
  memreg.userspace_addr = hva; 

  if (ioctl(vm_fd, KVM_SET_USER_MEMORY_REGION, &memreg) < 0) {
    MV_PANIC("failed to set user memory region slot=%d gpa=%016lx size=%016lx hva=%016lx flags=%d",
      slot, gpa, size, hva, flags);
  }
}

MemoryManager::MemoryManager(const Machine* machine)
    : machine_(machine) {

  InitializeSystemRam();
}

MemoryManager::~MemoryManager() {
  munmap(ram_host_, machine_->ram_size_);
}


/* Allocate system ram for guest */
void MemoryManager::InitializeSystemRam() {
  if (machine_->debug_)
    MV_LOG("RAM size: %lu MB", machine_->ram_size_ >> 20);

  ram_host_ = mmap(nullptr, machine_->ram_size_, PROT_READ | PROT_WRITE,
    MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  MV_ASSERT(ram_host_ != MAP_FAILED);

  /* Don't map MMIO region */
  const uint64_t low_ram_upper_bound = 2 * (1LL << 30);
  const uint64_t high_ram_lower_bound = 1LL << 32;
  if (machine_->ram_size_ <= low_ram_upper_bound) {
    Map(0, machine_->ram_size_, ram_host_, kMemoryTypeRam, "free");
  } else {
    // Split the ram to two segments leaving a hole in the GPA
    Map(0, low_ram_upper_bound, ram_host_, kMemoryTypeRam, "free");
    // Skip the hole and map the rest
    Map(high_ram_lower_bound, machine_->ram_size_ - low_ram_upper_bound,
      (uint8_t*)ram_host_ + low_ram_upper_bound, kMemoryTypeRam, "free");
  }
}

/* Don't call this funciton, use Map and Unmap */
void MemoryManager::AddMemoryRegion(MemoryRegion* region) {
  std::unordered_set<MemorySlot*> pending_slots;

  MemorySlot* slot = new MemorySlot;
  slot->slot = get_new_slot_id();
  slot->region = region;
  slot->begin = region->gpa;
  slot->end = region->gpa + region->size;
  slot->hva = reinterpret_cast<uint64_t>(region->host);
  pending_slots.insert(slot);
  
  // Find all overlapped slots, split them delete the old ones (resizing is not supported by KVM)
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
        left->slot = get_new_slot_id();
      }
      if (slot->end < hit->end) {
        // Right collision
        right = new MemorySlot(*hit);
        right->begin = slot->end;
        right->hva = reinterpret_cast<uint64_t>(right->region->host) + (right->begin - right->region->gpa);
        right->slot = get_new_slot_id();
      }
      // Move the iterator before we change kvm_slots
      ++it;
      // Replace the hit slot with left and right
      if (left) {
        pending_slots.insert(left);
        kvm_slots_[left->begin] = left;
      }
      if (right) {
        pending_slots.insert(right);
        kvm_slots_[right->begin] = right;
      }
      
      if (pending_slots.find(hit) != pending_slots.end()) {
        // Second collision happened, just remove the previous created
        pending_slots.erase(hit);
      } else {
        if (hit->region->type == kMemoryTypeRam) {
          kvm_set_user_memory_region(machine_->vm_fd_, hit->slot, hit->begin,
            0, hit->hva, hit->region->flags);
        }
        // tell listeners we removed a slot
        for (auto listener : listeners_) {
          listener->callback(hit, true);
        }
      }
      delete hit;
    } else {
      ++it;
    }
  }
  // Finally add the new slot
  kvm_slots_[slot->begin] = slot;
  regions_.insert(region);

  // Commit the pending slots to KVM
  for (auto slot : pending_slots) {
    if (slot->region->type == kMemoryTypeRam) {
      kvm_set_user_memory_region(machine_->vm_fd_, slot->slot, slot->begin,
        slot->end - slot->begin, slot->hva, slot->region->flags);
      // tell listeners we have new slots
      for (auto listener : listeners_) {
        listener->callback(slot, false);
      }
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
  region->flags = 0;
  strncpy(region->name, name, 20 - 1);

  if (machine_->debug_)
    MV_LOG("map region %s gpa=0x%lx size=0x%lx type=0x%x", region->name,
      region->gpa, region->size, region->type);
  
  AddMemoryRegion(region);
  return region;
}

/* TODO: should merge the slots after unmap */
void MemoryManager::Unmap(const MemoryRegion** pregion) {
  auto region = (MemoryRegion*)*pregion;
  if (machine_->debug_)
    MV_LOG("unmap region %s gpa=0x%lx size=%lx type=%x", region->name,
      region->gpa, region->size, region->type);

  // Remove KVM slots
  for (auto it = kvm_slots_.begin(); it != kvm_slots_.end(); ) {
    auto slot = it->second;
    if (slot->region == region) {
      if (region->type == kMemoryTypeRam) {
        // Remvoe kvm slot
        kvm_set_user_memory_region(machine_->vm_fd_, slot->slot, slot->begin, 0, slot->hva, slot->region->flags);
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
  static const char* type_strings[] = { "reserved", "ram", "device" };
  MV_LOG("%lu memory slots", kvm_slots_.size());
  for (auto it = kvm_slots_.begin(); it != kvm_slots_.end(); it++) {
    MemorySlot* slot = it->second;
    MV_LOG("memory slot=%d %016lx-%016lx hva=%016lx %10s %10s",
      slot->slot, slot->begin, slot->end, slot->hva, type_strings[slot->region->type],
      slot->region->name);
  }
}

std::vector<const MemorySlot*> MemoryManager::GetMemoryFlatView() {
  std::vector<const MemorySlot*> slots;
  for (auto it = kvm_slots_.begin(); it != kvm_slots_.end(); it++) {
    slots.push_back(it->second);
  }
  return slots;
}

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
