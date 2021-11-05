#include "memory_manager.h"
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cstring>
#include <linux/kvm.h>
#include <unordered_set>
#include "machine.h"
#include "logger.h"

MemoryManager::MemoryManager(const Machine* machine)
    : machine_(machine) {
  InitializeSystemRam();
}

MemoryManager::~MemoryManager() {
  munmap(ram_host_, machine_->ram_size_);
}

void MemoryManager::InitializeSystemRam() {
  // Add reserved memory region
  MV_LOG("ram size: %lu MB", machine_->ram_size_ >> 20);
  ram_host_ = mmap(nullptr, machine_->ram_size_, PROT_READ | PROT_WRITE,
    MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
  MV_ASSERT(ram_host_ != MAP_FAILED);

  // Don't map MMIO region
  const uint64_t low_ram_upper_bound = 2 * (1LL << 30);
  const uint64_t high_ram_lower_bound = 1LL << 32;
  if (machine_->ram_size_ < low_ram_upper_bound) {
    Map(0, machine_->ram_size_, ram_host_, kMemoryTypeRam, "free");
  } else {
    // Split the ram to two segments leaving a hole in the GPA
    Map(0, low_ram_upper_bound, ram_host_, kMemoryTypeRam, "free");
    // Skip the hole and map the rest
    Map(high_ram_lower_bound, machine_->ram_size_ - low_ram_upper_bound,
      (uint8_t*)ram_host_ + low_ram_upper_bound, kMemoryTypeRam, "free");
  }
}

const MemoryRegion* MemoryManager::Map(uint64_t gpa, uint64_t size, void* host, MemoryType type, const char* name) {
  MemoryRegion* region = new MemoryRegion;
  region->gpa = gpa;
  region->host = host;
  region->size = size;
  region->type = type;
  region->flags = 0;
  strncpy(region->name, name, 20 - 1);
  
  AddMemoryRegion(region);
  return region;
}

void MemoryManager::Unmap(const MemoryRegion* region) {
  MV_PANIC("not implemented");
}

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
  // MV_LOG("set mem slot %d %016lx-%016lx to %p", slot, gpa, gpa + size, hva);
}

void MemoryManager::AddMemoryRegion(MemoryRegion* region) {
  KvmSlot* slot = new KvmSlot;
  slot->slot = get_new_slot_id();
  slot->region = region;
  slot->begin = region->gpa;
  slot->end = region->gpa + region->size;
  slot->hva = reinterpret_cast<uint64_t>(region->host);
  pending_slots_.insert(slot);
  
  // Find the first one whose gpa is smaller than slot->gpa
  auto it = kvm_slots_.upper_bound(slot->begin);
  // Make sure it->second->gpa < begin
  if (it != kvm_slots_.begin()) {
    --it;
  }
  // Find all overlapped slots, split them delete the old ones
  // Maybe later we should support priorities
  while (it != kvm_slots_.end() && it->second->begin < slot->end) {
    if (it->second->begin < slot->end && slot->begin < it->second->end) {
      KvmSlot *hit = it->second;
      KvmSlot *left = nullptr, *right = nullptr;
  
      // Collision found, split the slot
      if (hit->begin < slot->begin) {
        // Left collision
        left = new KvmSlot(*hit);
        left->end = slot->begin;
        left->slot = get_new_slot_id();
      }
      if (slot->end < hit->end) {
        // Right collision
        right = new KvmSlot(*hit);
        right->begin = slot->end;
        right->hva = reinterpret_cast<uint64_t>(right->region->host) + (right->begin - right->region->gpa);
        right->slot = get_new_slot_id();
      }
      // Move the iterator before we change kvm_slots
      ++it;
      // Replace the hit slot with left and right
      if (left) {
        pending_slots_.insert(left);
        kvm_slots_[left->begin] = left;
      }
      if (right) {
        pending_slots_.insert(right);
        kvm_slots_[right->begin] = right;
      }
      
      if (pending_slots_.find(hit) != pending_slots_.end()) {
        pending_slots_.erase(hit);
      } else {
        kvm_set_user_memory_region(machine_->vm_fd_, hit->slot, hit->begin,
          0, hit->hva, hit->region->flags);
      }
      delete hit;
    } else {
      ++it;
    }
  }
  // Finally add the new slot
  kvm_slots_[slot->begin] = slot;
  regions_.push_back(region);

  if (!map_transaction_enabled_) {
    Commit();
  }
}

void MemoryManager::Commit() {
  for (auto slot : pending_slots_) {
    kvm_set_user_memory_region(machine_->vm_fd_, slot->slot, slot->begin,
      slot->end - slot->begin, slot->hva, slot->region->flags);
  }
  pending_slots_.clear();
}

void MemoryManager::PrintMemoryScope() {
  static const char* type_strings[] = { "reserved", "ram", "device" };
  MV_LOG("%lu memory slots", kvm_slots_.size());
  for (auto it = kvm_slots_.begin(); it != kvm_slots_.end(); it++) {
    KvmSlot* slot = it->second;
    MV_LOG("memory slot=%d %016lx-%016lx hva=%016lx %10s %10s",
      slot->slot, slot->begin, slot->end, slot->hva, type_strings[slot->region->type],
      slot->region->name);
  }
}

void* MemoryManager::GuestToHostAddress(uint64_t gpa) {
  // Find the first slot whose begin is smaller than gpa
  auto it = kvm_slots_.upper_bound(gpa);
  if (it != kvm_slots_.begin()) {
    --it;
  }
  MV_ASSERT(it != kvm_slots_.end());

  KvmSlot* slot = it->second;
  if (gpa >= slot->begin && gpa < slot->end) {
    uint64_t address = slot->hva + gpa - slot->begin;
    return reinterpret_cast<void*>(address);
  }
  MV_PANIC("failed to translate guest physical address 0x%016lx", gpa);
  return nullptr;
}

uint64_t MemoryManager::HostToGuestAddress(void* host) {
  MV_PANIC("not implemented");
  return 0;
}


void MemoryManager::BeginMapTransaction() {
  map_transaction_enabled_ = true;
}

void MemoryManager::EndMapTransaction() {
  Commit();
  map_transaction_enabled_ = false;
}
