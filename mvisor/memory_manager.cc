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
  munmap(reserved_ram_, machine_->ram_size_);
}

void MemoryManager::InitializeSystemRam() {
  // Add reserved memory region
  MV_LOG("ram size: %lu MB", machine_->ram_size_ >> 20);
  reserved_ram_ = mmap(nullptr, machine_->ram_size_, PROT_READ | PROT_WRITE,
    MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
  MV_ASSERT(reserved_ram_ != MAP_FAILED);

  // Don't map MMIO region
  const uint64_t high_memory_start = 1LL << 32;
  const uint64_t mmio_size = 768 << 20;
  const uint64_t mmio_start = high_memory_start - mmio_size;
  if (machine_->ram_size_ < mmio_start) {
    Map(0, machine_->ram_size_, reserved_ram_, kMemoryTypeReserved);
  } else {
    Map(0, mmio_start, reserved_ram_, kMemoryTypeReserved);
    // Skip the gap and map the rest
    Map(high_memory_start, machine_->ram_size_ - mmio_start,
      (uint8_t*)reserved_ram_ + mmio_start, kMemoryTypeReserved);
  }
}

const MemoryRegion* MemoryManager::Map(uint64_t gpa, uint64_t size, void* host, MemoryType type) {
  MemoryRegion* region = new MemoryRegion;
  region->gpa = gpa;
  region->host = host;
  region->size = size;
  region->type = type;
  region->flags = 0;
  cached_regions_.push_back(region);
  return region;
}

void MemoryManager::Unmap(const MemoryRegion* region) {
  MV_PANIC("not implemented");
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

static uint32_t _new_slot_id = 0;
static inline uint32_t get_new_slot_id() {
  return _new_slot_id++;
}

void MemoryManager::Commit() {
  std::unordered_set<KvmSlot*> to_add, to_remove;

  for (auto region : cached_regions_) {
    KvmSlot* slot = new KvmSlot;
    slot->slot = get_new_slot_id();
    slot->region = region;
    slot->begin = region->gpa;
    slot->end = region->gpa + region->size;
    slot->hva = reinterpret_cast<uint64_t>(region->host);
    to_add.insert(slot);
    
    // Find the lower bound and iterates
    auto it = kvm_slots_.upper_bound(slot->begin);
    // Make sure it->second->gpa < begin
    if (it != kvm_slots_.begin()) {
      --it;
    }

    while (it != kvm_slots_.end() && it->second->begin < slot->end) {
      if (it->second->begin < slot->end && slot->begin < it->second->end) {
        KvmSlot *hit = it->second;
        KvmSlot *left = nullptr, *right = nullptr;
        MV_ASSERT(hit->region->type == kMemoryTypeReserved);
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
        ++it;
        if (left) {
          to_add.insert(left);
          kvm_slots_[left->begin] = left;
        }
        if (right) {
          to_add.insert(right);
          kvm_slots_[right->begin] = right;
        }
        to_remove.insert(hit);
      } else {
        ++it;
      }
    }
    kvm_slots_[slot->begin] = slot;
    committed_regions_.push_back(region);
  }

  for (auto slot : to_remove) {
    if (to_add.find(slot) == to_add.end()) {
      kvm_set_user_memory_region(machine_->vm_fd_, slot->slot, slot->begin,
        0, slot->hva, slot->region->flags);
    }
    delete slot;
  }

  for (auto slot : to_add) {
    if (to_remove.find(slot) == to_remove.end()) {
      kvm_set_user_memory_region(machine_->vm_fd_, slot->slot, slot->begin,
        slot->end - slot->begin, slot->hva, slot->region->flags);
    }
  }

  cached_regions_.clear();
  PrintMemoryScope();
}

void MemoryManager::PrintMemoryScope() {
  static const char* type_strings[] = { "reserved", "ram", "device" };
  MV_LOG("%lu memory slots", kvm_slots_.size());
  for (auto it = kvm_slots_.begin(); it != kvm_slots_.end(); it++) {
    KvmSlot* slot = it->second;
    MV_LOG("memory slot=%d %016lx-%016lx hva=%016lx %s",
      slot->slot, slot->begin, slot->end, slot->hva, type_strings[slot->region->type]);
  }
}

void* MemoryManager::GuestToHostAddress(uint64_t gpa) {
  // Find the lower bound and iterates
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
