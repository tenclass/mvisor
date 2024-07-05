#include "memory_region.h"
#include "machine.h"

static const char* type_strings[] = {
  "Reserved", 
  "RAM", 
  "Device", 
  "ROM", 
  "Unknown" 
};

const std::string MemoryRegion::type_name() const {
  return type_ < sizeof(type_strings) / sizeof(type_strings[0]) ? type_strings[type_] : type_strings[sizeof(type_strings) - 1];
}


MemoryRegion::MemoryRegion(uint64_t gpa, uint64_t size, void* host, MemoryType type, const char* name)
    : gpa_(gpa), host_(host), size_(size), type_(type), name_(name) {
  flags_ = type == kMemoryTypeRom ? KVM_MEM_READONLY : 0;
  is_system_ = type_ == kMemoryTypeRam && name_ == "System";
}


bool MemoryRegion::ForeachDirtyPage(std::function<bool (uint64_t offset)> callback) {
  for (auto slot: slots_) {
    auto start_i = (slot->begin - gpa_) / PAGE_SIZE;
    auto end_i = (slot->end - gpa_) / PAGE_SIZE;
    for (uint64_t i = start_i; i < end_i; i++) {
      auto byte = dirty_bitmap_[i / 8];
      if (byte == 0) {
        continue;
      }
      
      for (uint64_t j = 0; j < 8; j++) {
        if (byte & (1 << j)) {
          uint64_t offset = (i * 8 + j) * PAGE_SIZE;
          if (!callback(offset)) {
            return false;
          }
        }
      }
    }
  }
  return true;
}

bool MemoryRegion::IsDirty(uint64_t offset) {
  auto i = (offset - gpa_) / PAGE_SIZE;
  auto byte = dirty_bitmap_[i / 8];
  return byte & (1 << (i % 8));
}
