#ifndef _MVISOR_MEMORY_REGION_H
#define _MVISOR_MEMORY_REGION_H

#include <cstdint>
#include <vector>
#include <string>
#include <unordered_set>
#include <functional>

#include <linux/kvm.h>

enum MemoryType {
  kMemoryTypeReserved = 0,
  kMemoryTypeRam = 1,
  kMemoryTypeDevice = 2,
  kMemoryTypeRom = 3
};

class MemorySlot;
class MemoryRegion {
 private:
  uint64_t      gpa_;
  void*         host_;
  uint64_t      size_;
  uint32_t      flags_;
  MemoryType    type_;
  std::string   name_;
  std::vector<uint8_t>            dirty_bitmap_;
  std::unordered_set<MemorySlot*> slots_;
  bool          is_system_;

  friend class MemoryManager;

 public:
  MemoryRegion(uint64_t gpa, uint64_t size, void* host, MemoryType type, const char* name);
  bool ForeachDirtyPage(std::function<bool (uint64_t offset)> callback);
  bool IsDirty(uint64_t offset, uint64_t length);

  uint64_t gpa() const { return gpa_; }
  void* host() const { return host_; }
  uint64_t size() const { return size_; }
  uint32_t flags() const { return flags_; }
  MemoryType type() const { return type_; }
  const std::string& name() const { return name_; }
  const std::string  type_name() const;
  bool is_system() { return is_system_; }
};

struct MemorySlot {
  MemoryType    type;
  uint64_t      begin;
  uint64_t      end;
  uint64_t      size;
  uint64_t      hva;
  uint32_t      id;
  uint32_t      flags;
  MemoryRegion* region;

  bool commitable() const { return type == kMemoryTypeRam || type == kMemoryTypeRom; }
  bool is_system() const { return region && region->is_system(); }
};

#endif // _MVISOR_MEMORY_REGION_H
