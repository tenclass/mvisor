#ifndef _MVISOR_MM_H
#define _MVISOR_MM_H

#include <map>
#include <vector>

enum MemoryType {
  kMemoryTypeReserved = 0,
  kMemoryTypeRam = 1,
  kMemoryTypeDevice = 2
};

struct MemoryRegion {
  uint64_t gpa;
  void* host;
  uint64_t size;
  uint32_t flags;
  MemoryType type;
};

struct KvmSlot {
  uint64_t begin;
  uint64_t end;
  uint32_t slot;
  uint64_t hva;
  MemoryRegion* region;
};

class Machine;
class MemoryManager {
 public:
  MemoryManager(const Machine* machine);
  ~MemoryManager();
  // Add to and remove from regions_
  const MemoryRegion* Map(uint64_t gpa, uint64_t size, void* host, MemoryType type);
  void Unmap(const MemoryRegion* region);
  // Commit regions to kvm_slots_
  void Commit();
  void PrintMemoryScope();
  void* GuestToHostAddress(uint64_t gpa);
  uint64_t HostToGuestAddress(void* host);

 private:
  void InitializeSystemRam();
  const Machine* machine_;
  std::vector<MemoryRegion*> cached_regions_;
  std::vector<MemoryRegion*> committed_regions_;
  std::map<uint64_t, KvmSlot*> kvm_slots_;
  void* reserved_ram_;
};

#endif // _MVISOR_MM_H
