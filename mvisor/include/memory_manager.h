#ifndef _MVISOR_MM_H
#define _MVISOR_MM_H

#include <map>
#include <set>

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
  char name[20];
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

  const MemoryRegion* Map(uint64_t gpa, uint64_t size, void* host, MemoryType type, const char* name);
  void Unmap(const MemoryRegion** region);

  void PrintMemoryScope();
  void* GuestToHostAddress(uint64_t gpa);
  uint64_t HostToGuestAddress(void* host);

  const std::set<MemoryRegion*>& regions() const { return regions_; }

 private:
  void InitializeSystemRam();
  void AddMemoryRegion(MemoryRegion* region);

  const Machine* machine_;
  std::set<MemoryRegion*> regions_;
  std::map<uint64_t, KvmSlot*> kvm_slots_;
  void* ram_host_;
};

#endif // _MVISOR_MM_H
