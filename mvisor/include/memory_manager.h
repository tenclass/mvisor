#ifndef _MVISOR_MM_H
#define _MVISOR_MM_H

#include <map>
#include <vector>
#include <unordered_set>

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
  void InitializeSystemRam();

  // Add to and remove from regions_
  const MemoryRegion* Map(uint64_t gpa, uint64_t size, void* host, MemoryType type);
  void Unmap(const MemoryRegion* region);
  void BeginMapTransaction();
  void EndMapTransaction();

  void PrintMemoryScope();
  void* GuestToHostAddress(uint64_t gpa);
  uint64_t HostToGuestAddress(void* host);

  const std::vector<MemoryRegion*>& regions() const { return regions_; }
 private:
  void Commit();
  void AddMemoryRegion(MemoryRegion* region);
  const Machine* machine_;
  std::vector<MemoryRegion*> regions_;

  std::unordered_set<KvmSlot*> pending_slots_;
  std::map<uint64_t, KvmSlot*> kvm_slots_;
  void* ram_host_;
  bool map_transaction_enabled_ = false;
};

#endif // _MVISOR_MM_H
