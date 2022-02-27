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

#ifndef _MVISOR_MM_H
#define _MVISOR_MM_H

#include <map>
#include <set>
#include <vector>
#include <functional>

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

struct MemorySlot {
  uint64_t begin;
  uint64_t end;
  uint32_t slot;
  uint64_t hva;
  MemoryRegion* region;
};

typedef std::function<void (const MemorySlot* slot, bool unmap)> MemoryListenerCallback;
struct MemoryListener {
  MemoryListenerCallback callback;
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
  std::vector<const MemorySlot*> GetMemoryFlatView();
  const MemoryListener* RegisterMemoryListener(MemoryListenerCallback callback);
  void UnregisterMemoryListener(const MemoryListener** plistener);

  const std::set<MemoryRegion*>& regions() const { return regions_; }

 private:
  void InitializeSystemRam();
  void AddMemoryRegion(MemoryRegion* region);

  const Machine* machine_;
  void* ram_host_;
  std::set<MemoryRegion*> regions_;
  std::map<uint64_t, MemorySlot*> kvm_slots_;
  std::set<const MemoryListener*> listeners_;
};

#endif // _MVISOR_MM_H
