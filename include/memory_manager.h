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
#include <shared_mutex>
#include <unordered_set>

#include "migration.h"
#include "memory_region.h"


typedef std::function<void (const MemorySlot slot, bool unmap)> MemoryListenerCallback;
struct MemoryListener {
  MemoryListenerCallback callback;
};

enum DirtyMemoryCommand {
  kGetDirtyMemoryBitmap = 0,
  kStopTrackingDirtyMemory = 1,
  kStartTrackingDirtyMemory = 2
};

enum DirtyMemoryType {
  kDirtyMemoryTypeKvm = 0,
  kDirtyMemoryTypeListener = 1,
  kDirtyMemoryTypeDma = 2
};

typedef std::function<std::vector<struct DirtyMemoryBitmap> (const DirtyMemoryCommand command)> DirtyMemoryListenerCallback;
struct DirtyMemoryListener {
  DirtyMemoryListenerCallback callback;
};

typedef std::function<bool (size_t offset)> DirtyBitmapCallback;

class Machine;
class MemoryManager {
 public:
  MemoryManager(Machine* machine);
  ~MemoryManager();

  MemoryRegion* Map(uint64_t gpa, uint64_t size, void* host, MemoryType type, const char* name);
  void Unmap(MemoryRegion** region);
  void Reset();

  /* Used for migration */
  bool SaveState(MigrationWriter* writer);
  bool LoadState(MigrationReader* reader);

  void SetLogDirtyBitmap(MemoryRegion* region, bool log);
  void SynchronizeDirtyBitmap(MemoryRegion* region);
  void SetDirtyMemoryRegion(uint64_t gpa, size_t size);
  void StartTrackingDirtyMemory();
  void StopTrackingDirtyMemory();

  bool SaveDirtyMemory(MigrationNetworkWriter* writer, DirtyMemoryType type);
  bool LoadDirtyMemory(MigrationNetworkReader* reader, DirtyMemoryType type);

  const DirtyMemoryListener* RegisterDirtyMemoryListener(DirtyMemoryListenerCallback callback);
  void UnregisterDirtyMemoryListener(const DirtyMemoryListener** plistener);

  void PrintMemoryScope();
  void* GuestToHostAddress(uint64_t gpa);
  uint64_t HostToGuestAddress(void* host);
  std::vector<MemorySlot> GetMemoryFlatView();
  const MemoryListener* RegisterMemoryListener(MemoryListenerCallback callback);
  void UnregisterMemoryListener(const MemoryListener** plistener);

  const std::vector<MemoryRegion*>& regions() const { return regions_; }

 private:
  void InitializeSystemRam();
  void InitializeReservedMemory();
  void LoadBiosFile();
  void CommitMemoryRegionChanges();
  void NotifySlotChange(const MemorySlot* slot, bool unmap);
  void UpdateKvmSlot(MemorySlot* slot, bool remove);
  uint AllocateSlotId();
  bool GetDirtyBitmapFromKvm(uint32_t slot, void* bitmap);
  bool HandleBitmap(const char* bitmap, size_t size, DirtyBitmapCallback callback);

  Machine*                        machine_;
  void*                           system_ram_host_;
  std::vector<MemoryRegion*>      system_regions_;
  std::vector<MemoryRegion*>      regions_;
  std::map<uint64_t, MemorySlot*> kvm_slots_;
  std::set<const MemoryListener*> memory_listeners_;
  std::set<uint>                  free_slots_;
  std::unordered_set<std::string> trace_slot_names_;
  mutable std::shared_mutex       mutex_;
  
  /* BIOS data */
  size_t                          bios_size_;
  void*                           bios_data_ = nullptr;
  void*                           bios_backup_ = nullptr;
  const MemoryRegion*             bios_region_ = nullptr;

  bool                                 track_dirty_memory_ = false;
  std::mutex                           dirty_memory_region_mutex_;
  std::map<uint64_t, size_t>           dirty_memory_regions_;
  std::set<const DirtyMemoryListener*> dirty_memory_listeners_;
};

#endif // _MVISOR_MM_H
