/* 
 * MVisor VFIO Manager
 * Copyright (C) 2024 Terrence <terrence@tenclass.com>
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

#ifndef _MVISOR_VFIO_MANAGER_H
#define _MVISOR_VFIO_MANAGER_H

#include <map>
#include <set>
#include <string>

#include "memory_manager.h"
#include "linuz/vfio.h"

struct IommuGroup {
  int group_id;
  int group_fd;
  std::set<int> device_fds;
};


class Machine;
class VfioManager {
 private:
  Machine                            *machine_;
  int                                 kvm_device_fd_ = -1;
  int                                 container_fd_ = -1;
  std::map<int, IommuGroup*>          iommu_groups_;
  const DirtyMemoryListener*                            dirty_memory_listener_ = nullptr;
  const MemoryListener*                                 memory_listener_ = nullptr;
  std::map<const MemorySlot*, vfio_iommu_type1_dma_map> iommu_dma_maps_;

  IommuGroup* GetIommuGroup(std::string sysfs_path);
  void FreeIommuGroup(IommuGroup* group);
  void SetupDmaMaps();
  void MapDmaPages(const MemorySlot* slot);
  void UnmapDmaPages(const MemorySlot* slot);
 public:
  VfioManager(Machine* machine);
  ~VfioManager();

  int AttachDevice(std::string sysfs_path);
  void DetachDevice(int device_fd);
};


#endif // _MVISOR_VFIO_MANAGER_H
