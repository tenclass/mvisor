/* 
 * MVisor VFIO for vGPU
 * Copyright (C) 2022 Terrence <terrence@tenclass.com>
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

#ifndef _MVISOR_DEVICES_VFIO_VFIO_PCI_H
#define _MVISOR_DEVICES_VFIO_VFIO_PCI_H

#include <cstdint>
#include <string>
#include "linuz/vfio.h"
#include "pci_device.h"
#include "memory_manager.h"

#define MAX_VFIO_REGIONS      (12)
#define MAX_VFIO_INTERRUPTS   (PCI_MAX_MSIX_ENTRIES)

struct VfioMmapArea {
  uint64_t  offset;
  uint64_t  size;
  void*     mmap;
};

struct VfioRegion {
  uint32_t  index;
  uint32_t  flags;
  uint64_t  offset;
  uint64_t  size;
  uint32_t  type;
  uint32_t  subtype;
  std::vector<VfioMmapArea> mmap_areas;
};

struct VfioInterrupt {
  int       event_fd;
  int       gsi;
};

class VfioPci : public PciDevice {
 public:
  VfioPci();
  virtual ~VfioPci();
  virtual void Connect();
  virtual void Disconnect();
  virtual void Reset();
  virtual bool ActivatePciBar(uint8_t index);
  virtual bool DeactivatePciBar(uint8_t index);
  
  virtual void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length);
  virtual void ReadPciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length);

 protected:
  void SetupVfioGroup();
  void SetupVfioDevice();
  void SetupVfioContainer();
  void SetupPciConfiguration();
  void SetupPciInterrupts();
  void SetupGfxPlane();
  void SetupDmaMaps();
  void UpdateMsiRoutes();
  void MapDmaPages(const MemorySlot* slot);
  void UnmapDmaPages(const MemorySlot* slot);
  void MapBarRegion(uint8_t index);
  void UnmapBarRegion(uint8_t index);
  ssize_t ReadRegion(uint8_t index, uint64_t offset, uint8_t* data, uint32_t length);
  ssize_t WriteRegion(uint8_t index, uint64_t offset, uint8_t* data, uint32_t length);

 private:
  std::string   sysfs_path_;
  std::string   device_name_;
  int           container_fd_ = -1;
  int           group_id_ = -1;
  int           group_fd_ = -1;
  int           device_fd_ = -1;
  vfio_device_info                                device_info_ = { 0 };
  std::array<VfioRegion, MAX_VFIO_REGIONS>        regions_;
  std::array<VfioInterrupt, MAX_VFIO_INTERRUPTS>  interrupts_;
  const MemoryListener*                           memory_listener_ = nullptr;
};

#endif // _MVISOR_DEVICES_VFIO_VFIO_PCI_H
