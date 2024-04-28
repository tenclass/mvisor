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
#include "vfio_manager.h"
#include "machine.h"

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
  void*     mmap;
};

struct VfioInterrupt {
  int       event_fd;
  int       gsi;
};

struct VfioMigration {
  bool        enabled;
  VfioRegion* region;
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
  virtual void WritePciCommand(uint16_t command);

  /* VFIO migration */
  virtual bool SaveState(MigrationWriter* writer);
  virtual bool LoadState(MigrationReader* reader);

 protected:
  void SetupVfioDevice();
  void SetupPciConfiguration();
  void SetupGfxPlane();
  void SetupMigraionInfo();
  void DisableInterrupts();
  void SetIntxInterruptEnabled(bool enabled);

  void UpdateMsiRoutes();
  void MapBarRegion(uint8_t index);
  void UnmapBarRegion(uint8_t index);
  int  FindRegion(uint32_t type, uint32_t subtype);
  void SetMigrationDeviceState(uint32_t device_state);
  void SetInterruptEventFds(uint32_t index, uint32_t vector, int action, int* fds, uint8_t count);
  void SetInterruptMasked(uint32_t index, bool masked);

 private:
  VfioManager*  vfio_manager_ = nullptr;
  std::string   sysfs_path_;
  std::string   device_name_;
  int           device_fd_ = -1;
  int           intx_trigger_fd_ = -1;
  int           intx_unmask_fd_ = -1;
  bool          multi_function_ = false;
  vfio_device_info                                      device_info_;
  std::array<VfioRegion, MAX_VFIO_REGIONS>              regions_;
  std::array<VfioInterrupt, MAX_VFIO_INTERRUPTS>        interrupts_;
  VfioMigration                                         migration_;
  const StateChangeListener*                            state_change_listener_ = nullptr;
};

#endif // _MVISOR_DEVICES_VFIO_VFIO_PCI_H
