/* 
 * MVisor
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

#ifndef _MVISOR_DEVICES_IDE_PORT_H
#define _MVISOR_DEVICES_IDE_PORT_H

#include <deque>
#include <vector>

#include <cstdint>
#include "ata_storage.h"
#include "device.h"
#include "ide_host.pb.h"


#define BM_STATUS_RUNNING 0x01
#define BM_STATUS_ERROR   0x02
#define BM_STATUS_INT     0x04

#define BM_CMD_START      0x01
#define BM_CMD_READ       0x08

struct PhysicalRegionDescriptor
{
  uint32_t  physical_address;
  uint16_t  byte_count;
  uint16_t  reserved : 15;
  uint16_t  end_of_table : 1;
} __attribute__((packed));

struct IdeBusMaster {
  uint8_t   command;
  uint8_t   status;
  uint32_t  prdt_address;
};


class DeviceManager;
class IdeHost;
class IdePort : public AtaPort {
 public:
  IdePort(DeviceManager* manager, IdeHost* host, int index);
  virtual ~IdePort();

  void AttachDevice(AtaStorageDevice* device, bool slave);
  void Write(uint64_t offset, uint8_t* data, uint32_t size);
  void Read(uint64_t offset, uint8_t* data, uint32_t size);
  void WriteControl(uint64_t offset, uint8_t* data, uint32_t size);
  void ReadControl(uint64_t offset, uint8_t* data, uint32_t size);
  void Reset();
  void OnBusMasterDmaStart(std::deque<iovec>& buffers);
  void OnBusMasterDmaStop();

  void ReadBusMaster(uint64_t offset, uint8_t* data, uint32_t size);
  void WriteBusMaster(uint64_t offset, uint8_t* data, uint32_t size);

  /* Called by AtaStorage */
  virtual void OnDmaPrepare();
  virtual void OnDmaTransfer();
  virtual void OnPioTransfer();
  virtual void OnCommandDone();

  /* Called by IdeHost */
  void SaveState(IdeHostState_PortState* port_state);
  void LoadState(const IdeHostState_PortState* port_state);

  inline AtaStorageDevice** drives() { return drives_; } 

 private:
  DeviceManager*        manager_;
  IdeHost *             host_;
  int                   port_index_;
  AtaStorageDevice*     drives_[2];
  TaskFile              task_files_[2];
  uint                  active_tf_index_ = 0;
  std::string           buffer_;
  size_t                buffer_position_;
  uint8_t               control_ = 0;
  IdeBusMaster          bus_master_;
  std::vector<iovec>    current_dma_vector_;

  void StartBusMasterDma();
  void StopBusMasterDma();
  void ExecuteCommand();
  void ReadIoBuffer(uint8_t* data, uint32_t size);
  void WriteIoBuffer(uint8_t* data, uint32_t size);
  void SetIrq(uint level);
};

#endif // _MVISOR_DEVICES_IDE_PORT_H

