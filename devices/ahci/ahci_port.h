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

#ifndef __MVISOR_DEVICES_AHCI_PORT_H
#define __MVISOR_DEVICES_AHCI_PORT_H

#include <mutex>
#include <deque>

#include <cstdint>
#include "ata_storage.h"
#include "device.h"
#include "ahci_host.pb.h"

struct AhciCommandHeader {
  uint8_t     command_fis_length : 5; // in DWORDS, 2 ~ 16
  uint8_t     is_atapi : 1;
  uint8_t     is_write : 1;
  uint8_t     prefetchable : 1;

  uint8_t     reset : 1;
  uint8_t     built_in_self_test : 1;
  uint8_t     clear_busy_upon_r_ok : 1;
  uint8_t     reserved0 : 1;
  uint8_t     port_multiplier : 4;

  uint16_t    prdt_length; // physical region descriptor table length in entries
  uint32_t    bytes_transferred;
  uint64_t    command_table_base;
  uint32_t    reserved[4];
} __attribute__((packed));

struct AhciPortRegs {
  uint32_t    command_list_base0;
  uint32_t    command_list_base1;
  uint32_t    fis_base0;
  uint32_t    fis_base1;
  uint32_t    irq_status;
  uint32_t    irq_mask;
  uint32_t    command;
  uint32_t    reserved0;
  uint32_t    task_flie_data;
  uint32_t    signature;
  uint32_t    sata_status;
  uint32_t    sata_control;
  uint32_t    sata_error;
  uint32_t    sata_active;
  uint32_t    command_issue;
  uint32_t    sata_notification;
  uint32_t    fis_based_switch_control;
  uint32_t    reserved1[11];
  uint32_t    vendor[4];
};

class DeviceManager;
class AhciHost;
struct AhciRxFis;
struct AhciPrdtEntry;

class AhciPort: public AtaPort {
 public:
  AhciPort(DeviceManager* manager, AhciHost* host, int index);
  virtual ~AhciPort();

  void AttachDevice(AtaStorageDevice* device);
  void Write(uint64_t offset, uint32_t value);
  void Read(uint64_t offset, uint8_t* data, uint32_t size);
  void Reset();

  /* Called by AhciHost */
  void SaveState(AhciHostState_PortState* port_state);
  void LoadState(const AhciHostState_PortState* port_state);

  /* Called by AtaStorage */
  virtual void OnDmaPrepare();
  virtual void OnDmaTransfer();
  virtual void OnPioTransfer();
  virtual void OnCommandDone();

  const AhciPortRegs& port_control() const { return port_control_; }

 private:
  void SoftReset();
  void TrigerIrq(int irqbit);
  void UpdateInitD2H();
  void UpdateFisRegisterD2H();
  void UpdateFisSetupPio();
  void UpdateFisSetDeviceBits(int slot);
  void SetNcqError(int slot);
  void HandleNcqCommand(int slot, void* fis);
  bool HandleCommand(int slot);
  void CheckEngines();
  void CheckCommand();
  void ParseDmaVector(AhciPrdtEntry* entries, uint16_t length, bool is_write);

  DeviceManager*        manager_;
  AhciHost*             host_;
  int                   port_index_;
  AhciPortRegs          port_control_;
  AtaStorageDevice*     drive_ = nullptr;
  TaskFile              task_file_;
  bool                  init_d2h_sent_ = false;
  uint8_t*              command_list_ = nullptr;
  AhciRxFis*            rx_fis_ = nullptr;
  int                   busy_slot_ = -1;
  AhciCommandHeader*    current_command_ = nullptr;
  std::vector<iovec>    current_dma_vector_;
};

#endif // __MVISOR_DEVICES_AHCI_PORT_H
