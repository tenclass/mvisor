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

#ifndef _MVISOR_DEVICES_AHCI_H
#define _MVISOR_DEVICES_AHCI_H

#include "pci_device.h"
#include "ahci_port.h"
#include <array>

struct AHCIHostControlRegs {
  uint32_t    capabilities;
  uint32_t    global_host_control;
  uint32_t    irq_status;
  uint32_t    ports_implemented;
  uint32_t    version;
  uint32_t    command_completion_coalescing_control;
  uint32_t    command_completion_coalescing_ports;
  uint32_t    enclosure_management_location;
  uint32_t    enclosure_management_control;
  uint32_t    extended_capabilities;
  uint32_t    bohc; // BIOS/OS handoff control and status

  // 0x2C - 0x9F, Reserved
  uint8_t     reserved[0xA0-0x2C];
 
  // 0xA0 - 0xFF, Vendor specific registers
  uint8_t     vendor[0x100-0xA0];
} __attribute__((packed));


class AhciHost : public PciDevice {
 public:
  AhciHost();
  virtual ~AhciHost();

  virtual void Connect();
  virtual void Disconnect();
  virtual void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void Reset();
  virtual void SoftReset();
  void CheckIrq();

  bool SaveState(MigrationWriter* writer);
  bool LoadState(MigrationReader* reader);

 private:
  uint num_ports_;
  AHCIHostControlRegs host_control_;
  std::array<AhciPort*, 32> ports_ = { 0 };
};

#endif // _MVISOR_DEVICES_AHCI_H
