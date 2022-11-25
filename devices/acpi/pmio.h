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

#ifndef _MVISOR_DEVICES_PMIO_H
#define _MVISOR_DEVICES_PMIO_H

#include "device_interface.h"
#include "pci_device.h"

#define PMIO_SMI_EN_APMC_EN       (1 << 5)
#define PMIO_PM1_CTRL_SMI_EN      (1)


class Pmio : public PciDevice, public PowerDownInterface {
 protected:
  struct {
    uint16_t status;
    uint16_t enable;
    uint32_t control;
    uint32_t timer;
  } pm1_;

  struct {
    uint32_t status;
    uint32_t enable;
  } gpe0_;

  struct {
    uint32_t status;
    uint32_t enable;
  } smi_;

  uint32_t   pmio_base_ = 0;

  uint64_t GetClock();
  void UpdateSystemControlIrq();
  void AcpiSuspend(uint8_t type);
  void PowerDown();

  virtual void Reset();
  virtual bool SaveState(MigrationWriter* writer);
  virtual bool LoadState(MigrationReader* reader);

  virtual void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);
};

#endif // _MVISOR_DEVICES_PMIO_H
