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

#include "pmio.h"

#include <sys/time.h>

#include "device_manager.h"
#include "machine.h"
#include "pmio.pb.h"


#define ACPI_BITMASK_SLEEP_ENABLE   0x2000
#define PM_TIMER_FREQUENCY          3579545


void Pmio::UpdateSystemControlIrq() {
  uint level = (pm1_.enable & pm1_.status & 0x521) || (gpe0_.enable & gpe0_.status);
  // FIXME: hardcoded IRQ 9
  manager_->SetGsiLevel(9, level);
}

void Pmio::Reset() {
  PciDevice::Reset();

  bzero(&pm1_, sizeof(pm1_));
  bzero(&gpe0_, sizeof(gpe0_));
  bzero(&smi_, sizeof(smi_));

  smi_.enable = PMIO_SMI_EN_APMC_EN;
  pmio_base_ = 0;
}

bool Pmio::SaveState(MigrationWriter* writer) {
  PmioState state;
  state.set_pm1_status(pm1_.status);
  state.set_pm1_enable(pm1_.enable);
  state.set_pm1_control(pm1_.control);
  state.set_pm1_timer(pm1_.timer);
  state.set_gpe0_enable(gpe0_.enable);
  state.set_gpe0_status(gpe0_.status);
  state.set_smi_enable(smi_.enable);
  state.set_smi_status(smi_.status);

  writer->WriteProtobuf("PMIO", state);
  return PciDevice::SaveState(writer);
}

bool Pmio::LoadState(MigrationReader* reader) {
  if (!PciDevice::LoadState(reader)) {
    return false;
  }

  PmioState state;
  /* For old snapshots, PMIO is within LPC, so just skip it */
  if (!reader->Exists("PMIO")) {
    return true;
  }
  if (!reader->ReadProtobuf("PMIO", state)) {
    return false;
  }
  smi_.status = state.smi_status();
  smi_.enable = state.smi_enable();
  gpe0_.status = state.gpe0_status();
  gpe0_.enable = state.gpe0_enable();
  pm1_.timer = state.pm1_timer();
  pm1_.control = state.pm1_control();
  pm1_.enable = state.pm1_enable();
  pm1_.status = state.pm1_status();
  return true;
}

void Pmio::AcpiSuspend(uint8_t type) {
  switch (type)
  {
  case 0: // soft power off
    std::thread([this]() {
      manager_->machine()->Pause();
      MV_LOG("machine is power off");
    }).detach();
    break;
  case 1: // suspend request
    MV_PANIC("suspend is not supported");
    break;
  default:
    MV_ERROR("unknown acpi suspend type=%d", type);
    break;
  }
}


uint64_t Pmio::GetClock() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  uint64_t now = tv.tv_sec * 1000000000LL + (tv.tv_usec * 1000);

  auto clock = (__int128_t)now * PM_TIMER_FREQUENCY / 1000000000;
  return clock & 0xFFFFFF;
}

void Pmio::Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  if (resource->base == pmio_base_) {
    uint64_t value;
    switch (offset)
    {
    case 0x00:
      value = pm1_.status;
      break;
    case 0x02:
      value = pm1_.enable;
      break;
    case 0x04:
      value = pm1_.control;
      break;
    case 0x08:
      value = GetClock();
      break;
    case 0x20 ... 0x27:
      value = gpe0_.status >> ((offset - 0x20) << 3);
      break;
    case 0x28 ... 0x2F:
      value = gpe0_.enable >> ((offset - 0x28) << 3);
      break;
    case 0x30:
      value = smi_.enable;
      break;
    case 0x34:
      value = smi_.status;
      break;
    default:
      MV_WARN("not supported reading at ACPI offset=0x%x", offset);
      break;
    }
    memcpy(data, &value, size);
  } else {
    PciDevice::Read(resource, offset, data, size);
  }
}

void Pmio::Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  if (resource->base == pmio_base_) {
    uint64_t value = 0;
    memcpy(&value, data, size);

    switch (offset)
    {
    case 0x00:
      pm1_.status &= ~value;
      UpdateSystemControlIrq();
      break;
    case 0x02:
      pm1_.enable = value;
      UpdateSystemControlIrq();
      break;
    case 0x04:
      pm1_.control &= ~ACPI_BITMASK_SLEEP_ENABLE;
      if (value & ACPI_BITMASK_SLEEP_ENABLE) {
        AcpiSuspend((value >> 10) & 7);
      }
      break;
    case 0x20 ... 0x27:
      MV_ASSERT(size == 1);
      ((uint8_t*)&gpe0_.status)[offset - 0x20] &= ~data[0];
      break;
    case 0x28 ... 0x2F:
      memcpy((uint8_t*)&gpe0_.enable + (offset - 0x28), data, size);
      break;
    default:
      MV_WARN("not supported writing at ACPI offset=0x%lx value=0x%lx", offset, value);
      break;
    }
  } else {
    PciDevice::Write(resource, offset, data, size);
  }
}

/* Power down interface */
void Pmio::PowerDown() {
  Schedule([this]() {
    if (pm1_.enable & 0x100) {
      pm1_.status |= 0x100;
      UpdateSystemControlIrq();
    }
  });
}
