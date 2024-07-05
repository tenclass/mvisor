/* 
 * MVisor
 * Copyright (C) 2024 Terrence <terrence@tenclas.com>
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

#include <linux/kvm.h>
#include <sys/ioctl.h>

#include <cstring>

#include "device_manager.h"
#include "logger.h"
#include "machine.h"

struct IrqChipState {
  kvm_irqchip master;
  kvm_irqchip slave;
  kvm_irqchip ioapic;
};

class KvmIrqchip : public Device {
 private:
  IrqChipState    default_irqchip_state_;
  kvm_pit_state2  default_pit_state_;
  Machine*        machine_;

  void LoadPitStateFrom(const kvm_pit_state2& pit_state) {
    MV_ASSERT(ioctl(machine_->vm_fd(), KVM_SET_PIT2, &pit_state) == 0);
  }

  void SavePitStateTo(kvm_pit_state2& pit_state) {
    MV_ASSERT(ioctl(machine_->vm_fd(), KVM_GET_PIT2, &pit_state) == 0);
  }

  void LoadIrqChipStateFrom(const IrqChipState& state) {
    /* Load irq chip */
    kvm_irqchip chip = state.master;
    chip.chip_id = KVM_IRQCHIP_PIC_MASTER;
    MV_ASSERT(ioctl(machine_->vm_fd(), KVM_SET_IRQCHIP, &chip) == 0);

    chip = state.slave;
    chip.chip_id = KVM_IRQCHIP_PIC_SLAVE;
    MV_ASSERT(ioctl(machine_->vm_fd(), KVM_SET_IRQCHIP, &chip) == 0);

    chip = state.ioapic;
    chip.chip_id = KVM_IRQCHIP_IOAPIC;
    MV_ASSERT(ioctl(machine_->vm_fd(), KVM_SET_IRQCHIP, &chip) == 0);
  }

  void SaveIrqChipStateTo(IrqChipState& state) {
    state.master.chip_id = KVM_IRQCHIP_PIC_MASTER;
    MV_ASSERT(ioctl(machine_->vm_fd(), KVM_GET_IRQCHIP, &state.master) == 0);

    state.slave.chip_id = KVM_IRQCHIP_PIC_SLAVE;
    MV_ASSERT(ioctl(machine_->vm_fd(), KVM_GET_IRQCHIP, &state.slave) == 0);

    state.ioapic.chip_id = KVM_IRQCHIP_IOAPIC;
    MV_ASSERT(ioctl(machine_->vm_fd(), KVM_GET_IRQCHIP, &state.ioapic) == 0);
  }

 public:
  void Connect() override {
    Device::Connect();
    machine_ = manager_->machine();
  
    // Save the default state for reset
    SaveIrqChipStateTo(default_irqchip_state_);
    SavePitStateTo(default_pit_state_);
  }

  void Reset() override {
    LoadIrqChipStateFrom(default_irqchip_state_);
    LoadPitStateFrom(default_pit_state_);
  }

  bool SaveState(MigrationWriter* writer) override {
    IrqChipState state;
    SaveIrqChipStateTo(state);
    writer->WriteRaw("PIC_MASTER", &state.master.chip.pic, sizeof(state.master.chip.pic));
    writer->WriteRaw("PIC_SLAVE", &state.slave.chip.pic, sizeof(state.slave.chip.pic));
    writer->WriteRaw("IOAPIC", &state.ioapic.chip.ioapic, sizeof(state.ioapic.chip.ioapic));

    kvm_pit_state2 pit2;
    SavePitStateTo(pit2);
    writer->WriteRaw("PIT2", &pit2, sizeof(pit2));
    
    return Device::SaveState(writer);
  }

  bool LoadState(MigrationReader* reader) override {
    IrqChipState state;
    if (!reader->ReadRaw("PIC_MASTER", &state.master.chip.pic, sizeof(state.master.chip.pic)))
      return false;
    if (!reader->ReadRaw("PIC_SLAVE", &state.slave.chip.pic, sizeof(state.slave.chip.pic)))
      return false;
    if (!reader->ReadRaw("IOAPIC", &state.ioapic.chip.ioapic, sizeof(state.ioapic.chip.ioapic)))
      return false;
    LoadIrqChipStateFrom(state);

    kvm_pit_state2 pit2;
    if (!reader->ReadRaw("PIT2", &pit2, sizeof(pit2)))
      return false;
    LoadPitStateFrom(pit2);
    return Device::LoadState(reader);
  }
};

DECLARE_DEVICE(KvmIrqchip);
