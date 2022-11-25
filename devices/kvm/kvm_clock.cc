/* 
 * MVisor
 * Copyright (C) 2021 cair <rui.cai@tenclass.com>
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

class KvmClock : public Device {
 private:
  kvm_clock_data              kvm_clock_;
  const StateChangeListener*  state_change_listener_ = nullptr;

  void SynchronizeClock() {
    auto machine = manager_->machine();
    MV_ASSERT(ioctl(machine->kvm_fd(), KVM_CHECK_EXTENSION, KVM_CAP_KVMCLOCK_CTRL) == 1);
    for (auto vcpu: machine->vcpus()) {
      vcpu->Schedule([vcpu]() {
        auto ret = ioctl(vcpu->fd(), KVM_KVMCLOCK_CTRL, 0);
        MV_ASSERT(ret == 0 || (ret == -1 && errno == EINVAL));
      });
    }
  }

  void SaveClock() {
    bzero(&kvm_clock_, sizeof(kvm_clock_));
    MV_ASSERT(ioctl(manager_->machine()->vm_fd(), KVM_GET_CLOCK, &kvm_clock_) == 0);
  }

  void LoadClock() {
    /* Reset kvm clock, only when kvm_clock_ was set before */
    if (!kvm_clock_.clock) {
      return;
    }

    kvm_clock_.flags = 0;
    bzero(kvm_clock_.pad, sizeof(kvm_clock_.pad));
    MV_ASSERT(ioctl(manager_->machine()->vm_fd(), KVM_SET_CLOCK, &kvm_clock_) == 0);

    /* Synchronize kvm-clock among all vcpus */
    SynchronizeClock();
  }

 public:
  KvmClock() {
    bzero(&kvm_clock_, sizeof(kvm_clock_));
  }

  void Connect() {
    Device::Connect();
    auto machine = manager_->machine();
    state_change_listener_ = machine->RegisterStateChangeListener([=]() {
      if (machine->IsPaused()) {
        SaveClock();
      } else {
        LoadClock();
      }
    });
  }

  void Disconnect () {
    if (state_change_listener_) {
      manager_->machine()->UnregisterStateChangeListener(&state_change_listener_);
    }
    Device::Disconnect();
  }

  bool SaveState(MigrationWriter* writer) {
    writer->WriteRaw("CLOCK", &kvm_clock_, sizeof(kvm_clock_));
    return Device::SaveState(writer);
  }

  bool LoadState(MigrationReader* reader) {
    if (!reader->ReadRaw("CLOCK", &kvm_clock_, sizeof(kvm_clock_))) {
      return false;
    }

    // KVM_SET_CLOCK would be called when vm was resumed
    return Device::LoadState(reader);
  }
};

DECLARE_DEVICE(KvmClock);
