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

#ifndef _MVISOR_VCPU_H
#define _MVISOR_VCPU_H

#include <linux/kvm.h>
#include <thread>
#include <deque>
#include <functional>
#include <mutex>
#include <vector>
#include <condition_variable>

#include "hyperv/hyperv.h"
#include "migration.h"
#include "vcpu.pb.h"

#define SIG_USER_INTERRUPT (SIGRTMIN + 0)

/* https://github.com/torvalds/linux/blob/master/include/linux/sched/prio.h */
#define MAX_NICE	19
#define MIN_NICE	-20

class Machine;

typedef std::function<void(void)> VoidCallback;
struct VcpuTask {
  VoidCallback   callback;
};

struct HyperVSynic {
  bool                      enabled = false;
  uint64_t                  message_address = 0;
  uint64_t                  event_address = 0;
  hyperv_message_page*      message_page = nullptr;
  hyperv_event_flags_page*  event_flags_page = nullptr;
};

class Vcpu {
 public:
  Vcpu(Machine* machine, int vcpu_id);
  ~Vcpu();

  /* Create a vCPU thread and execute in guest VM */
  void Start();
  /* Wakeup a sleeping guest vCPU */
  void Kick();
  /* Inject a function and also signal the vCPU */
  void Schedule(VoidCallback callback);
  /* Reset vCPU registers to default values */
  void Reset();

  /* Used for migration */
  bool SaveState(MigrationWriter* writer);
  bool LoadState(MigrationReader* reader);

  /* Used for debugging */
  void EnableSingleStep();
  void PrintRegisters();

  int fd() { return fd_; }
  int vcpu_id() { return vcpu_id_; }
  std::thread& thread() { return thread_; }
  static Vcpu* current_vcpu() { return current_vcpu_; }
  const char* name() { return name_; }
  uint64_t cpuid_features() { return cpuid_features_; }
  uint32_t cpuid_version() { return cpuid_version_; }
  const std::string& cpuid_model() { return cpuid_model_; }

 private:
  static void SignalHandler(int signum);
  void SetupSignalHandler();
  void SetupCpuid();
  void SetupMsrIndices();
  void SetupSchedPriority(int priority);
  void SetupHyperV(kvm_cpuid2* cpuid);
  void SetupMachineCheckException();
  void SetupModelSpecificRegisters();
  uint64_t GetSupportedMsrFeature(uint index);
  void SaveDefaultRegisters();
  bool PreRun();
  void PostRun();
  void Process();
  void HandleIo();
  void HandleMmio();
  void HandleHyperV();
  void ExecuteTasks();
  void SaveStateTo(VcpuState& state);
  void LoadStateFrom(VcpuState& state, bool load_cpuid);

  static __thread Vcpu*     current_vcpu_;

  Machine*                  machine_;
  int                       vcpu_id_ = -1;
  int                       fd_ = -1;
  char                      name_[16];
  kvm_run*                  kvm_run_ = nullptr;
  kvm_coalesced_mmio_ring*  mmio_ring_ = nullptr;
  std::thread               thread_;
  bool                      single_step_ = false;
  bool                      paused_ = true;
  int                       wait_count_ = 0;
  std::condition_variable   wait_to_resume_;
  std::condition_variable   wait_for_paused_;
  VcpuState                 default_state_;
  std::deque<VcpuTask>      tasks_;
  std::mutex                mutex_;
  std::set<uint32_t>        msr_indices_;
  uint32_t                  hyperv_features_ = 0;
  uint64_t                  cpuid_features_ = 0;
  uint32_t                  cpuid_version_ = 0;
  std::string               cpuid_model_;
  HyperVSynic               hyperv_synic_;
  
  friend class              VcpuRunLockGuard;
};


// Pause vCPU threads
class VcpuRunLockGuard {
 private:
  std::vector<Vcpu*> vcpus_;

 public:
  VcpuRunLockGuard(Vcpu* vcpu) {
    vcpus_.push_back(vcpu);
    PauseAll();
  }
  VcpuRunLockGuard(const std::vector<Vcpu*>& vcpus) {
    vcpus_ = vcpus;
    PauseAll();
  }
  ~VcpuRunLockGuard() {
    ResumeAll();
  }

  void PauseAll() {
    for (auto vcpu: vcpus_) {
      std::unique_lock<std::mutex> lock(vcpu->mutex_);
      vcpu->wait_count_++;
      vcpu->Kick();
    }
    for (auto vcpu: vcpus_) {
      std::unique_lock<std::mutex> lock(vcpu->mutex_);
      if (vcpu->paused_) {
        // Don't wait in case the vCPU is already paused or exited
        continue;
      }
      vcpu->wait_for_paused_.wait(lock, [vcpu]() {
        return vcpu->paused_;
      });
    }
  }

  void ResumeAll() {
    for (auto vcpu: vcpus_) {
      std::unique_lock<std::mutex> lock(vcpu->mutex_);
      vcpu->wait_count_--;
      if (vcpu->wait_count_ == 0) {
        vcpu->wait_to_resume_.notify_all();
      }
    }
  }
};

#endif // _MVISOR_VCPU_H
