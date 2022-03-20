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

#include "migration.h"

#define SIG_USER_INTERRUPT (SIGRTMIN + 0)

class Machine;

struct VcpuRegisters {
  struct kvm_regs regs;
  struct kvm_sregs sregs;
};

typedef std::function<void(void)> VoidCallback;
struct VcpuTask {
  VoidCallback   callback;
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

  int vcpu_id() { return vcpu_id_; }
  std::thread& thread() { return thread_; }
  static Vcpu* current_vcpu() { return current_vcpu_; }
  const char* name() { return name_; }

 private:
  static void SignalHandler(int signum);
  void SetupSingalHandler();
  void SetupCpuid();
  void SetupMachineCheckException();
  void SetupModelSpecificRegisters();
  uint64_t GetSupportedMsrFeature(uint index);
  void SaveDefaultRegisters();
  void Process();
  void ProcessIo();
  void ProcessMmio();
  void ExecuteTasks();

  static __thread Vcpu*     current_vcpu_;

  Machine*                  machine_;
  int                       vcpu_id_ = -1;
  int                       fd_ = -1;
  char                      name_[16];
  kvm_run*                  kvm_run_;
  kvm_coalesced_mmio_ring*  mmio_ring_;
  std::thread               thread_;
  bool                      single_step_ = false;
  VcpuRegisters             default_registers_;
  std::deque<VcpuTask>      tasks_;
  std::mutex                mutex_;
};

#endif // _MVISOR_VCPU_H
