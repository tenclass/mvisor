#ifndef _MVISOR_VCPU_H
#define _MVISOR_VCPU_H

#include <linux/kvm.h>
#include <thread>
#include <deque>
#include <functional>
#include <mutex>

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
  void SaveDefaultRegisters();
  void Process();
  void ProcessIo();
  void ProcessMmio();
  void ExecuteTasks();

  static __thread Vcpu* current_vcpu_;

  Machine* machine_;
  int vcpu_id_ = -1;
  int fd_ = -1;
  char name_[16];
  struct kvm_run *kvm_run_;
  struct kvm_coalesced_mmio_ring *mmio_ring_;
  std::thread thread_;
  bool debug_ = false;
  VcpuRegisters default_registers_;
  std::deque<VcpuTask> tasks_;
  std::mutex mutex_;
};

#endif // _MVISOR_VCPU_H
