#ifndef _MVISOR_VCPU_H
#define _MVISOR_VCPU_H

#include <linux/kvm.h>
#include <thread>

#define SIG_USER_INTERRUPT (SIGRTMIN + 0)

class Machine;

class Vcpu {
 public:
  Vcpu(Machine* machine, int vcpu_id);
  ~Vcpu();
  void Start();
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
  void Process();
  void ProcessIo();
  void ProcessMmio();

  static __thread Vcpu* current_vcpu_;

  Machine* machine_;
  int vcpu_id_ = -1;
  int fd_ = -1;
  char name_[16];
  struct kvm_run *kvm_run_;
  struct kvm_coalesced_mmio_ring *mmio_ring_;
  std::thread thread_;
  bool debug_ = false;
};

#endif // _MVISOR_VCPU_H
