#ifndef _MVISOR_VCPU_H
#define _MVISOR_VCPU_H

#include <linux/kvm.h>
#include <thread>

class Machine;

class Vcpu {
 public:
  Vcpu(const Machine* machine, int vcpu_id);
  ~Vcpu();
  void Start();

 private:
  void Process();
  void PrintRegisters();
  void EnableSingleStep();

  const Machine* machine_;
  int vcpu_id_ = -1;
  int fd_ = -1;
  char thread_name_[16];
  struct kvm_run *kvm_run_;
  struct kvm_coalesced_mmio_ring *mmio_ring_;
  std::thread thread_;
};

#endif // _MVISOR_VCPU_H
