#ifndef _MVISOR_VCPU_H
#define _MVISOR_VCPU_H

#include <linux/kvm.h>
#include <thread>

namespace mvisor {

class Machine;

class Vcpu {
 public:
  Vcpu(const Machine* machine);
  ~Vcpu();

 private:
  void Process();
  void TestRealMode();

  const Machine* machine_;
  int fd_ = -1;
  struct kvm_run *kvm_run_;
  std::thread thread_;
};

} // namespace mvisor

#endif // _MVISOR_VCPU_H
