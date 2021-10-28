#ifndef MVISOR_MACHINE_H
#define MVISOR_MACHINE_H

#include <vector>
#include "vcpu.h"

namespace mvisor {

class Machine {
public:
  Machine();
  ~Machine();

  int Run();
private:
  friend class Vcpu;
  void InitializeKvm();
  void CreateVm();
  void CreateVcpu();

  int kvm_fd_ = -1;
  int kvm_vcpu_mmap_size_ = 0;
  int vm_fd_ = -1;
  std::vector<Vcpu*> vcpus_;
  void* vm_memory_;
};

} // namespace mvisor

#endif // MVISOR_MACHINE_H
