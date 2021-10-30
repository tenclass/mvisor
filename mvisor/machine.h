#ifndef MVISOR_MACHINE_H
#define MVISOR_MACHINE_H

#include <vector>
#include "vcpu.h"
#include "memory_manager.h"

class Machine {
 public:
  Machine();
  ~Machine();

  int Run();
 private:
  friend class Vcpu;
  friend class MemoryManager;
  void InitializeKvm();
  void CreateVm();
  void CreateVcpu();
  void LoadBiosFile(const char* path);

  int kvm_fd_ = -1;
  int kvm_vcpu_mmap_size_ = 0;
  int vm_fd_ = -1;
  
  std::vector<Vcpu*> vcpus_;
  MemoryManager* memory_manager_;
  // Allocate 8GB RAM
  uint64_t ram_size_ = 8LL * (1 << 30);
  uint8_t* bios_data_ = nullptr;
  size_t bios_size_ = 0;
};

#endif // MVISOR_MACHINE_H
