#ifndef MVISOR_MACHINE_H
#define MVISOR_MACHINE_H

#define PAGE_SIZE 4096

#include <thread>
#include <vector>
#include "vcpu.h"
#include "memory_manager.h"
#include "device_manager.h"

#define X86_EPT_IDENTITY_BASE 0xfeffc000

class Machine {
 public:
  Machine(int vcpus, uint64_t ram_size);
  ~Machine();

  int Run();
  void Interrupt(uint32_t irq, uint32_t level);

  DeviceManager* device_manager() { return device_manager_; }
  MemoryManager* memory_manager() { return memory_manager_; }
  Vcpu* current_vcpu();
  int num_vcpus() { return num_vcpus_; }
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
  
  int num_vcpus_ = 0;
  std::vector<Vcpu*> vcpus_;
  MemoryManager* memory_manager_;
  DeviceManager* device_manager_;
  // Allocate 8GB RAM
  uint64_t ram_size_ = 8LL * (1 << 30);
  uint8_t* bios_data_ = nullptr;
  size_t bios_size_ = 0;
};

#endif // MVISOR_MACHINE_H
