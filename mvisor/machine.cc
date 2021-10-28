#include "mvisor/machine.h"
#include <linux/kvm.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>

#include "mvisor/logger.h"

extern const unsigned char guest16[], guest16_end[];

namespace mvisor {

Machine::Machine() {
  InitializeKvm();
  CreateVm();
  CreateVcpu();
}

Machine::~Machine() {
  if (vm_fd_ > 0)
    close(vm_fd_);
  if (kvm_fd_ > 0)
    close(kvm_fd_);
}

void Machine::InitializeKvm() {
  kvm_fd_ = open("/dev/kvm", O_RDWR);
  MV_ASSERT(kvm_fd_ > 0);

  int api_version = ioctl(kvm_fd_, KVM_GET_API_VERSION, 0);
  if (api_version != KVM_API_VERSION) {
    MV_PANIC("kvm api verison %d, expected: %d", api_version, KVM_API_VERSION);
  }

  kvm_vcpu_mmap_size_ = ioctl(kvm_fd_, KVM_GET_VCPU_MMAP_SIZE, 0);
  MV_ASSERT(kvm_vcpu_mmap_size_ > 0);
}

void Machine::CreateVm() {
  vm_fd_ = ioctl(kvm_fd_, KVM_CREATE_VM, 0);
  MV_ASSERT(vm_fd_ > 0);

  size_t vm_memory_size = 0x200000;
  vm_memory_ = mmap(nullptr, vm_memory_size, PROT_READ | PROT_WRITE,
    MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
  MV_ASSERT(vm_memory_ != MAP_FAILED);
  memcpy(vm_memory_, guest16, guest16_end - guest16);

  struct kvm_userspace_memory_region memreg;
  memreg.slot = 0;
  memreg.flags = 0;
  memreg.guest_phys_addr = 0;
  memreg.memory_size = vm_memory_size;
  memreg.userspace_addr = (uint64_t)vm_memory_; 
  if (ioctl(vm_fd_, KVM_SET_USER_MEMORY_REGION, &memreg) < 0) {
    MV_PANIC("failed to set user memory region");
  }
}

void Machine::CreateVcpu() {
  Vcpu* vcpu = new Vcpu(this);
  vcpus_.push_back(vcpu);
}

int Machine::Run() {
  MV_LOG("ok");

  for (auto vcpu: vcpus_) {
    delete vcpu;
  }
  return 0;
}

} // namespace mvisor
