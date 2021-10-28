#include "mvisor/vcpu.h"

#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cstring>
#include "mvisor/machine.h"
#include "mvisor/logger.h"

namespace mvisor
{

Vcpu::Vcpu(const Machine* machine) : machine_(machine) {
  fd_ = ioctl(machine_->vm_fd_, KVM_CREATE_VCPU, 0);
  MV_ASSERT(fd_ > 0);

  kvm_run_ = (struct kvm_run*)mmap(nullptr, machine_->kvm_vcpu_mmap_size_,
    PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  MV_ASSERT(kvm_run_ != MAP_FAILED);

  TestRealMode();
}

Vcpu::~Vcpu() {
  if (thread_.joinable()) {
    thread_.join();
  }
  if (fd_ > 0)
    close(fd_);
}

void Vcpu::Start() {
  thread_ = std::thread(&Vcpu::Process, this);
}

void Vcpu::TestRealMode() {
  struct kvm_sregs sregs;
  if (ioctl(fd_, KVM_GET_SREGS, &sregs) < 0) {
    MV_PANIC("KVM_GET_SREGS");
  }
  sregs.cs.selector = 0;
  sregs.cs.base = 0;
  if (ioctl(fd_, KVM_SET_SREGS, &sregs) < 0) {
    MV_PANIC("KVM_SET_SREGS");
  }

  struct kvm_regs regs;
  bzero(&regs, sizeof(regs));
  regs.rflags = 2;
  regs.rip = 0;

  if (ioctl(fd_, KVM_SET_REGS, &regs) < 0) {
    MV_PANIC("KVM_SET_REGS");
  }
}

void Vcpu::Process() {
  for (;;) {
    if (ioctl(fd_, KVM_RUN, 0) < 0) {
      MV_PANIC("KVM_RUN");
    }

    switch (kvm_run_->exit_reason)
    {
    case KVM_EXIT_HLT:
      goto check;
    case KVM_EXIT_IO:
      if (kvm_run_->io.port == 0x01 && kvm_run_->io.direction == KVM_EXIT_IO_OUT) {
        const char* character = reinterpret_cast<char*>(kvm_run_) + kvm_run_->io.data_offset;
        putchar(*character);
        fflush(stdout);
      }
      break;
    default:
      MV_PANIC("exit reason %d, expected KVM_EXIT_HLT(%d)\n",
        kvm_run_->exit_reason, KVM_EXIT_HLT);
    }
  }

check:
  return;
}

} // namespace mvisor
