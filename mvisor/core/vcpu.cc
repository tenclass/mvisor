#include "vcpu.h"

extern "C" {
#include "apicdef.h"
}
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cstring>
#include "machine.h"
#include "logger.h"
#include "arch/cpuid.h"

__thread Vcpu* Vcpu::current_vcpu_ = nullptr;

Vcpu::Vcpu(Machine* machine, int vcpu_id)
    : machine_(machine), vcpu_id_(vcpu_id) {
  fd_ = ioctl(machine_->vm_fd_, KVM_CREATE_VCPU, vcpu_id_);
  MV_ASSERT(fd_ > 0);

  kvm_run_ = (struct kvm_run*)mmap(nullptr, machine_->kvm_vcpu_mmap_size_,
    PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  MV_ASSERT(kvm_run_ != MAP_FAILED);

  int coalesced_offset = ioctl(machine_->kvm_fd_, KVM_CHECK_EXTENSION, KVM_CAP_COALESCED_MMIO);
  if (coalesced_offset) {
    mmio_ring_ = (struct kvm_coalesced_mmio_ring*)((uint64_t)kvm_run_ + coalesced_offset * PAGE_SIZE);
  }
  
  struct local_apic lapic;
  if (ioctl(fd_, KVM_GET_LAPIC, &lapic) < 0) {
    MV_PANIC("KVM_GET_LAPIC");
  }
  lapic.lvt_lint0.delivery_mode = APIC_MODE_EXTINT;
  lapic.lvt_lint1.delivery_mode = APIC_MODE_NMI;
  if (ioctl(fd_, KVM_SET_LAPIC, &lapic) < 0) {
    MV_PANIC("KVM_SET_LAPIC");
  }
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

void Vcpu::EnableSingleStep() {
  struct kvm_guest_debug debug = {
    .control = KVM_GUESTDBG_ENABLE | KVM_GUESTDBG_SINGLESTEP,
  };

  if (ioctl(fd_, KVM_SET_GUEST_DEBUG, &debug) < 0)
    MV_PANIC("KVM_SET_GUEST_DEBUG");
  debug_ = true;
}

void Vcpu::ProcessMmio() {
  if (mmio_ring_) {
    const int max_entries = ((PAGE_SIZE - sizeof(struct kvm_coalesced_mmio_ring)) / \
      sizeof(struct kvm_coalesced_mmio));
    while (mmio_ring_->first != mmio_ring_->last) {
      struct kvm_coalesced_mmio *m = &mmio_ring_->coalesced_mmio[mmio_ring_->first];
      device_manager_->HandleMmio(m->phys_addr, m->data, m->len, 1);
      mmio_ring_->first = (mmio_ring_->first + 1) % max_entries;
    }
  }
  auto *mmio = &kvm_run_->mmio;
  device_manager_->HandleMmio(mmio->phys_addr, mmio->data, mmio->len, mmio->is_write);
}

void Vcpu::ProcessIo() {
  auto *io = &kvm_run_->io;
  uint8_t* data = reinterpret_cast<uint8_t*>(kvm_run_) + kvm_run_->io.data_offset;
  device_manager_->HandleIo(io->port, data, io->size, io->direction, io->count);
}

void Vcpu::vcpu_thread_handler(int signum) {
  // Do nothing now ...
}

void Vcpu::SetupSingalHandlers() {
  sigset_t sigset;
  sigemptyset(&sigset);
  pthread_sigmask(SIG_BLOCK, &sigset, nullptr);

  signal(SIG_USER_INTERRUPT, Vcpu::vcpu_thread_handler);
}

void Vcpu::Process() {
  current_vcpu_ = this;
  sprintf(thread_name_, "vcpu-%d", vcpu_id_);
  SetThreadName(thread_name_);
  SetupSingalHandlers();
  MV_LOG("%s started", thread_name_);

  device_manager_ = machine_->device_manager();
  kvm_cpu_setup_cpuid(machine_->kvm_fd_, fd_);

  for (; machine_->valid_;) {
    int ret = ioctl(fd_, KVM_RUN, 0);
    if (ret < 0) {
      MV_LOG("KVM_RUN failed vcpu=%d ret=%d", vcpu_id_, ret);
    }

    switch (kvm_run_->exit_reason)
    {
    case KVM_EXIT_UNKNOWN:
      MV_LOG("KVM_EXIT_UNKNOWN vcpu=%d", vcpu_id_);
      break;
    case KVM_EXIT_SHUTDOWN:
    case KVM_EXIT_HLT:
      goto check;
    case KVM_EXIT_DEBUG:
      PrintRegisters();
      getchar();
      break;
    case KVM_EXIT_IO:
      ProcessIo();
      break;
    case KVM_EXIT_MMIO:
      ProcessMmio();
      break;
    default:
      MV_PANIC("exit reason %d, expected KVM_EXIT_HLT(%d)\n",
        kvm_run_->exit_reason, KVM_EXIT_HLT);
    }
  }

check:
  MV_LOG("%s ended", thread_name_);
}

void Vcpu::PrintRegisters() {
  struct kvm_regs regs;
  struct kvm_sregs sregs;
  if (ioctl(fd_, KVM_GET_REGS, &regs) < 0)
    MV_PANIC("KVM_GET_REGS failed");
  if (ioctl(fd_, KVM_GET_SREGS, &sregs) < 0)
    MV_PANIC("KVM_GET_REGS failed");
  ::PrintRegisters(regs, sregs);
}
