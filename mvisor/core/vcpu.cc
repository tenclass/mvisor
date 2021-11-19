#include "vcpu.h"

#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cstring>
#include "machine.h"
#include "logger.h"

#define MAX_KVM_CPUID_ENTRIES 100

/* Use Vcpu::current_vcpu() */
__thread Vcpu* Vcpu::current_vcpu_ = nullptr;


Vcpu::Vcpu(Machine* machine, int vcpu_id)
    : machine_(machine), vcpu_id_(vcpu_id) {

  fd_ = ioctl(machine_->vm_fd_, KVM_CREATE_VCPU, vcpu_id_);
  MV_ASSERT(fd_ > 0);

  /* A one page size memory region stored some information of current vcpu */
  kvm_run_ = (struct kvm_run*)mmap(nullptr, machine_->kvm_vcpu_mmap_size_,
    PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  MV_ASSERT(kvm_run_ != MAP_FAILED);

  /* Handle multiple MMIO operations at one time */
  int coalesced_offset = ioctl(machine_->kvm_fd_, KVM_CHECK_EXTENSION, KVM_CAP_COALESCED_MMIO);
  if (coalesced_offset) {
    mmio_ring_ = (struct kvm_coalesced_mmio_ring*)((uint64_t)kvm_run_ + coalesced_offset * PAGE_SIZE);
  }
  
  SetupCpuid();
}

Vcpu::~Vcpu() {
  if (thread_.joinable()) {
    thread_.join();
  }
  if (fd_ > 0)
    close(fd_);
}

void Vcpu::Start() {
  /* Starting a vcpu is as simple as starting a thread on the host */
  thread_ = std::thread(&Vcpu::Process, this);
}

/* 
 * Intel CPUID Instruction Reference
 * https://www.intel.com/content/dam/develop/external/us/en/documents/ \
 * architecture-instruction-set-extensions-programming-reference.pdf
 * TODO: Win10 shows unknown processor
 */
void Vcpu::SetupCpuid() {
  struct kvm_cpuid2 *cpuid = (struct kvm_cpuid2*)malloc(
    sizeof(*cpuid) + MAX_KVM_CPUID_ENTRIES * sizeof(cpuid->entries[0]));
  
  cpuid->nent = MAX_KVM_CPUID_ENTRIES;
  if (ioctl(machine_->kvm_fd_, KVM_GET_SUPPORTED_CPUID, cpuid) < 0)
    MV_PANIC("KVM_GET_SUPPORTED_CPUID failed");
  
  for (uint32_t i = 0; i < cpuid->nent; i++) {
    auto entry = &cpuid->entries[i];
    switch (entry->function)
    {
    case 0x1: // ACPI ID & Features
      entry->ecx &= ~(1 << 31); // disable hypervisor mode now
      entry->ebx = (vcpu_id_ << 24) | (machine_->num_vcpus_ << 16) | (entry->ebx & 0xFFFF);
      break;
    case 0x6: // Thermal and Power Management Leaf
      entry->ecx = entry->ecx & ~(1 << 3); // disable peformance energy bias
      break;
    default:
      break;
    }
  }

  if (ioctl(fd_, KVM_SET_CPUID2, cpuid) < 0)
    MV_PANIC("KVM_SET_CPUID2 failed");

  free(cpuid);
}

/* Used for debugging sometimes */
void Vcpu::EnableSingleStep() {
  struct kvm_guest_debug debug = {
    .control = KVM_GUESTDBG_ENABLE | KVM_GUESTDBG_SINGLESTEP,
  };

  if (ioctl(fd_, KVM_SET_GUEST_DEBUG, &debug) < 0)
    MV_PANIC("KVM_SET_GUEST_DEBUG");
  
  debug_ = true;
}

/* Memory trapped IO */
void Vcpu::ProcessMmio() {
  if (mmio_ring_) {
    const int max_entries = ((PAGE_SIZE - sizeof(struct kvm_coalesced_mmio_ring)) / \
      sizeof(struct kvm_coalesced_mmio));
    while (mmio_ring_->first != mmio_ring_->last) {
      struct kvm_coalesced_mmio *m = &mmio_ring_->coalesced_mmio[mmio_ring_->first];
      machine_->device_manager()->HandleMmio(m->phys_addr, m->data, m->len, 1);
      mmio_ring_->first = (mmio_ring_->first + 1) % max_entries;
    }
  }

  auto *mmio = &kvm_run_->mmio;
  machine_->device_manager()->HandleMmio(mmio->phys_addr, mmio->data, mmio->len, mmio->is_write);
}

/* Traditional IN, OUT operations */
void Vcpu::ProcessIo() {
  auto *io = &kvm_run_->io;
  uint8_t* data = reinterpret_cast<uint8_t*>(kvm_run_) + kvm_run_->io.data_offset;
  machine_->device_manager()->HandleIo(io->port, data, io->size, io->direction, io->count);
}

/* To wake up a vcpu thread, the easist way is to send a signal */
void Vcpu::SignalHandler(int signum) {
  // Do nothing now ...
}

/* Vcpu thread only response to SIG_USER at the moment */
void Vcpu::SetupSingalHandler() {
  sigset_t sigset;
  sigemptyset(&sigset);
  pthread_sigmask(SIG_BLOCK, &sigset, nullptr);

  signal(SIG_USER_INTERRUPT, Vcpu::SignalHandler);
}

/* Initialize and executing a vCPU thread */
void Vcpu::Process() {
  current_vcpu_ = this;
  sprintf(name_, "vcpu-%d", vcpu_id_);
  
  SetThreadName(name_);
  SetupSingalHandler();

  MV_LOG("%s started", name_);

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
      MV_LOG("KVM_EXIT_SHUTDOWN vcpu=%d", vcpu_id_);
      goto quit;
    case KVM_EXIT_HLT:
      MV_LOG("KVM_EXIT_HLT vcpu=%d", vcpu_id_);
      goto quit;
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
      MV_PANIC("exit reason %d, expected KVM_EXIT_HLT(%d)",
        kvm_run_->exit_reason, KVM_EXIT_HLT);
    }
  }

quit:
  MV_LOG("%s ended", name_);

  // FIXME: should I call Quit()?
  if (!machine_->valid_) {
    machine_->valid_ = false;
  }
}

void Vcpu::Kick() {
  pthread_kill(thread_.native_handle(), SIG_USER_INTERRUPT);
}

/* Used for debugging */
void Vcpu::PrintRegisters() {
  struct kvm_regs regs;
  struct kvm_sregs sregs;
  if (ioctl(fd_, KVM_GET_REGS, &regs) < 0)
    MV_PANIC("KVM_GET_REGS failed");
  if (ioctl(fd_, KVM_GET_SREGS, &sregs) < 0)
    MV_PANIC("KVM_GET_REGS failed");

  // call logger.h
  ::PrintRegisters(regs, sregs);
}
