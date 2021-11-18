#include "machine.h"
#include <linux/kvm.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <bits/sigthread.h>

#include "logger.h"

#define X86_EPT_IDENTITY_BASE 0xfeffc000
#define BIOS_PATH "../assets/bios-256k.bin"

Machine::Machine(int vcpus, uint64_t ram_size)
    : num_vcpus_(vcpus), ram_size_(ram_size) {

  InitializeKvm();
  memory_manager_ = new MemoryManager(this);

  CreateArchRelated();
  CreateVcpu();

  device_manager_ = new DeviceManager(this);
  device_manager_->IntializeQ35();
  
  LoadBiosFile(BIOS_PATH);
}

/* Free VM resources */
Machine::~Machine() {
  valid_ = false;

  // Join all vcpu threads and free resources
  for (auto vcpu: vcpus_) {
    delete vcpu;
  }

  delete device_manager_;
  delete memory_manager_;

  if (vm_fd_ > 0)
    close(vm_fd_);
  if (kvm_fd_ > 0)
    close(kvm_fd_);
  if (bios_data_)
    free(bios_data_);
}

void Machine::InitializeKvm() {
  kvm_fd_ = open("/dev/kvm", O_RDWR);
  MV_ASSERT(kvm_fd_ > 0);

  int api_version = ioctl(kvm_fd_, KVM_GET_API_VERSION, 0);
  if (api_version != KVM_API_VERSION) {
    MV_PANIC("kvm api verison %d, expected: %d", api_version, KVM_API_VERSION);
  }

  // Get the vcpu information block size that share with kernel
  // Vcpu uses this value
  kvm_vcpu_mmap_size_ = ioctl(kvm_fd_, KVM_GET_VCPU_MMAP_SIZE, 0);
  MV_ASSERT(kvm_vcpu_mmap_size_ > 0);

  // Create vm so that we can map userspace memory
  vm_fd_ = ioctl(kvm_fd_, KVM_CREATE_VM, 0);
  MV_ASSERT(vm_fd_ > 0);
}

/* SeaBIOS is loaded into the end of 1MB and the end of 4GB */
void Machine::LoadBiosFile(const char* path) {
  // Read BIOS data from path to bios_data
  int fd = open(path, O_RDONLY);
  MV_ASSERT(fd > 0);
  struct stat st;
  fstat(fd, &st);

  bios_size_ = st.st_size;
  bios_data_ = static_cast<uint8_t*>(valloc(bios_size_));
  MV_ASSERT(bios_data_);
  read(fd, bios_data_, bios_size_);
  close(fd);

  // Map BIOS file to memory
  memory_manager_->Map(0x100000 - bios_size_, bios_size_, bios_data_, kMemoryTypeRam, "seabios");
  memory_manager_->Map(0x100000000 - bios_size_, bios_size_, bios_data_, kMemoryTypeRam, "seabios");
}


/*
  * On older Intel CPUs, KVM uses vm86 mode to emulate 16-bit code directly.
  * In order to use vm86 mode, an EPT identity map and a TSS  are needed.
  * Since these must be part of guest physical memory, we need to allocate
  * them, both by setting their start addresses in the kernel and by
  * creating a corresponding e820 entry. We need 4 pages before the BIOS.
  *
  * Older KVM versions may not support setting the identity map base. In
  * that case we need to stick with the default, i.e. a 256K maximum BIOS
  * size.
  */
void Machine::CreateArchRelated() {
  /* Allows up to 16M BIOSes. */
  uint64_t identity_base = X86_EPT_IDENTITY_BASE;
  if (ioctl(vm_fd_, KVM_SET_IDENTITY_MAP_ADDR, &identity_base) < 0) {
    MV_PANIC("failed to set identity map address");
  }

  if (ioctl(vm_fd_, KVM_SET_TSS_ADDR, identity_base + 0x1000) < 0) {
    MV_PANIC("failed to set tss");
  }
  
  /* Map these addresses as reserved so the guest never touch it */
  memory_manager_->Map(X86_EPT_IDENTITY_BASE, 4 * PAGE_SIZE, nullptr, kMemoryTypeReserved, "ept+tss");

  // Use Kvm in-kernel IRQChip
  if (ioctl(vm_fd_, KVM_CREATE_IRQCHIP) < 0) {
    MV_PANIC("failed to create irqchip");
  }

  // Use Kvm in-kernel PITClock
  struct kvm_pit_config pit_config = { 0 };
  if (ioctl(vm_fd_, KVM_CREATE_PIT2, &pit_config) < 0) {
    MV_PANIC("failed to create pit");
  }
}


void Machine::CreateVcpu() {
  for (int i = 0; i < num_vcpus_; ++i) {
    Vcpu* vcpu = new Vcpu(this, i);
    vcpus_.push_back(vcpu);
  }
}

/* Send an IRQ to the guest */
void Machine::Interrupt(uint32_t irq, uint32_t level) {
  struct kvm_irq_level irq_level = {
    .irq = irq,
    .level = level
  };
  if (ioctl(vm_fd_, KVM_IRQ_LINE, &irq_level) != 0) {
    MV_PANIC("KVM_IRQ_LINE failed");
  }
}

/* We should do the async IO here */
int Machine::Run() {
  MV_LOG("ok");
  for (auto vcpu: vcpus_) {
    vcpu->Start();
  }

  return 0;
}

/* Maybe there are lots of things to do before quiting a VM */
void Machine::Quit() {
  if (!valid_)
    return;
  valid_ = false;

  for (auto vcpu: vcpus_) {
    pthread_kill(vcpu->thread().native_handle(), SIG_USER_INTERRUPT);
  }
}


