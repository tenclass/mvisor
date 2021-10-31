#include "machine.h"
#include <linux/kvm.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>

#include "logger.h"

Machine::Machine(int vcpus, uint64_t ram_size)
    : num_vcpus_(vcpus), ram_size_(ram_size) {
  InitializeKvm();
  CreateVm();
  CreateVcpu();

  memory_manager_ = new MemoryManager(this);
  device_manager_ = new DeviceManager(this);
  LoadBiosFile("./assets/bios-debug.bin");
}

Machine::~Machine() {
  delete memory_manager_;
  delete device_manager_;

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

  kvm_vcpu_mmap_size_ = ioctl(kvm_fd_, KVM_GET_VCPU_MMAP_SIZE, 0);
  MV_ASSERT(kvm_vcpu_mmap_size_ > 0);
}

void Machine::LoadBiosFile(const char* path) {
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
  memory_manager_->Map(0x100000 - bios_size_, bios_size_, bios_data_, kMemoryTypeRam);
  memory_manager_->Map(0x100000000 - bios_size_, bios_size_, bios_data_, kMemoryTypeRam);
}

void Machine::CreateVm() {
  vm_fd_ = ioctl(kvm_fd_, KVM_CREATE_VM, 0);
  MV_ASSERT(vm_fd_ > 0);

  // Fix Intel x86 bugs
  if (ioctl(vm_fd_, KVM_SET_TSS_ADDR, 0xFFFBD000) < 0) {
    MV_PANIC("failed to set tss");
  }
  // Use Kvm kernel irqchip
  if (ioctl(vm_fd_, KVM_CREATE_IRQCHIP) < 0) {
    MV_PANIC("failed to create irqchip");
  }
  // PIT Clock
  if (ioctl(vm_fd_, KVM_CREATE_PIT) < 0) {
    MV_PANIC("failed to create pit");
  }
}

void Machine::CreateVcpu() {
  for (int i = 0; i < num_vcpus_; ++i) {
    Vcpu* vcpu = new Vcpu(this, i);
    vcpus_.push_back(vcpu);
  }
}

void Machine::Interrupt(uint32_t irq, uint32_t level) {
	struct kvm_irq_level irq_level;
	irq_level	= (struct kvm_irq_level) {
		{
			.irq		= irq,
		},
		.level		= level,
	};
	if (ioctl(vm_fd_, KVM_IRQ_LINE, &irq_level) < 0)
		MV_PANIC("KVM_IRQ_LINE failed");
}

int Machine::Run() {
  // Apply all memory slots to kvm
  memory_manager_->Commit();
  MV_LOG("ok");
  for (auto vcpu: vcpus_) {
    vcpu->Start();
  }

  for (auto vcpu: vcpus_) {
    delete vcpu;
  }
  return 0;
}

Vcpu* Machine::current_vcpu() {
  std::thread::id current_id = std::this_thread::get_id();
  for (auto vcpu: vcpus_) {
    if (vcpu->thread().get_id() == current_id) {
      return vcpu;
    }
  }
  MV_PANIC("failed to get current vcpu");
  return nullptr;
}
