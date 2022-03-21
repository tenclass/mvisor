/* 
 * MVisor
 * Copyright (C) 2021 Terrence <terrence@tenclass.com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "vcpu.h"

#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cstring>

#include "machine.h"
#include "logger.h"
#include "states/vcpu.pb.h"

#define MAX_KVM_CPUID_ENTRIES         100
#define CPUID_TOPOLOGY_LEVEL_SMT      (1U << 8)
#define CPUID_TOPOLOGY_LEVEL_CORE     (2U << 8)
#define CPUID_TOPOLOGY_LEVEL_INVALID  (0U << 8)

#define MSR_IA32_TSC                  0x10
#define MSR_IA32_UCODE_REV            0x8B
#define KVM_MSR_ENTRY(_index, _data) 	(struct kvm_msr_entry) { .index = _index, .data = _data }

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
    auto ring = (kvm_coalesced_mmio_ring*)((uint64_t)kvm_run_ + coalesced_offset * PAGE_SIZE);
    machine_->device_manager()->SetupCoalescingMmioRing(ring);
  }

  /* Save default registers for system reset */
  ioctl(fd_, KVM_GET_REGS, &default_registers_.regs);
  ioctl(fd_, KVM_GET_SREGS, &default_registers_.sregs);
  
  SetupCpuid();

  /* Setup MCE for booting Linux */
  SetupMachineCheckException();
  SetupModelSpecificRegisters();
}

Vcpu::~Vcpu() {
  if (thread_.joinable()) {
    thread_.join();
  }
  if (fd_ > 0)
    safe_close(&fd_);
  if (kvm_run_)
    munmap(kvm_run_, machine_->kvm_vcpu_mmap_size_);
}

/* Starting a vcpu is as simple as starting a thread on the host */
void Vcpu::Start() {
  thread_ = std::thread(&Vcpu::Process, this);
}


void Vcpu::Reset() {
  if (ioctl(fd_, KVM_SET_REGS, &default_registers_.regs) < 0)
    MV_PANIC("KVM_SET_REGS failed");
  if (ioctl(fd_, KVM_SET_SREGS, &default_registers_.sregs) < 0)
    MV_PANIC("KVM_SET_SREGS failed");
}

/* 
 * Intel CPUID Instruction Reference
 * https://www.intel.com/content/dam/develop/external/us/en/documents/ \
 * architecture-instruction-set-extensions-programming-reference.pdf
 * TODO: Hyper-V
 */
void Vcpu::SetupCpuid() {
  static const char cpu_model[48] = "Intel Xeon Processor (Cascadelake)";
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
      if (machine_->hypervisor_) {
        entry->ecx |= (1 << 31);
      } else {
        entry->ecx &= ~(1 << 31); // disable hypervisor mode now
      }
      entry->ebx = (vcpu_id_ << 24) | (machine_->num_vcpus_ << 16) | (entry->ebx & 0xFFFF);
      machine_->cpuid_version_ = entry->eax;
      machine_->cpuid_features_ = entry->edx;
      break;
    case 0xB: // CPU topology (cores = num_vcpus / 2, threads per core = 2)
      switch (entry->index) {
      case 0:
          entry->eax = 1;
          entry->ebx = 2;
          entry->ecx |= CPUID_TOPOLOGY_LEVEL_SMT;
          break;
      case 1:
          entry->eax = 2;
          entry->ebx = machine_->num_vcpus_ * 2;
          entry->ecx |= CPUID_TOPOLOGY_LEVEL_CORE;
          break;
      default:
          entry->eax = 0;
          entry->ebx = 0;
          entry->ecx |= CPUID_TOPOLOGY_LEVEL_INVALID;
      }
      entry->ecx = entry->index & 0xFF;
      entry->edx = vcpu_id_;
      break;
    case 0x80000002 ... 0x80000004: { // Setup CPU model string
      uint32_t offset = (entry->function - 0x80000002) * 16;
      memcpy(&entry->eax, cpu_model + offset, 16);
      break;
    }
    default:
      break;
    }
  }

  if (ioctl(fd_, KVM_SET_CPUID2, cpuid) < 0)
    MV_PANIC("KVM_SET_CPUID2 failed");

  free(cpuid);
}

void Vcpu::SetupMachineCheckException() {
  if ((machine_->cpuid_features_ & 0x4080) != 0x4080) {
    /* MCE / MCA not supported */
    return;
  }
  
  uint64_t mce_cap;
  uint64_t banks = ioctl(machine_->kvm_fd_, KVM_CHECK_EXTENSION, KVM_CAP_MCE);
  MV_ASSERT(ioctl(machine_->kvm_fd_, KVM_X86_GET_MCE_CAP_SUPPORTED, &mce_cap) == 0);
  mce_cap = (mce_cap & ~0xFF) | banks; 

  if (ioctl(fd_, KVM_X86_SETUP_MCE, &mce_cap) < 0) {
    MV_PANIC("failed to setup x86 MCE");
  }
}

uint64_t Vcpu::GetSupportedMsrFeature(uint index) {
  struct {
    kvm_msrs      msrs;
    kvm_msr_entry entries[1];
  } msrs = { 0 };

  msrs.entries[0].index = MSR_IA32_UCODE_REV;
  msrs.msrs.nmsrs = 1;
  if (ioctl(machine_->kvm_fd_, KVM_GET_MSRS, &msrs) != 1) {
    MV_PANIC("failed to get msr feature index=0x%x", index);
  }
  return msrs.entries[0].data;
}

void Vcpu::SetupModelSpecificRegisters() {
  struct {
    kvm_msrs      msrs;
    kvm_msr_entry entries[100];
  } msrs = { 0 };
	uint index = 0;

  msrs.entries[index++] = KVM_MSR_ENTRY(MSR_IA32_TSC, 0);
	msrs.entries[index++] = KVM_MSR_ENTRY(MSR_IA32_UCODE_REV, GetSupportedMsrFeature(MSR_IA32_UCODE_REV));

	msrs.msrs.nmsrs = index;
  auto ret = ioctl(fd_, KVM_SET_MSRS, &msrs);
	if (ret < 0)
		MV_PANIC("KVM_SET_MSRS failed");
}

/* Used for debugging sometimes */
void Vcpu::EnableSingleStep() {
  struct kvm_guest_debug debug = {
    .control = KVM_GUESTDBG_ENABLE | KVM_GUESTDBG_SINGLESTEP,
  };

  if (ioctl(fd_, KVM_SET_GUEST_DEBUG, &debug) < 0)
    MV_PANIC("KVM_SET_GUEST_DEBUG");
  
  single_step_ = true;
}

/* Memory trapped IO */
void Vcpu::ProcessMmio() {
  auto dm = machine_->device_manager();
  dm->FlushCoalescingMmioBuffer();

  auto *mmio = &kvm_run_->mmio;
  for (size_t i = mmio->len; i < sizeof(mmio->data); i++)
    mmio->data[i] = 0;
  dm->HandleMmio(mmio->phys_addr, mmio->data, mmio->len, mmio->is_write);
}

/* Traditional IN, OUT operations */
void Vcpu::ProcessIo() {
  auto dm = machine_->device_manager();
  dm->FlushCoalescingMmioBuffer();

  auto *io = &kvm_run_->io;
  uint8_t* data = reinterpret_cast<uint8_t*>(kvm_run_) + kvm_run_->io.data_offset;
  dm->HandleIo(io->port, data, io->size, io->direction, io->count);
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
  sprintf(name_, "mvisor-vcpu-%d", vcpu_id_);
  
  SetThreadName(name_);
  SetupSingalHandler();

  if (machine_->debug()) MV_LOG("%s started", name_);

  while (true) {
    while (machine_->IsPaused()) {
      machine_->WaitToResume();
    }
    if (!machine_->IsValid()) {
      break;
    }
    int ret = ioctl(fd_, KVM_RUN, 0);
    if (ret < 0 && errno != EINTR) {
      if (errno == EAGAIN) {
        continue;
      }
      MV_LOG("KVM_RUN failed vcpu=%d ret=%d errno=%d", vcpu_id_, ret, errno);
    }

    switch (kvm_run_->exit_reason)
    {
    case KVM_EXIT_MMIO:
      ProcessMmio();
      break;
    case KVM_EXIT_IO:
      ProcessIo();
      break;
    case KVM_EXIT_INTR:
      /* User interrupt */
      break;
    case KVM_EXIT_UNKNOWN:
      MV_PANIC("KVM_EXIT_UNKNOWN vcpu=%d", vcpu_id_);
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
    default:
      MV_PANIC("vcpu %d exit reason %d, expected KVM_EXIT_HLT(%d)", vcpu_id_,
        kvm_run_->exit_reason, KVM_EXIT_HLT);
    }

    /* Execute tasks after processing IO/MMIO */
    ExecuteTasks();
  }

quit:
  if (machine_->debug_) MV_LOG("%s ended", name_);
}

void Vcpu::Kick() {
  if (thread_.joinable()) {
    pthread_kill(thread_.native_handle(), SIG_USER_INTERRUPT);
  }
}

void Vcpu::Schedule(VoidCallback callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  tasks_.emplace_back(VcpuTask {
    .callback = callback
  });
  Kick();
}

void Vcpu::ExecuteTasks() {
  while (!tasks_.empty()) {
    VcpuTask task = tasks_.front();
    task.callback();
  
    mutex_.lock();
    tasks_.pop_front();
    mutex_.unlock();
  }
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


bool Vcpu::SaveState(MigrationWriter* writer) {
  std::stringstream prefix;
  prefix << "vcpu-" << vcpu_id_;
  writer->SetPrefix(prefix.str());

  VcpuState state;

  /* KVM vcpu events */
  kvm_vcpu_events events;
  MV_ASSERT(ioctl(fd_, KVM_GET_VCPU_EVENTS, &events) == 0);
  state.set_events(&events, sizeof(events));

  /* KVM MP State */
  kvm_mp_state mp_state;
  MV_ASSERT(ioctl(fd_, KVM_GET_MP_STATE, &mp_state) == 0);
  state.set_mp_state(mp_state.mp_state);

  /* Common regsiters */
  kvm_regs regs;
  MV_ASSERT(ioctl(fd_, KVM_GET_REGS, &regs) == 0);
  state.set_regs(&regs, sizeof(regs));

  /* XSAVE */
  kvm_xsave xsave;
  MV_ASSERT(ioctl(fd_, KVM_GET_XSAVE, &xsave) == 0);
  state.set_xsave(&xsave, sizeof(xsave));

  /* XCRS */
  kvm_xcrs xcrs;
  MV_ASSERT(ioctl(fd_, KVM_GET_XCRS, &xcrs) == 0);
  state.set_xcrs(&xcrs, sizeof(xcrs));

  /* Special registers */
  kvm_sregs sregs;
  MV_ASSERT(ioctl(fd_, KVM_GET_SREGS, &sregs) == 0);
  state.set_sregs(&sregs, sizeof(sregs));

  /* MSRS Indices */
  struct {
    kvm_msr_list  list;
    uint32_t      indices[100];
  } msr_list = { .list = { sizeof(msr_list.indices) / sizeof(uint32_t) } };
  MV_ASSERT(ioctl(machine_->kvm_fd_, KVM_GET_MSR_INDEX_LIST, &msr_list) == 0);

  /* MSRS */
  struct {
    kvm_msrs      msrs;
    kvm_msr_entry entries[100];
  } msrs = { .msrs = { .nmsrs = msr_list.list.nmsrs } };
  for (uint i = 0; i < msr_list.list.nmsrs; i++) {
    msrs.entries[i].index = msr_list.indices[i];
  }
  MV_ASSERT(ioctl(fd_, KVM_GET_MSRS, &msrs) == (int)msrs.msrs.nmsrs);
  state.set_msrs(&msrs, sizeof(msrs));

  /* LAPIC */
  kvm_lapic_state lapic;
  MV_ASSERT(ioctl(fd_, KVM_GET_LAPIC, &lapic) == 0);
  state.set_lapic(&lapic, sizeof(lapic));

  writer->WriteProtobuf("CPU", state);
  return true;
}

bool Vcpu::LoadState(MigrationReader* reader) {
  std::stringstream prefix;
  prefix << "vcpu-" << vcpu_id_;
  reader->SetPrefix(prefix.str());

  VcpuState state;
  if (!reader->ReadProtobuf("CPU", state)) {
    return false;
  }

  /* Special registers */
  kvm_sregs sregs;
  memcpy(&sregs, state.sregs().data(), sizeof(sregs));
  MV_ASSERT(ioctl(fd_, KVM_SET_SREGS, &sregs) == 0);

  /* Common regsiters */
  kvm_regs regs;
  memcpy(&regs, state.regs().data(), sizeof(regs));
  MV_ASSERT(ioctl(fd_, KVM_SET_REGS, &regs) == 0);

  /* XSAVE */
  kvm_xsave xsave;
  memcpy(&xsave, state.xsave().data(), sizeof(xsave));
  MV_ASSERT(ioctl(fd_, KVM_SET_XSAVE, &xsave) == 0);

  /* XCRS */
  kvm_xcrs xcrs;
  memcpy(&xcrs, state.xcrs().data(), sizeof(xcrs));
  MV_ASSERT(ioctl(fd_, KVM_SET_XCRS, &xcrs) == 0);

  /* MSRS */
  struct {
    kvm_msrs      msrs;
    kvm_msr_entry entries[100];
  } msrs;
  memcpy(&msrs, state.msrs().data(), sizeof(msrs));
  MV_ASSERT(ioctl(fd_, KVM_SET_MSRS, &msrs) == (int)msrs.msrs.nmsrs);

  /* KVM vcpu events */
  kvm_vcpu_events events;
  memcpy(&events, state.events().data(), sizeof(events));
  MV_ASSERT(ioctl(fd_, KVM_SET_VCPU_EVENTS, &events) == 0);
  
  /* KVM MP State */
  kvm_mp_state mp_state;
  mp_state.mp_state = state.mp_state();
  MV_ASSERT(ioctl(fd_, KVM_SET_MP_STATE, &mp_state) == 0);

  /* LAPIC */
  kvm_lapic_state lapic;
  memcpy(&lapic, state.lapic().data(), sizeof(lapic));
  MV_ASSERT(ioctl(fd_, KVM_SET_LAPIC, &lapic) == 0);

  return true;
}
