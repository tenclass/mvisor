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

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/kvm.h>
#include <cstring>

#include "machine.h"
#include "logger.h"
#include "hyperv/hyperv.h"
#include "hyperv/cpuid.h"
#include "linuz/kvm_para.h"


#define MAX_KVM_MSR_ENTRIES           256
#define MAX_KVM_CPUID_ENTRIES         100
#define KVM_MSR_ENTRY(_index, _data)  (struct kvm_msr_entry) { .index = _index, .reserved = 0, .data = _data }


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

  PrepareX86Vcpu();
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
  /* Reset CPU registers, MSRs */
  LoadStateFrom(default_state_, false);
}

/* 
 * Intel CPUID Instruction Reference
 * https://www.intel.com/content/dam/develop/external/us/en/documents/ \
 * architecture-instruction-set-extensions-programming-reference.pdf
 * Model: Skylake-Server Compatible
 */
#define CPU_VERSION(family, model, stepping) \
  (((family & 0xF) << 8) | (((model >> 4) & 0xF) << 16) | ((model & 0xF) << 4) | (stepping & 0xF))

void Vcpu::SetupCpuid() {
  auto cpuid = (kvm_cpuid2*)new uint8_t[sizeof(kvm_cpuid2) + MAX_KVM_CPUID_ENTRIES * sizeof(kvm_cpuid_entry2)]();
  cpuid->nent = MAX_KVM_CPUID_ENTRIES;

  if (ioctl(machine_->kvm_fd_, KVM_GET_SUPPORTED_CPUID, cpuid) < 0) {
    MV_PANIC("failed to get supported CPUID");
  }

  for (uint i = 0; i < cpuid->nent; i++) {
    auto entry = &cpuid->entries[i];
    switch (entry->function)
    {
    case 0x0: // CPUID Signature
    case 0x80000000: // Extended CPUID Information
    {
      if (!machine_->vcpu_vendor_.empty()) {
        char vendor[13];
        strncpy(vendor, machine_->vcpu_vendor_.c_str(), 12);
        memcpy(&entry->ebx, &vendor[0], 4);
        memcpy(&entry->edx, &vendor[4], 4);
        memcpy(&entry->ecx, &vendor[8], 4);
      }
      break;
    }
    case 0x1: { // ACPI ID & Features
      entry->eax = CPU_VERSION(15, 1, 0);
      entry->ebx = (vcpu_id_ << 24) | (machine_->num_vcpus_ << 16) | (entry->ebx & 0xFFFF);

      bool tsc_deadline = ioctl(machine_->kvm_fd_, KVM_CHECK_EXTENSION, KVM_CAP_TSC_DEADLINE_TIMER);
      ALTER_FEATURE(entry->ecx, CPUID_EXT_TSC_DEADLINE_TIMER, tsc_deadline);
      ALTER_FEATURE(entry->ecx, CPUID_EXT_HYPERVISOR, machine_->hypervisor_);
      ALTER_FEATURE(entry->ecx, CPUID_EXT_PDCM, false); // Disable PMU
      ALTER_FEATURE(entry->edx, CPUID_HT, true);  // Max ACPI IDs reserved field is valid
      ALTER_FEATURE(entry->edx, CPUID_SS, false); // Self snoop

      cpuid_features_ = (uint64_t(entry->edx) << 32) | entry->ecx;
      break;
    }
    case 0x2: // Cache and TLB Information
      break;
    case 0x7: // Extended CPU features 7
      if (entry->index == 0) {
        // Disable HLE, RTM, MPX (Intel Memory Protection Extensions)
        entry->ebx &= (CPUID_7_0_EBX_FSGSBASE | CPUID_7_0_EBX_BMI1 |
          CPUID_7_0_EBX_AVX2 | CPUID_7_0_EBX_SMEP | CPUID_7_0_EBX_BMI2 |
          CPUID_7_0_EBX_ERMS | CPUID_7_0_EBX_INVPCID |
          CPUID_7_0_EBX_RDSEED | CPUID_7_0_EBX_ADX |
          CPUID_7_0_EBX_SMAP | CPUID_7_0_EBX_CLWB |
          CPUID_7_0_EBX_AVX512F | CPUID_7_0_EBX_AVX512DQ |
          CPUID_7_0_EBX_AVX512BW | CPUID_7_0_EBX_AVX512CD |
          CPUID_7_0_EBX_AVX512VL | CPUID_7_0_EBX_CLFLUSHOPT);
        // Disable PKU
        entry->ecx &= 0;
        entry->edx &= 0;
      }
      break;
    case 0xB: // CPU topology (cores = num_vcpus / 2, threads per core = 2)
      entry->edx = vcpu_id_;
      break;
    case 0xD:
      if (entry->index == 0) {
        entry->eax &= 0x2E7; // MPX is disabled in CPU features 7
      }
      break;
    case 0x80000001:
      entry->ecx &= ~(1U << 22); // Disable Topology Extensions
      break;
    case 0x80000002 ... 0x80000004: { // CPU Model String
      static const char cpu_model[48] = "Intel Compatible Processor";
      uint32_t offset = (entry->function - 0x80000002) * 16;
      memcpy(&entry->eax, cpu_model + offset, 16);
      break;
    }
    case 0x80000006: // Cache Line Information
    case 0x80000008: // Memory Address Size
      break;
    case 0x40000000 ... 0x4000FFFF:
      /* Move KVM CPUID to 0x40000100, leaving the place for Hyper-V */
      if (entry->function == 0x40000000) {
        entry->eax += 0x100;
      } else if (entry->function == 0x40000001) {
        entry->eax &= (
          KVM_FEATURE_CLOCKSOURCE | KVM_FEATURE_NOP_IO_DELAY | KVM_FEATURE_MMU_OP |
          KVM_FEATURE_CLOCKSOURCE2 | KVM_FEATURE_ASYNC_PF | KVM_FEATURE_STEAL_TIME |
          KVM_FEATURE_PV_EOI | KVM_FEATURE_PV_UNHALT | KVM_FEATURE_PV_TLB_FLUSH |       
          KVM_FEATURE_ASYNC_PF_VMEXIT | KVM_FEATURE_PV_SEND_IPI | KVM_FEATURE_POLL_CONTROL |
          KVM_FEATURE_PV_SCHED_YIELD | KVM_FEATURE_CLOCKSOURCE_STABLE_BIT
        );
      }
      entry->function += 0x100;
      break;
    default:
      /* Remove the function if not handled */
      memmove(entry, entry + 1, sizeof(*entry) * (cpuid->nent - i - 1));
      --i;
      --cpuid->nent;
      continue;
    }
  }

  /* Add Cache Parameters Leaf to Intel CPU */
  kvm_cpuid_entry2 cache_entries[5] = {
    { 0x4, 0, 1, 0x121, 0x1C0003F, 0x003F, 1 },
    { 0x4, 1, 1, 0x122, 0x1C0003F, 0x003F, 1 },
    { 0x4, 2, 1, 0x143, 0x3C0003F, 0x0FFF, 1 },
    { 0x4, 3, 1, 0x163, 0x3C0003F, 0x3FFF, 6 },
    { 0x4, 4, 1, 0, 0, 0, 0 }
  };
  for (int index = 0; index < 5; index++) {
    auto entry = &cpuid->entries[cpuid->nent++];
    *entry = cache_entries[index];
    entry->eax |= (machine_->num_cores_ - 1) << 26;
    if (index == 2) {
      entry->eax |= (machine_->num_threads_ - 1) << 14;
    } else if (index == 3) {
      entry->eax |= (machine_->num_vcpus_ - 1) << 14;
    }
  }

  /* Add Hyper-V functions to cpuid */
  if (machine_->hypervisor_) {
    SetupHyperV(cpuid);
  }

  if (ioctl(fd_, KVM_SET_CPUID2, cpuid) < 0) {
    MV_PANIC("KVM_SET_CPUID2 failed");
  }

  delete[] cpuid;
}

void Vcpu::SetupHyperV(kvm_cpuid2* cpuid) {
  auto hyperv_cpuid = (kvm_cpuid2*)new uint8_t[sizeof(kvm_cpuid2) + MAX_KVM_CPUID_ENTRIES * sizeof(kvm_cpuid_entry2)]();
  hyperv_cpuid->nent = MAX_KVM_CPUID_ENTRIES;

  if (ioctl(fd_, KVM_GET_SUPPORTED_HV_CPUID, hyperv_cpuid) < 0) {
    MV_ASSERT(ioctl(machine_->kvm_fd_, KVM_CHECK_EXTENSION, KVM_CAP_HYPERV));
    MV_ASSERT(ioctl(machine_->kvm_fd_, KVM_CHECK_EXTENSION, KVM_CAP_HYPERV_TIME));
    MV_ASSERT(ioctl(machine_->kvm_fd_, KVM_CHECK_EXTENSION, KVM_CAP_HYPERV_SYNIC));
    MV_ASSERT(ioctl(machine_->kvm_fd_, KVM_CHECK_EXTENSION, KVM_CAP_HYPERV_TLBFLUSH));
    MV_ASSERT(ioctl(machine_->kvm_fd_, KVM_CHECK_EXTENSION, KVM_CAP_HYPERV_SEND_IPI));
    MV_PANIC("failed to get supported Hyper-V CPUID. Please upgrade your kernel.");
  }

  for (uint i = 0; i < hyperv_cpuid->nent; i++) {
    auto entry = &hyperv_cpuid->entries[i];
    switch (entry->function)
    {
    case 0x40000000: // HV_CPUID_VENDOR_AND_MAX_FUNCTIONS
      entry->eax = HV_CPUID_IMPLEMENT_LIMITS;
      memcpy(&entry->ebx, "Microsoft Hv", 12);
      break;
    case 0x40000001: // HV_CPUID_INTERFACE
      memcpy(&entry->eax, "Hv#1\0\0\0\0\0\0\0\0\0\0\0\0", 16);
      break;
    case 0x40000002: // HV_CPUID_VERSION
    case 0x40000005: // HV_CPUID_IMPLEMENT_LIMITS
      /* use the default values */
      break;
    case 0x40000003: // HV_CPUID_FEATURES
      entry->eax &= (
        HV_VP_RUNTIME_AVAILABLE | HV_TIME_REF_COUNT_AVAILABLE | HV_REFERENCE_TSC_AVAILABLE |
        HV_APIC_ACCESS_AVAILABLE | HV_SYNIC_AVAILABLE | HV_HYPERCALL_AVAILABLE |
        HV_VP_INDEX_AVAILABLE | HV_SYNTIMERS_AVAILABLE
      );
      entry->ebx &= HV_POST_MESSAGES | HV_SIGNAL_EVENTS;
      entry->edx = HV_CPU_DYNAMIC_PARTITIONING_AVAILABLE;

      hyperv_features_ = entry->eax;
      break;
    case 0x40000004: // HV_CPUID_ENLIGHTMENT_INFO
      entry->eax = HV_APIC_ACCESS_RECOMMENDED | HV_RELAXED_TIMING_RECOMMENDED |
        HV_CLUSTER_IPI_RECOMMENDED | HV_REMOTE_TLB_FLUSH_RECOMMENDED;
      /* spinlock retry attempts */
      entry->ebx = 0x0FFF;
      break;
    default:
      entry->function = 0; // disabled
    }
    
    if (entry->function) {
      cpuid->entries[cpuid->nent++] = *entry;
    }
  }

  delete[] hyperv_cpuid;

  if (hyperv_features_ & HV_SYNIC_AVAILABLE) {
    struct kvm_enable_cap enable_cap;
    bzero(&enable_cap, sizeof(enable_cap));
    enable_cap.cap = KVM_CAP_HYPERV_SYNIC;
    MV_ASSERT(ioctl(fd_, KVM_ENABLE_CAP, &enable_cap) == 0);
  }
}

void Vcpu::SetupMachineCheckException() {
  if (((cpuid_features_ >> 32) & 0x4080) != 0x4080) {
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
  auto msrs = (kvm_msrs*)new uint8_t[sizeof(kvm_msrs) + 1 * sizeof(kvm_msr_entry)];
  msrs->entries[0].index = index;
  msrs->nmsrs = 1;
  if (ioctl(machine_->kvm_fd_, KVM_GET_MSRS, msrs) != 1) {
    MV_PANIC("failed to get msr feature index=0x%x", index);
  }

  auto feature = msrs->entries[0].data;
  delete[] msrs;
  return feature;
}

void Vcpu::SetupMsrIndices() {
  msr_indices_.clear();

  /* Setup common MSRS indices */
  auto msr_list = (kvm_msr_list*)new uint8_t[sizeof(kvm_msr_list) + 1000 * sizeof(uint32_t)];
  msr_list->nmsrs = 1000;
  MV_ASSERT(ioctl(machine_->kvm_fd_, KVM_GET_MSR_INDEX_LIST, msr_list) == 0);

  for (uint i = 0; i < msr_list->nmsrs; i++) {
    auto index = msr_list->indices[i];
    // MSR_IA32_BNDCFGS
    if (index == 0xD90) {
      continue;
    }
    // PMU is disabled
    if(index >= 0x300 && index < 0x400) {
      continue;
    }
    // MSR_IA32_PERFCTR0
    if (index >= 0xC1 && index <= 0xC8) {
      continue;
    }
    // P4/Xeon+ specific
    if(index >= 0x180 && index < 0x200) {
      continue;
    }
    if (!(cpuid_features_ & CPUID_EXT_VMX) && (index >= 0x480 && index < 0x500)) {
      continue;
    }
    msr_indices_.insert(index);
  }

  delete[] msr_list;

  /* Add up some MSR indices that we cannot get from supported list */
  if (hyperv_features_ & HV_SYNIC_AVAILABLE) {
    msr_indices_.insert(HV_X64_MSR_SCONTROL);
    msr_indices_.insert(HV_X64_MSR_SVERSION);
    msr_indices_.insert(HV_X64_MSR_SIMP);
    msr_indices_.insert(HV_X64_MSR_SIEFP);

    for (uint i = 0; i < HV_SINT_COUNT; i++) {
      msr_indices_.insert(HV_X64_MSR_SINT0 + i);
    }
  }

  if (hyperv_features_ & HV_SYNTIMERS_AVAILABLE) {
    for (uint i = 0; i < HV_STIMER_COUNT; i++) {
      msr_indices_.insert(HV_X64_MSR_STIMER0_CONFIG + i * 2);
      msr_indices_.insert(HV_X64_MSR_STIMER0_COUNT + i * 2);
    }
  }
}

void Vcpu::SetupModelSpecificRegisters() {
  auto msrs = (kvm_msrs*)new uint8_t[sizeof(kvm_msrs) + 100 * sizeof(kvm_msr_entry)];
  uint index = 0;

  /* UCODE is needed for MCE / MCA */
  msrs->entries[index++] = KVM_MSR_ENTRY(MSR_IA32_TSC, 0);
  msrs->entries[index++] = KVM_MSR_ENTRY(MSR_IA32_UCODE_REV, GetSupportedMsrFeature(MSR_IA32_UCODE_REV));

  if (hyperv_features_ & HV_SYNIC_AVAILABLE) {
    msrs->entries[index++] = KVM_MSR_ENTRY(HV_X64_MSR_SVERSION, HV_SYNIC_VERSION);
    msrs->entries[index++] = KVM_MSR_ENTRY(HV_X64_MSR_SCONTROL, 0);
    msrs->entries[index++] = KVM_MSR_ENTRY(HV_X64_MSR_SIMP, 0);
    msrs->entries[index++] = KVM_MSR_ENTRY(HV_X64_MSR_SIEFP, 0);
  }

  msrs->nmsrs = index;
  auto ret = ioctl(fd_, KVM_SET_MSRS, msrs);
  if (ret < 0) {
    MV_PANIC("KVM_SET_MSRS failed");
  }
  delete[] msrs;  
}

/* Used for debugging sometimes */
void Vcpu::EnableSingleStep() {
  struct kvm_guest_debug debug = {
    .control = KVM_GUESTDBG_ENABLE | KVM_GUESTDBG_SINGLESTEP,
    .pad = 0,
    .arch = {
      .debugreg = {0}
    }
  };

  if (ioctl(fd_, KVM_SET_GUEST_DEBUG, &debug) < 0)
    MV_PANIC("KVM_SET_GUEST_DEBUG");
  
  single_step_ = true;
}

/* Memory trapped IO */
void Vcpu::ProcessMmio() {
  auto dm = machine_->device_manager();
  dm->FlushCoalescingMmioBuffer();

  auto mmio = &kvm_run_->mmio;
  for (size_t i = mmio->len; i < sizeof(mmio->data); i++)
    mmio->data[i] = 0;
  dm->HandleMmio(mmio->phys_addr, mmio->data, mmio->len, mmio->is_write);
}

/* Traditional IN, OUT operations */
void Vcpu::ProcessIo() {
  auto dm = machine_->device_manager();
  dm->FlushCoalescingMmioBuffer();

  auto io = &kvm_run_->io;
  uint8_t* data = reinterpret_cast<uint8_t*>(kvm_run_) + kvm_run_->io.data_offset;
  dm->HandleIo(io->port, data, io->size, io->direction, io->count);
}

/* Hyper-V SynIc / Hypercalls */
void Vcpu::ProcessHyperV() {
  auto& hyperv_exit = kvm_run_->hyperv;
  switch (hyperv_exit.type)
  {
  case KVM_EXIT_HYPERV_SYNIC:
    switch (hyperv_exit.u.synic.msr) {
    case HV_X64_MSR_SCONTROL:
      if (machine_->debug_) {
        MV_LOG("msr_hv_synic_control = 0x%lx", hyperv_exit.u.synic.control);
      }
      hyperv_synic_.enabled = hyperv_exit.u.synic.control & 1;
      break;
    case HV_X64_MSR_SIMP:
      if (machine_->debug_) {
        MV_LOG("msr_hv_synic_msg_page = 0x%lx", hyperv_exit.u.synic.msg_page);
      }
      if (hyperv_exit.u.synic.msg_page & 1) {
        hyperv_synic_.message_address = hyperv_exit.u.synic.msg_page & 0xFFF;
      } else {
        hyperv_synic_.message_address = 0;
      }
      break;
    case HV_X64_MSR_SIEFP:
      if (machine_->debug_) {
        MV_LOG("msr_hv_synic_evt_page = 0x%lx", hyperv_exit.u.synic.evt_page);
      }
      if (hyperv_exit.u.synic.evt_page & 1) {
        hyperv_synic_.message_address = hyperv_exit.u.synic.evt_page & 0xFFF;
      } else {
        hyperv_synic_.event_address = 0;
      }
      break;
    default:
      MV_PANIC("invalid msr=%d", hyperv_exit.u.synic.msr);
    }
    break;
  case KVM_EXIT_HYPERV_HCALL:
    if (machine_->debug_) {
      MV_WARN("KVM_EXIT_HYPERV_HCALL not implemented");
    }
    break;
  default:
    MV_PANIC("invalid hyperv exit type=%d", hyperv_exit.type);
  }
}

/* To wake up a vcpu thread, the easist way is to send a signal */
void Vcpu::SignalHandler(int signum) {
  // Do nothing now ...
  MV_UNUSED(signum);
}

/* Vcpu thread only response to SIG_USER at the moment */
void Vcpu::SetupSignalHandler() {
  sigset_t sigset;
  sigemptyset(&sigset);
  pthread_sigmask(SIG_BLOCK, &sigset, nullptr);

  signal(SIG_USER_INTERRUPT, Vcpu::SignalHandler);
}

void Vcpu::PrepareX86Vcpu() {
  SetupCpuid();
  SetupMsrIndices();
  SetupModelSpecificRegisters();

  /* Setup MCE for booting Linux */
  SetupMachineCheckException();

  /* Save default registers for system reset */
  SaveStateTo(default_state_);
}

/* Set the lowest priority to vcpu thread */
void Vcpu::SetupSchedPriority(int priority) {
  // https://man7.org/linux/man-pages/man7/sched.7.html
  if (priority == 0) {
    // linux set priority 0 to thread as default
    return;
  }
  MV_ASSERT(priority >= MIN_NICE && priority <= MAX_NICE);
  MV_ASSERT(nice(priority));
}

/* Initialize and executing a vCPU thread */
void Vcpu::Process() {
  current_vcpu_ = this;
  sprintf(name_, "mvisor-vcpu-%d", vcpu_id_);
  SetThreadName(name_);
  
  SetupSignalHandler();
  SetupSchedPriority(machine_->vcpu_priority_);

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
    case KVM_EXIT_HYPERV:
      ProcessHyperV();
      break;
    case KVM_EXIT_INTR:
      /* User interrupt */
      break;
    case KVM_EXIT_UNKNOWN:
      MV_PANIC("KVM_EXIT_UNKNOWN vcpu=%d", vcpu_id_);
      break;
    case KVM_EXIT_SHUTDOWN:
      /* A hard reset request reached here */
      MV_LOG("KVM_EXIT_SHUTDOWN vcpu=%d", vcpu_id_);
      machine_->Reset();
      break;
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
    .callback = std::move(callback)
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

void Vcpu::SaveStateTo(VcpuState& state) {
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

  /* FPU regsiters */
  kvm_fpu fpu;
  MV_ASSERT(ioctl(fd_, KVM_GET_FPU, &fpu) == 0);
  state.set_fpu(&fpu, sizeof(fpu));

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

  /* MSRS */
  size_t msrs_size = sizeof(kvm_msrs) + MAX_KVM_MSR_ENTRIES * sizeof(kvm_msr_entry);
  auto msrs = (kvm_msrs*)new uint8_t[msrs_size]();
  for (auto index : msr_indices_) {
    msrs->entries[msrs->nmsrs++].index = index;
  }
  int nr = ioctl(fd_, KVM_GET_MSRS, msrs);
  if (nr != (int)msrs->nmsrs) {
    MV_PANIC("failed to get MSR(%d) index=0x%x", nr, msrs->entries[nr].index);
  }
  state.set_msrs(msrs, msrs_size);
  delete[] msrs;

  /* LAPIC */
  kvm_lapic_state lapic;
  MV_ASSERT(ioctl(fd_, KVM_GET_LAPIC, &lapic) == 0);
  state.set_lapic(&lapic, sizeof(lapic));

  /* TSC kHz */
  int64_t tsc_khz = ioctl(fd_, KVM_GET_TSC_KHZ);
  if (tsc_khz > 0) {
    state.set_tsc_khz(tsc_khz);
  }

  /* Guest debugs */
  kvm_guest_debug debug;
  MV_ASSERT(ioctl(fd_, KVM_GET_DEBUGREGS, &debug) == 0);
  state.set_debug_regs(&debug, sizeof(debug));
  
  /* CPUID (save for future use) */
  size_t cpuid_size = sizeof(kvm_cpuid2) + MAX_KVM_CPUID_ENTRIES * sizeof(kvm_cpuid_entry2);
  auto cpuid = (kvm_cpuid2*)new uint8_t[cpuid_size]();
  cpuid->nent = MAX_KVM_CPUID_ENTRIES;
  MV_ASSERT(ioctl(fd_, KVM_GET_CPUID2, cpuid) == 0);
  state.set_cpuid(cpuid, cpuid_size);
  delete[] cpuid;
}

void Vcpu::LoadStateFrom(VcpuState& state, bool load_cpuid) {
  if (load_cpuid) {
    auto cpuid = new uint8_t[sizeof(kvm_cpuid2) + MAX_KVM_CPUID_ENTRIES * sizeof(kvm_cpuid_entry2)];
    memcpy(cpuid, state.cpuid().data(), state.cpuid().size());
    MV_ASSERT(ioctl(fd_, KVM_SET_CPUID2, cpuid) == 0);
    delete[] cpuid;
    
    /* Reset MSR indices */
    SetupMsrIndices();
  }

  /* Special registers */
  kvm_sregs sregs;
  memcpy(&sregs, state.sregs().data(), sizeof(sregs));
  MV_ASSERT(ioctl(fd_, KVM_SET_SREGS, &sregs) == 0);

  /* LAPIC */
  kvm_lapic_state lapic;
  memcpy(&lapic, state.lapic().data(), sizeof(lapic));
  MV_ASSERT(ioctl(fd_, KVM_SET_LAPIC, &lapic) == 0);

  /* Common regsiters */
  kvm_regs regs;
  memcpy(&regs, state.regs().data(), sizeof(regs));
  MV_ASSERT(ioctl(fd_, KVM_SET_REGS, &regs) == 0);

  /* FPU */
  kvm_fpu fpu;
  memcpy(&fpu, state.fpu().data(), sizeof(fpu));
  MV_ASSERT(ioctl(fd_, KVM_SET_FPU, &fpu) == 0);

  /* XSAVE */
  kvm_xsave xsave;
  memcpy(&xsave, state.xsave().data(), sizeof(xsave));
  MV_ASSERT(ioctl(fd_, KVM_SET_XSAVE, &xsave) == 0);

  /* XCRS */
  kvm_xcrs xcrs;
  memcpy(&xcrs, state.xcrs().data(), sizeof(xcrs));
  MV_ASSERT(ioctl(fd_, KVM_SET_XCRS, &xcrs) == 0);

  /* TSC must be set before KVM_SET_MSRS, otherwise it may cause the guest to get time in a mess */
  // https://patchwork.kernel.org/project/kvm/patch/1443418711-24106-4-git-send-email-haozhong.zhang@intel.com/#15469651
  if (ioctl(machine_->kvm_fd_, KVM_CHECK_EXTENSION, KVM_CAP_TSC_CONTROL)) {
    MV_ASSERT(ioctl(fd_, KVM_SET_TSC_KHZ, state.tsc_khz()) == 0);
  } else {
    MV_WARN("KVM_CAP_TSC_CONTROL was not supported");
  }

  /* MSRS */
  auto msrs = (kvm_msrs*)new uint8_t[sizeof(kvm_msrs) + MAX_KVM_MSR_ENTRIES * sizeof(kvm_msr_entry)];
  memcpy(msrs, state.msrs().data(), state.msrs().size());
  int nmsrs = msrs->nmsrs;
  while (nmsrs > 0) {
    auto ret = ioctl(fd_, KVM_SET_MSRS, msrs);
    MV_ASSERT(ret >= 0);
    if (ret < nmsrs) {
      MV_LOG("Failed to set MSR 0x%x=0x%x. Maybe kernel version is too old?",
        msrs->entries[ret].index, msrs->entries[ret].data);
      // Skip the failed one
      nmsrs -= ret + 1;
      memmove(msrs->entries, &msrs->entries[ret + 1], nmsrs * sizeof(kvm_msr_entry));
    } else {
      nmsrs -= ret;
    }
  }
  delete[] msrs;

  /* KVM vcpu events */
  kvm_vcpu_events events;
  memcpy(&events, state.events().data(), sizeof(events));
  MV_ASSERT(ioctl(fd_, KVM_SET_VCPU_EVENTS, &events) == 0);
  
  /* KVM MP State */
  kvm_mp_state mp_state;
  mp_state.mp_state = state.mp_state();
  MV_ASSERT(ioctl(fd_, KVM_SET_MP_STATE, &mp_state) == 0);

  /* Guest debugs */
  kvm_guest_debug debug;
  memcpy(&debug, state.debug_regs().data(), sizeof(debug));
  MV_ASSERT(ioctl(fd_, KVM_SET_GUEST_DEBUG, &debug) == 0);
}

bool Vcpu::SaveState(MigrationWriter* writer) {
  std::stringstream prefix;
  prefix << "vcpu-" << vcpu_id_;
  writer->SetPrefix(prefix.str());

  VcpuState state;
  SaveStateTo(state);

  writer->WriteProtobuf("CURRENT", state);
  writer->WriteProtobuf("DEFAULT", default_state_);
  return true;
}

bool Vcpu::LoadState(MigrationReader* reader) {
  std::stringstream prefix;
  prefix << "vcpu-" << vcpu_id_;
  reader->SetPrefix(prefix.str());

  if (!reader->ReadProtobuf("DEFAULT", default_state_)) {
    return false;
  }

  VcpuState state;
  if (!reader->ReadProtobuf("CURRENT", state)) {
    return false;
  }

  LoadStateFrom(state, true);
  return true;
}
