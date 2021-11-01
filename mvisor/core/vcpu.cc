#include "vcpu.h"

extern "C" {
#include "apicdef.h"
}
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/prctl.h>
#include <sys/mman.h>
#include <cstring>
#include "machine.h"
#include "logger.h"
#include "cpuid.h"

Vcpu::Vcpu(Machine* machine, int vcpu_id)
    : machine_(machine), vcpu_id_(vcpu_id) {
  fd_ = ioctl(machine_->vm_fd_, KVM_CREATE_VCPU, vcpu_id_);
  MV_ASSERT(fd_ > 0);

  kvm_run_ = (struct kvm_run*)mmap(nullptr, machine_->kvm_vcpu_mmap_size_,
    PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  MV_ASSERT(kvm_run_ != MAP_FAILED);

  int coalesced_offset = ioctl(fd_, KVM_CHECK_EXTENSION, KVM_CAP_COALESCED_MMIO);
  if (coalesced_offset)
    mmio_ring_ = (struct kvm_coalesced_mmio_ring*)((char*)kvm_run_ + coalesced_offset * PAGE_SIZE);
  
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
}

void Vcpu::Process() {
  sprintf(thread_name_, "vcpu-%d", vcpu_id_);
  prctl(PR_SET_NAME, thread_name_);
  MV_LOG("%s started", thread_name_);

  DeviceManager* device_manager = machine_->device_manager();

  kvm_cpu_setup_cpuid(machine_->kvm_fd_, fd_);

  for (;;) {
    int ret = ioctl(fd_, KVM_RUN, 0);
    if (ret < 0) {
      MV_LOG("KVM_RUN failed vcpu=%d ret=%d", vcpu_id_, ret);
    }

    switch (kvm_run_->exit_reason)
    {
    case KVM_EXIT_UNKNOWN:
      break;
    case KVM_EXIT_HLT:
      goto check;
    case KVM_EXIT_DEBUG:
      PrintRegisters();
      getchar();
      break;
    case KVM_EXIT_IO: {
      auto *io = &kvm_run_->io;
      uint8_t* data = reinterpret_cast<uint8_t*>(kvm_run_) + kvm_run_->io.data_offset;
      device_manager->HandleIo(io->port, data, io->size, io->direction, io->count);
      break;
    }
    case KVM_EXIT_MMIO: {
      if (mmio_ring_) {
        const int max_entries = ((PAGE_SIZE - sizeof(struct kvm_coalesced_mmio_ring)) / \
          sizeof(struct kvm_coalesced_mmio));
        while (mmio_ring_->first != mmio_ring_->last) {
          struct kvm_coalesced_mmio *m;
          m = &mmio_ring_->coalesced_mmio[mmio_ring_->first];
          device_manager->HandleMmio(m->phys_addr, m->data, m->len, 1);
          mmio_ring_->first = (mmio_ring_->first + 1) % max_entries;
        }
      }
      auto *mmio = &kvm_run_->mmio;
      device_manager->HandleMmio(mmio->phys_addr, mmio->data, mmio->len, mmio->is_write);
      break;
    }
    default:
      MV_PANIC("exit reason %d, expected KVM_EXIT_HLT(%d)\n",
        kvm_run_->exit_reason, KVM_EXIT_HLT);
    }
  }

check:
  MV_LOG("%s ended", thread_name_);
}


static inline void print_dtable(FILE* fp, const char *name, struct kvm_dtable *dtable)
{
  fprintf(fp, " %s                 %016lx  %08hx\n",
    name, (uint64_t) dtable->base, (uint16_t) dtable->limit);
}

static inline void print_segment(FILE* fp, const char *name, struct kvm_segment *seg)
{
  fprintf(fp, " %s       %04hx      %016lx  %08x  %02hhx    %x %x   %x  %x %x %x %x\n",
    name, (uint16_t) seg->selector, (uint64_t) seg->base, (uint32_t) seg->limit,
    (uint8_t) seg->type, seg->present, seg->dpl, seg->db, seg->s, seg->l, seg->g, seg->avl);
}

void Vcpu::PrintRegisters() {
  unsigned long cr0, cr2, cr3;
  unsigned long cr4, cr8;
  unsigned long rax, rbx, rcx;
  unsigned long rdx, rsi, rdi;
  unsigned long rbp,  r8,  r9;
  unsigned long r10, r11, r12;
  unsigned long r13, r14, r15;
  unsigned long rip, rsp;
  struct kvm_sregs sregs;
  unsigned long rflags;
  struct kvm_regs regs;
  int i;

  if (ioctl(fd_, KVM_GET_REGS, &regs) < 0)
    MV_PANIC("KVM_GET_REGS failed");

  rflags = regs.rflags;

  rip = regs.rip; rsp = regs.rsp;
  rax = regs.rax; rbx = regs.rbx; rcx = regs.rcx;
  rdx = regs.rdx; rsi = regs.rsi; rdi = regs.rdi;
  rbp = regs.rbp; r8  = regs.r8;  r9  = regs.r9;
  r10 = regs.r10; r11 = regs.r11; r12 = regs.r12;
  r13 = regs.r13; r14 = regs.r14; r15 = regs.r15;

  FILE* output = stdout;
  fprintf(output, "\n Registers:\n");
  fprintf(output,   " ----------\n");
  fprintf(output, " rip: %016lx   rsp: %016lx flags: %016lx\n", rip, rsp, rflags);
  fprintf(output, " rax: %016lx   rbx: %016lx   rcx: %016lx\n", rax, rbx, rcx);
  fprintf(output, " rdx: %016lx   rsi: %016lx   rdi: %016lx\n", rdx, rsi, rdi);
  fprintf(output, " rbp: %016lx    r8: %016lx    r9: %016lx\n", rbp, r8,  r9);
  fprintf(output, " r10: %016lx   r11: %016lx   r12: %016lx\n", r10, r11, r12);
  fprintf(output, " r13: %016lx   r14: %016lx   r15: %016lx\n", r13, r14, r15);

  if (ioctl(fd_, KVM_GET_SREGS, &sregs) < 0)
    MV_PANIC("KVM_GET_REGS failed");

  cr0 = sregs.cr0; cr2 = sregs.cr2; cr3 = sregs.cr3;
  cr4 = sregs.cr4; cr8 = sregs.cr8;

  fprintf(output, " cr0: %016lx   cr2: %016lx   cr3: %016lx\n", cr0, cr2, cr3);
  fprintf(output, " cr4: %016lx   cr8: %016lx\n", cr4, cr8);
  fprintf(output, "\n Segment registers:\n");
  fprintf(output,   " ------------------\n");
  fprintf(output, " register  selector  base              limit     type  p dpl db s l g avl\n");
  print_segment(output, "cs ", &sregs.cs);
  print_segment(output, "ss ", &sregs.ss);
  print_segment(output, "ds ", &sregs.ds);
  print_segment(output, "es ", &sregs.es);
  print_segment(output, "fs ", &sregs.fs);
  print_segment(output, "gs ", &sregs.gs);
  print_segment(output, "tr ", &sregs.tr);
  print_segment(output, "ldt", &sregs.ldt);
  print_dtable(output, "gdt", &sregs.gdt);
  print_dtable(output, "idt", &sregs.idt);

  fprintf(output, "\n APIC:\n");
  fprintf(output,   " -----\n");
  fprintf(output, " efer: %016lx  apic base: %016lx  nmi: %s\n",
    (uint64_t) sregs.efer, (uint64_t) sregs.apic_base,
    (true ? "disabled" : "enabled"));

  fprintf(output, "\n Interrupt bitmap:\n");
  fprintf(output,   " -----------------\n");
  for (i = 0; i < (KVM_NR_INTERRUPTS + 63) / 64; i++)
    fprintf(output, " %016lx", (uint64_t) sregs.interrupt_bitmap[i]);
  fprintf(output, "\n");
}
