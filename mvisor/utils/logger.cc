#include "logger.h"
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <cerrno>
#include <cstdlib>
#include <cstdint>
#include <linux/kvm.h>
#include <sys/prctl.h>

void Log(LogType type, const char* file, int line, const char* function, const char* format, ...)
{
  char message[512];
  va_list args;
  va_start(args, format);
  vsnprintf(message, 512, format, args);
  va_end(args);

  time_t now = time(NULL);
  struct tm* tm_now;
  char timestr[100];
  tm_now = localtime(&now);
  strftime(timestr, 100, "%Y-%m-%d %H:%M:%S", tm_now);

  if (type == kLogTypeDebug) {
    printf("[%s] %s:%d %s() %s\n", timestr, file, line, function, message);
  } else if (type == kLogTypeError) {
    fprintf(stderr,"[%s] %s:%d %s() %s\n", timestr, file, line, function, message);
  } else if (type == kLogTypePanic) {
    fprintf(stderr,"[%s] %s:%d %s() fatal error: %s\n", timestr, file, line, function, message);
    if (errno != 0) {
      fprintf(stderr, "errno=%d, %s\n", errno, strerror(errno));
    }
    exit(1);
  }
}

void SaveToFile(const char* path, void* data, size_t size) {
  FILE* fp = fopen(path, "wb");
  fwrite(data, size, 1, fp);
  fclose(fp);
}

void DumpHex(void* data, size_t size) {
  uint8_t* ptr = (uint8_t*)data;
  printf("%08x  ", 0);
  for (int i = 0; i < (int)size;) {
    printf("%02x ", ptr[i++]);
    if (i % 16 == 0) {
      printf("\n%08x  ", i);
    } else if (i % 8 == 0) {
      printf(" ");
    }
  }
  if (size % 16 != 0) {
    printf("\n");
  }
}

void SetThreadName(const char* name) {
  prctl(PR_SET_NAME, name);
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

void PrintRegisters(struct kvm_regs& regs, struct kvm_sregs& sregs) {
  unsigned long cr0, cr2, cr3;
  unsigned long cr4, cr8;
  unsigned long rax, rbx, rcx;
  unsigned long rdx, rsi, rdi;
  unsigned long rbp,  r8,  r9;
  unsigned long r10, r11, r12;
  unsigned long r13, r14, r15;
  unsigned long rip, rsp;
  unsigned long rflags;
  int i;

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
