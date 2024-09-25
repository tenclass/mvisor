#ifndef _MVISOR_WINDOWS_KERNEL_H
#define _MVISOR_WINDOWS_KERNEL_H

#include <stdint.h>
#include <string>

// hard code for windows 10 22h2
// https://www.vergiliusproject.com/kernels/x64/windows-10/22h2

#define EPROCESS_IN_KPROCESS_OFFSET               0x0
#define KTHREAD_IN_KPCR_OFFSET                    0x188
#define KPROCESS_IN_KTHREAD_OFFSET                0x220
#define ACTIVE_PROCESS_LINK_IN_EPROCESS_OFFSET    0x448
#define PROCESS_ID_IN_EPROCESS_OFFSET             0x440
#define PARENT_PROCESS_ID_IN_EPROCESS_OFFSET      0x540
#define PROCESS_NAME_IN_EPROCESS_OFFSET           0x5a8
#define DIRECTORY_TABLE_BASE_IN_EPROCESS_OFFSET   0x28
#define PEB_IN_EPROCESS_OFFSET                    0x550
#define IMAGE_BASE_ADDRESS_IN_PEB_OFFSET          0x10
#define LDR_IN_PEB_OFFSET                         0x18
#define LDR_IN_PEB32_OFFSET                       0xc

typedef uint16_t USHORT;
typedef wchar_t WCHAR;
typedef uint32_t ULONG;

struct _LIST_ENTRY32 {
  ULONG Flink;  //0x0
  ULONG Blink;  //0x4
}; 

// 0x10 bytes (sizeof)
struct _LIST_ENTRY {
  struct _LIST_ENTRY *Flink; // 0x0
  struct _LIST_ENTRY *Blink; // 0x8
};

//0x10 bytes (sizeof)
struct _UNICODE_STRING {
  USHORT Length;                                                          //0x0
  USHORT MaximumLength;                                                   //0x2
  WCHAR* Buffer;                                                          //0x8
}; 

//0x10 bytes (sizeof)
struct _UNICODE_STRING32 {
  USHORT Length;                                                          //0x0
  USHORT MaximumLength;                                                   //0x2
  ULONG  Buffer;                                                          //0x4
}; 

struct GuestProcess {
  void* eprocess;
  uint32_t pid;
  uint32_t ppid;
  uint64_t cr3;
  std::string name;
};

#endif
