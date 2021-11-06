#include "devices/debug_console.h"

DebugConsoleDevice::DebugConsoleDevice() {
  name_ = "debugcon";
  
  AddIoResource(kIoResourceTypePio, 0x402, 1);
}

void DebugConsoleDevice::Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  putchar(*data);
}

void DebugConsoleDevice::Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  *data = 0xe9;
}
