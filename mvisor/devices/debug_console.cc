#include "devices/debug_console.h"

DebugConsoleDevice::DebugConsoleDevice(DeviceManager* manager)
  : Device(manager) {
  name_ = "debugcon";
  
  AddIoResource(kIoResourceTypePio, 0x402, 1);
}

void DebugConsoleDevice::OnWrite(uint64_t base, uint8_t* data, uint32_t size) {
  putchar(*data);
}

void DebugConsoleDevice::OnRead(uint64_t base, uint8_t* data, uint32_t size) {
  *data = 0xe9;
}
