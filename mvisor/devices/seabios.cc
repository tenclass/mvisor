#include "devices/seabios.h"

SeaBiosDevice::SeaBiosDevice(DeviceManager* manager)
  : Device(manager) {
  name_ = "seabios";
  
  AddIoResource(kIoResourceTypePio, 0x402, 1);
}

void SeaBiosDevice::OnWrite(uint64_t base, uint8_t* data, uint32_t size) {
  putchar(*data);
}

void SeaBiosDevice::OnRead(uint64_t base, uint8_t* data, uint32_t size) {
  *data = 0xe9;
}
