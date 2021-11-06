#include "devices/serial_port.h"
#include "logger.h"

SerialPortDevice::SerialPortDevice() {
  name_ = "serial-port";
  
  AddIoResource(kIoResourceTypePio, 0x3F8, 8); // COM1
  AddIoResource(kIoResourceTypePio, 0x2F8, 8); // COM2
  AddIoResource(kIoResourceTypePio, 0x3E8, 8); // COM3
  AddIoResource(kIoResourceTypePio, 0x2E8, 8); // COM4
}

void SerialPortDevice::Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  MV_LOG("%s ignore base=0x%lx offset=0x%lx size=%d",
    name_.c_str(), ir.base, offset, size);
}

void SerialPortDevice::Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  MV_LOG("%s ignore base=0x%lx offset=0x%lx size=%d",
    name_.c_str(), ir.base, offset, size);
}
