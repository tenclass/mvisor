#include "devices/floppy.h"
#include "logger.h"

FloppyDevice::FloppyDevice(DeviceManager* manager)
  : Device(manager) {
  name_ = "floppy";
  
  AddIoResource(kIoResourceTypePio, 0x3F0, 7);
}

void FloppyDevice::Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  MV_PANIC("%s ignore base=0x%lx offset=0x%lx size=%d",
    name_.c_str(), ir.base, offset, size);
}

void FloppyDevice::Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  MV_PANIC("%s ignore base=0x%lx offset=0x%lx size=%d",
    name_.c_str(), ir.base, offset, size);
}
