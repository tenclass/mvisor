#include "devices/dummy.h"
#include "logger.h"

DummyDevice::DummyDevice(DeviceManager* manager)
  : Device(manager) {
  name_ = "dummy";
  
  
	/* Legacy ioport setup */
	/* 0000 - 001F - DMA1 controller */
  AddIoResource(kIoResourceTypePio, 0x0000, 32);

	/* 0x0020 - 0x003F - 8259A PIC 1 */
  AddIoResource(kIoResourceTypePio, 0x0020, 2);

	/* PORT 0040-005F - PIT - PROGRAMMABLE INTERVAL TIMER (8253, 8254) */
  AddIoResource(kIoResourceTypePio, 0x0040, 4);

	/* 0x00A0 - 0x00AF - 8259A PIC 2 */
  AddIoResource(kIoResourceTypePio, 0x00A0, 2);

	/* 00C0 - 001F - DMA2 controller */
  AddIoResource(kIoResourceTypePio, 0x00C0, 32);

	/* PORT 00ED - DUMMY PORT FOR DELAY??? */
  AddIoResource(kIoResourceTypePio, 0x00ed, 1);

	/* 0x00F0 - 0x00FF - Math co-processor */
  AddIoResource(kIoResourceTypePio, 0x00f0, 2);

	/* PORT 0278-027A - PARALLEL PRINTER PORT (usually LPT1, sometimes LPT2) */
  AddIoResource(kIoResourceTypePio, 0x0278, 3);

	/* PORT 0378-037A - PARALLEL PRINTER PORT (usually LPT2, sometimes LPT3) */
  AddIoResource(kIoResourceTypePio, 0x0378, 3);

	/* PORT 03D4-03D5 - COLOR VIDEO - CRT CONTROL REGISTERS */
  AddIoResource(kIoResourceTypePio, 0x03d4, 2);
}

void DummyDevice::OnWrite(uint64_t base, uint8_t* data, uint32_t size) {
  // Do nothing
  MV_LOG("Dummy OnWrite for %s base=0x%lx size=%d", name_.c_str(), base, size);
}

void DummyDevice::OnRead(uint64_t base, uint8_t* data, uint32_t size) {
  // Do nothing
  MV_LOG("Dummy OnRead for %s base=0x%lx size=%d", name_.c_str(), base, size);
}
