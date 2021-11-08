#include "devices/dummy.h"
#include <cstring>
#include "logger.h"

DummyDevice::DummyDevice() {
  name_ = "dummy";
  
  /* Legacy ioport setup */
  /* PORT 0x0020-0x003F - 8259A PIC 1 */
  AddIoResource(kIoResourceTypePio, 0x0020, 2, "PIC1");

  /* PORT 0040-005F - PIT - PROGRAMMABLE INTERVAL TIMER (8253, 8254) */
  AddIoResource(kIoResourceTypePio, 0x0040, 4, "PIT");

  /* PORT 0x00A0-0x00AF - 8259A PIC 2 */
  AddIoResource(kIoResourceTypePio, 0x00A0, 2, "PIC2");

  /* PORT 00ED */
  AddIoResource(kIoResourceTypePio, 0x00ED, 1, "Unknown");

  /* PORT 00F0-00FF - Math co-processor */
  AddIoResource(kIoResourceTypePio, 0x00F0, 2, "Math Processor");

  /* PORT 01F0-01F7 */
  // AddIoResource(kIoResourceTypePio, 0x01F0, 8, "IDE Primary");
  // AddIoResource(kIoResourceTypePio, 0x0170, 8, "IDE Secondary");
  AddIoResource(kIoResourceTypePio, 0x01E8, 8, "IDE Tertiary");
  AddIoResource(kIoResourceTypePio, 0x0168, 8, "IDE Quaternary");

  /* PORT 0278-027A - PARALLEL PRINTER PORT (usually LPT1, sometimes LPT2) */
  AddIoResource(kIoResourceTypePio, 0x0278, 3, "Parallel LPT1");

  /* PORT 02F2 DOS access this port */
  AddIoResource(kIoResourceTypePio, 0x02F2, 6, "PMC for Susi");

  /* PORT 0378-037A - PARALLEL PRINTER PORT (usually LPT2, sometimes LPT3) */
  AddIoResource(kIoResourceTypePio, 0x0378, 3, "Parallel LPT2");

  /* PORT 06F2 DOS access this port */
  AddIoResource(kIoResourceTypePio, 0x06F2, 8, "Unknown");
  
  /* PORT A20-A24 IBM Token Ring */
  AddIoResource(kIoResourceTypePio, 0x0A20, 5, "IBM Token Ring");
  
  /* PORT AE0C-AE20 */
  AddIoResource(kIoResourceTypePio, 0xAE0C, 20, "Pci Hotplug");

  /* HPET MMIO */
  AddIoResource(kIoResourceTypeMmio, 0xFED00000, 0x400, "HPET");
}

void DummyDevice::Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  // Do nothing
  MV_LOG("%s ignore %s write base=0x%lx offset=0x%lx size=%d",
    ir.type == kIoResourceTypeMmio ? "mmio" : "pio",
    ir.name, ir.base, offset, size);
}

void DummyDevice::Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  // Do nothing
  /*
  MV_LOG("%s ignore %s read base=0x%lx offset=0x%lx size=%d",
    ir.type == kIoResourceTypeMmio ? "mmio" : "pio",
    ir.name, ir.base, offset, size);
  */
  memset(data, 0xFF, size);
}
