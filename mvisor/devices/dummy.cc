/* 
 * MVisor
 * Copyright (C) 2021 Terrence <terrence@tenclass.com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <cstring>
#include "logger.h"
#include "device.h"
#include "machine.h"

/* All IO devices not implemented, put them here to ignore IO access */
class DummyDevice : public Device {
 public:
  DummyDevice() {
    /* Legacy ioport setup */
  
    /* 0000 - 001F - DMA1 controller */
    AddIoResource(kIoResourceTypePio, 0x0000, 0x10, "DMA Controller 1");
    AddIoResource(kIoResourceTypePio, 0x00C0, 0x1F, "DMA Controller 2");
    AddIoResource(kIoResourceTypePio, 0x0080, 0x0C, "DMA Page Registers");

    /* PORT 0x0020-0x003F - 8259A PIC 1 */
    AddIoResource(kIoResourceTypePio, 0x0020, 2, "PIC 1");

    /* PORT 0040-005F - PIT - PROGRAMMABLE INTERVAL TIMER (8253, 8254) */
    AddIoResource(kIoResourceTypePio, 0x0040, 4, "PIT");

    /* PORT 0x00A0-0x00AF - 8259A PIC 2 */
    AddIoResource(kIoResourceTypePio, 0x00A0, 2, "PIC 2");

    /* PORT 00ED */
    AddIoResource(kIoResourceTypePio, 0x00ED, 1, "IO Delay");

    /* PORT 00F0-00FF - Math co-processor */
    AddIoResource(kIoResourceTypePio, 0x00F0, 2, "Math Processor");

    /* PORT 01F0-01F7 */
    AddIoResource(kIoResourceTypePio, 0x01F0, 8, "IDE Primary");
    AddIoResource(kIoResourceTypePio, 0x0170, 8, "IDE Secondary");
    AddIoResource(kIoResourceTypePio, 0x01E8, 8, "IDE Tertiary");
    AddIoResource(kIoResourceTypePio, 0x0168, 8, "IDE Quaternary");
    AddIoResource(kIoResourceTypePio, 0x03F0, 8, "IDE Primary Control");
    AddIoResource(kIoResourceTypePio, 0x0370, 8, "IDE Secondary Control");

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

  void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    // Do nothing
    if (resource->base == 0x00ED) {
      /* IO Delay */
      return;
    }
    if (manager_->machine()->debug()) {
      MV_LOG("%s ignore %s write base=0x%lx offset=0x%lx data=0x%lx size=%d",
        resource->type == kIoResourceTypeMmio ? "MMIO" : "PIO",
        resource->name, resource->base, offset, *(uint64_t*)data, size);
    }
  }

  void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    // Do nothing
    /*
    MV_LOG("%s ignore %s read base=0x%lx offset=0x%lx size=%d",
      resource->type == kIoResourceTypeMmio ? "MMIO" : "PIO",
      resource->name, resource->base, offset, size);
    */
    memset(data, 0xFF, size);
  }

};

DECLARE_DEVICE(DummyDevice);
