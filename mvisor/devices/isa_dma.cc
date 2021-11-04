#include "devices/isa_dma.h"
#include <cstring>
#include "device_manager.h"
#include "logger.h"

#define WR_START_ADDR_04   0x0
#define WR_COUNT_04        0x1
#define WR_START_ADDR_15   0x2
#define WR_COUNT_15        0x3
#define WR_START_ADDR_26   0x4
#define WR_COUNT_26        0x5
#define WR_START_ADDR_37   0x6
#define WR_COUNT_37        0x7

#define R_COMMAND          0x8
#define WR_SINGLE_MASK_BIT 0xA
#define WR_MODE_REG        0xB
#define W_FLIPFLOP_RESET   0xC
#define WR_MASTER_CLEAR    0xD
#define WR_ALL_MASK_BITS   0xF


IsaDmaDevice::IsaDmaDevice(DeviceManager* manager)
  : Device(manager) {
  name_ = "isa-dma";

  bzero(controllers_, sizeof(controllers_));
  bzero(page_registers_, sizeof(page_registers_));
  
  /* 0000 - 001F - DMA1 controller */
  AddIoResource(kIoResourceTypePio, 0x0000, 0x10, "dma-ctrl-1");
  AddIoResource(kIoResourceTypePio, 0x00C0, 0x1F, "dma-ctrl-2");
  AddIoResource(kIoResourceTypePio, 0x0080, 0x0C, "dma-page-regs");
}

void IsaDmaDevice::Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  uint8_t value = *data;
  // MV_LOG("%s write offset=0x%lx size=%d data=%x", name_.c_str(), ir.base + offset, size, value);
  if (ir.base == 0x0080) { // Page registers
    page_registers_[offset] = value;
    return;
  }

  int controller_index = 0;
  if (ir.base == 0x00C0) {
    controller_index = 1;
    offset >>= 1;
  }
  auto controller = &controllers_[controller_index];
  switch (offset)
  {
  case WR_MASTER_CLEAR:
    bzero(controller, sizeof(*controller));
    break;
  case WR_MODE_REG:
    controller->mode = value;
    break;
  case WR_SINGLE_MASK_BIT:
    controller->mask = 0xF | (value & 0xF);
    break;
  case W_FLIPFLOP_RESET:
    if (value > 0) {
      MV_ASSERT(value == 0xff);
      bzero(controller->flipflop, sizeof(controller->flipflop));
    }
    break;
  default:
    if (offset < ISA_DMA_REGISTER_NUM) {
      controller->registers[offset][controller->flipflop[offset]++ & 1] = value;
    } else {
      MV_PANIC("%s ignore base=0x%lx offset=0x%lx size=%d data=%x",
        name_.c_str(), ir.base, offset, size, *data);
    }
  }
}

void IsaDmaDevice::Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  MV_PANIC("%s ignore base=0x%lx offset=0x%lx size=%d",
    name_.c_str(), ir.base, offset, size);
}

void IsaDmaDevice::TransferChannelData(uint8_t channel, void* data, size_t size, size_t* transferred) {
  /* Currently only floppy uses isa dma */
  MV_ASSERT(channel == 2);

  auto controller = &controllers_[0];
  uint64_t gpa = (page_registers_[1] << 16) |
    (controller->registers[WR_START_ADDR_26][1] << 8) |
    (controller->registers[WR_START_ADDR_26][0]);

  size_t count = (controller->registers[WR_COUNT_26][1] << 8) |
    (controller->registers[WR_COUNT_26][0]);
  ++count;
  MV_ASSERT(count <= size);
  
  // MV_LOG("gpa=0x%lx count=0x%lx size=0x%lx", gpa, count, size);
  void* host = manager_->TranslateGuestMemory(gpa);

  memcpy(host, data, count);
  *transferred = count;
}
