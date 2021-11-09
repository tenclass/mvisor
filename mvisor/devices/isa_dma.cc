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

#define REG_COMMAND        0x8
#define REG_STATUS         0x9
#define WR_SINGLE_MASK_BIT 0xA
#define WR_MODE_REG        0xB
#define W_FLIPFLOP_RESET   0xC
#define WR_MASTER_CLEAR    0xD
#define WR_MASK_CLEAR      0xE
#define WR_ALL_MASK_BITS   0xF


IsaDmaDevice::IsaDmaDevice() {
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
  case 0 ... ISA_DMA_REGISTER_NUM - 1:
    controller->registers[offset][controller->flipflop[offset]++ & 1] = value;
    break;
  case REG_COMMAND:
    controller->command = *data;
    break;
  case REG_STATUS:
    MV_PANIC("not implemented");
    break;
  case WR_SINGLE_MASK_BIT:
    controller->mask = 0xF | (value & 0xF);
    break;
  case WR_MODE_REG:
    controller->mode = value;
    break;
  case W_FLIPFLOP_RESET:
    if (value > 0) {
      bzero(controller->flipflop, sizeof(controller->flipflop));
    }
    break;
  case WR_MASTER_CLEAR:
    bzero(controller, sizeof(*controller));
    break;
  default:
    MV_PANIC("%s unhandled base=0x%lx offset=0x%lx size=%d data=%x",
      name_.c_str(), ir.base, offset, size, *data);
  }
}

void IsaDmaDevice::Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  if (ir.base == 0x0080) { // Page registers
    *data = page_registers_[offset];
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
  case 0 ... ISA_DMA_REGISTER_NUM - 1:
    *data = controller->registers[offset][controller->flipflop[offset]++ & 1];
    break;
  case WR_MODE_REG:
    *data = controller->mode;
    break;
  case WR_SINGLE_MASK_BIT:
    *data = controller->mask;
    break;
  case REG_COMMAND:
    *data = controller->command;
    break;
  default:
    MV_PANIC("%s unhandled base=0x%lx offset=0x%lx size=%d data=%x",
      name_.c_str(), ir.base, offset, size, *data);
  }
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
  
  void* host = manager_->TranslateGuestMemory(gpa);
  memcpy(host, data, count);
  *transferred = count;
}
