#include "devices/floppy.h"
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include "logger.h"
#include "device_manager.h"
#include "devices/isa_dma.h"
#include "images/image.h"

#define REG_STATUS_A        0x3F0
#define REG_STATUS_B        0x3F1
#define REG_DIGITAL_OUTPUT  0x3F2
#define REG_TAPE_DRIVE      0x3F3
#define REG_MAIN_STATUS     0x3F4
#define REG_DATARATE_SELECT 0x3F4
#define REG_DATA_FIFO       0x3F5
#define REG_DIGITAL_INPUT   0x3F7
#define REG_CONFIG_CONTROL  0x3F7

#define DOR_RESET           (1<<2)
#define DOR_IRQ             (1<<3)
#define DOR_MOTOR0          (1<<4)

#define MSR_READY           (1<<7)
#define MSR_DIO             (1<<6)
#define MSR_NO_DMA          (1<<5)
#define MSR_COMMAND_BUSY    (1<<4)

#define CMD_SPECIFY         0x3
#define CMD_READ_DATA       0x6
#define CMD_RECALIBRATE     0x7
#define CMD_SENSE_INTERRUPT 0x8
#define CMD_READ_ID         0xA
#define CMD_SEEK            0xF


FloppyStorageDevice::FloppyStorageDevice(DiskImage* image)
  : StorageDevice(image) {
  name_ = "floppy";

  AddIoResource(kIoResourceTypePio, 0x3F0, 6); // 3F0-3F5
  AddIoResource(kIoResourceTypePio, 0x3F7, 1); // 3F7
}

FloppyStorageDevice::~FloppyStorageDevice() {
}

void FloppyStorageDevice::Connect() {
  Reset();
  StorageDevice::Connect();
}

void FloppyStorageDevice::Reset() {
  memset(st_, 0, sizeof(st_));
  dor_ = ccr_ = dsr_ = 0;
  fifo_index_ = fifo_length_ = 0;
  head_ = cylinder_ = 0;
  sector_ = 1;
  // FDD ready
  msr_ = MSR_READY;

  if (dma_device_ == nullptr) {
    // Check dma device if exists
    dma_device_ = dynamic_cast<IsaDmaDevice*>(manager_->LookupDeviceByName("isa-dma"));
  }
}

void FloppyStorageDevice::Seek(uint8_t cylinder, uint8_t head, uint8_t sector) {
  cylinder_ = cylinder;
  head_ = head;
  sector_ = sector;
}

void FloppyStorageDevice::SenseInterrupt() {
  msr_ = MSR_READY | MSR_DIO;
  fifo_length_ = 2;
  fifo_index_ = 0;
  fifo_[0] = st_[0];
  fifo_[1] = 0; // current cylinder
  manager_->SetIrq(6, 0);
}

void FloppyStorageDevice::Recalibrate() {
  Seek(0, 0, 1);
  msr_ = MSR_READY;
  fifo_index_ = 0;
  manager_->SetIrq(6, 0);
  manager_->SetIrq(6, 1);
}

void FloppyStorageDevice::ReadId() {
  msr_ = MSR_READY | MSR_DIO;
  fifo_length_ = 7;
  fifo_index_ = 0;
  fifo_[0] = st_[0];
  fifo_[1] = st_[1];
  fifo_[2] = st_[2];
  fifo_[3] = cylinder_;
  fifo_[4] = head_;
  fifo_[5] = sector_;
  fifo_[6] = 2; // 512 bytes sector
  manager_->SetIrq(6, 1);
}

void FloppyStorageDevice::Seek() {
  // lower 2 bits is drive number
  head_ = fifo_[1] >> 2;
  cylinder_ = fifo_[2];
  fifo_index_ = 0;
  fifo_length_ = 0;
  msr_ = MSR_READY | MSR_NO_DMA;
  manager_->SetIrq(6, 1);
}

static unsigned int chs_to_lba(int cyl,int head,int sector) {
  sector--; 
  unsigned int lba = 18 * 2 * cyl; 
  lba += head * 18; 
  lba += sector; 
  return lba;
}

static void lba_to_chs(unsigned int lba, uint8_t* cyl, uint8_t* head, uint8_t* sector)
{
  *cyl    = lba / (2 * 18);
  *head   = ((lba % (2 * 512)) / 512);
  *sector = ((lba % (2 * 512)) % 512 + 1);
}

void FloppyStorageDevice::ReadData() {
  // MT mode should be on
  MV_ASSERT(fifo_[0] & 0x80);
  // Should I need to do this?
  Seek(fifo_[2], fifo_[3], fifo_[4]);
  unsigned int lba = chs_to_lba(fifo_[2], fifo_[3], fifo_[4]);
  int count = fifo_[6];
  size_t nbytes = count * 512;
  void* data = valloc(nbytes);
  MV_ASSERT(data);

  image_->Read(data, lba, count);

  MV_ASSERT(dma_device_);
  size_t ntransferred;
  dma_device_->TransferChannelData(2, data, nbytes, &ntransferred);
  // MV_LOG("read disk at offset=0x%lx bytes=0x%lx return=0x%lx", offset, nbytes, ntransferred);

  lba += ntransferred;
  lba_to_chs(lba, &cylinder_, &head_, &sector_);
  
  msr_ = MSR_READY | MSR_DIO;
  fifo_length_ = 7;
  fifo_index_ = 0;
  fifo_[0] = st_[0];
  fifo_[1] = st_[1];
  fifo_[2] = st_[2];
  fifo_[3] = cylinder_;
  fifo_[4] = head_;
  fifo_[5] = sector_;
  fifo_[6] = 2; // 512 bytes sector
  manager_->SetIrq(6, 1);
}

void FloppyStorageDevice::Specify() {
  /* Step rate time, Head load time, Head unload time */
}

void FloppyStorageDevice::WriteFifo(uint8_t value) {
  msr_ |= MSR_COMMAND_BUSY;
  fifo_[fifo_index_++] = value;
  switch(fifo_[0] & 0xf) {
    case CMD_READ_DATA:
      if (fifo_index_ >= 9)
        ReadData();
      break;
    case CMD_RECALIBRATE:
      if (fifo_index_ >= 2)
        Recalibrate();
      break;
    case CMD_SENSE_INTERRUPT:
      if (fifo_index_ >= 1)
        SenseInterrupt();
      break;
    case CMD_READ_ID:
      if (fifo_index_ >= 2)
        ReadId();
      break;
    case CMD_SEEK:
      if (fifo_index_ >= 3)
        Seek();
      break;
    case CMD_SPECIFY:
      if (fifo_index_ >= 2)
        Specify();
      break;
    default:
      MV_PANIC("unimplemented command 0x%02x", value);
  }
}

void FloppyStorageDevice::Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  uint64_t port = ir.base + offset;
  uint8_t value = *data;
  // MV_LOG("%s write port=0x%lx size=%d data=%x", name_.c_str(), port, size, value);
  switch (port)
  {
  case REG_DIGITAL_OUTPUT:
    if (value & DOR_RESET) {
      Reset();
      manager_->SetIrq(6, 0);
      manager_->SetIrq(6, 1);
    }
    dor_ = value & ~DOR_RESET;
    break;
  case REG_DATA_FIFO:
    WriteFifo(value);
    break;
  case REG_CONFIG_CONTROL:
    ccr_ = value;
    dsr_ = (dsr_ & ~0b11) | (value & 0b11);
    break;
  default:
    MV_PANIC("unhandled port %x", port);
    break;
  }
}

void FloppyStorageDevice::Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  uint64_t port = ir.base + offset;
  switch (port)
  {
  case REG_MAIN_STATUS:
    *data = msr_;
    break;
  case REG_DATA_FIFO:
    *data = fifo_[fifo_index_++];
    if (fifo_index_ >= fifo_length_) {
      fifo_index_ = 0;
      msr_ = MSR_READY | MSR_NO_DMA;
      manager_->SetIrq(6, 0);
    }
    break;
  default:
    MV_PANIC("unhandled port %x", port);
    break;
  }
  // MV_LOG("%s read 0x%lx size=%d data=0x%x", name_.c_str(), port, size, *data);
}

