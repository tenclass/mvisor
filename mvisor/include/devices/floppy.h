#ifndef _MVISOR_DEVICES_FLOPPY_H
#define _MVISOR_DEVICES_FLOPPY_H

#include "devices/device.h"

class IsaDmaDevice;
class FloppyStorageDevice : public StorageDevice {
 public:
  FloppyStorageDevice(DiskImage* image);
  ~FloppyStorageDevice();

  void Connect();
  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);

 private:
  void Reset();
  void ReadData();
  void Recalibrate();
  void SenseInterrupt();
  void ReadId();
  void Specify();
  void Seek();
  void Seek(uint8_t cylinder, uint8_t head, uint8_t sector);
  void WriteFifo(uint8_t value);
  void ExecuteCommand();

  // controls the motor and DMA
  uint8_t dor_  = 0;
  // fd status
  uint8_t msr_ = 0;
  // configuration control register
  uint8_t ccr_ = 0;
  uint8_t dsr_ = 0;
  // fifo
  uint8_t fifo_index_, fifo_length_;
  // read sector data in fifo_ when using PIO mode
  uint8_t fifo_[512 + 12];
  // error status
  uint8_t st_[3] = { 0 };
  // position
  uint8_t cylinder_, head_, sector_;
  
  IsaDmaDevice* dma_device_ = nullptr;
};

#endif // _MVISOR_DEVICES_FLOPPY_H
