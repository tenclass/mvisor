#ifndef _MVISOR_DEVICES_ISA_DMA_H
#define _MVISOR_DEVICES_ISA_DMA_H

#include "device.h"

#define ISA_DMA_REGISTER_NUM 8

struct IsaDmaController {
  uint8_t registers[ISA_DMA_REGISTER_NUM][2];
  int flipflop[ISA_DMA_REGISTER_NUM];
  uint8_t mode;
  uint8_t mask;
};

class IsaDmaDevice : public Device {
 public:
  IsaDmaDevice(DeviceManager* manager);
  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void TransferChannelData(uint8_t channel, void* data, size_t size, size_t* transferred);
 private:
  void WriteRegister(IsaDmaController* controller, uint64_t offset, uint8_t value);
  IsaDmaController controllers_[2];
  uint8_t page_registers_[16];
};

#endif // _MVISOR_DEVICES_ISA_DMA_H
