#ifndef _MVISOR_DEVICES_IDE_PORT_H
#define _MVISOR_DEVICES_IDE_PORT_H

#include <cstdint>
#include "devices/ide/ide_storage.h"
#include "devices/device.h"

class DeviceManager;
class IdeControllerDevice;
class IdePort {
 public:
  IdePort(DeviceManager* manager, int index);
  IdePort(DeviceManager* manager, IdeControllerDevice* controller, int index);
  virtual ~IdePort();

  void virtual RaiseIrq();
  void virtual LowerIrq();
  void virtual Reset();
  void AttachDevice(IdeStorageDevice* device);
  void WritePort(uint64_t offset, uint8_t* data, uint32_t size);
  void ReadPort(uint64_t offset, uint8_t* data, uint32_t size);
  void ReadControlPort(uint64_t offset, uint8_t* data, uint32_t size);
  void WriteControlPort(uint64_t offset, uint8_t* data, uint32_t size);

  const std::vector<IdeStorageDevice*>& attached_devices() { return attached_devices_; }
  inline IdeRegisters* registers() { return &registers_; }
  inline IdeIo* io() { return &io_; }

 protected:
  void ReadIoBuffer(uint8_t* data, uint32_t size);
  void WriteIoBuffer(uint8_t* data, uint32_t size);

  friend class IdeControllerDevice;
  DeviceManager* manager_;
  IdeControllerDevice* controller_;
  int index_;
  std::vector<IdeStorageDevice*> attached_devices_;
  IdeStorageDevice* drive_;

  IdeRegisters registers_;
  IdeIo io_;
};


#endif // _MVISOR_DEVICES_IDE_PORT_H
