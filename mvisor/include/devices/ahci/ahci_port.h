#ifndef __MVISOR_DEVICES_AHCI_PORT_H
#define __MVISOR_DEVICES_AHCI_PORT_H

#include <cstdint>
#include "devices/ide/ide_storage.h"
#include "devices/ide/ide_port.h"
#include "devices/device.h"

struct AHCICommandHeader {
  uint16_t    options;
  uint16_t    command_table_length;
  uint32_t    bytes;
  uint64_t    command_table_base;
  uint32_t    reserved[4];
} __attribute__((packed));

struct NativeCommandTransferState {
  AHCICommandHeader *command_header;
  uint32_t sector_count;
  uint64_t lba;
  uint8_t tag;
  uint8_t command;
  uint8_t slot;
  bool used;
  bool halt;
};

struct AHCIPortRegs {
  uint32_t    command_list_base;
  uint32_t    command_list_base_high;
  uint32_t    fis_base;
  uint32_t    fis_base_high;
  uint32_t    irq_status;
  uint32_t    irq_mask;
  uint32_t    command;
  uint32_t    unused0;
  uint32_t    task_flie_data;
  uint32_t    signature;
  uint32_t    sata_status;
  uint32_t    sata_control;
  uint32_t    sata_error;
  uint32_t    sata_active;
  uint32_t    command_issue;
  uint32_t    reserved;
};

class DeviceManager;
class AhciHostDevice;

class AhciPort : public IdePort {
 public:
  AhciPort(DeviceManager* manager, AhciHostDevice* host, int index);
  virtual ~AhciPort();

  void AttachDevice(IdeStorageDevice* device);
  void Write(uint64_t offset, uint32_t value);
  void Read(uint64_t offset, uint32_t* data);
  void Reset();
  void RaiseIrq();
  void LowerIrq();
  void TrigerIrq(int irqbit);
  void UpdateSetupPio(uint32_t size);
 private:
  void UpdateRegisterD2H();
  bool HandleCommand(int slot);
  void CheckEngines();
  void CheckCommand();

  friend class AhciHostDevice;
  AhciHostDevice* host_ = nullptr;
  AHCIPortRegs port_control_;
  IdeRegisters ide_regs_;
  IdeIo ide_io_;
  bool register_fis_posted_ = false;
  uint8_t* command_list_ = nullptr;
  uint8_t* res_fis_ = nullptr;
  AHCICommandHeader* current_command_ = nullptr;
  int index_;
};

#endif // __MVISOR_DEVICES_AHCI_PORT_H
