#ifndef _MVISOR_DEVICES_AHCI_H
#define _MVISOR_DEVICES_AHCI_H

#include "devices/pci_device.h"
#include "ahci_internal.h"
#include <array>

struct AHCICommandHeader {
  uint16_t    opts;
  uint16_t    prdtl;
  uint32_t    status;
  uint64_t    tbl_addr;
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

struct AHCIHostControlRegs {
  uint32_t    capabilities;
  uint32_t    global_host_control;
  uint32_t    irq_status;
  uint32_t    ports_implemented;
  uint32_t    version;
  uint32_t    command_completion_coalescing_control;
  uint32_t    command_completion_coalescing_ports;
  uint32_t    enclosure_management_location;
  uint32_t    enclosure_management_control;
};

class AhciHostDevice;

class AhciPort {
 public:
  AhciPort(DeviceManager* manager, AhciHostDevice* host);
  ~AhciPort();

  void Write(uint64_t offset, uint32_t value);
  void Read(uint64_t offset, uint32_t* data);
  void Reset();
  void CheckEngines();
  void CheckCommand();
  bool HandleCommand(int slot);
  void Register();
 private:
  friend class AhciHostDevice;
  DeviceManager* manager_;
  AhciHostDevice* host_;
  AHCIPortRegs port_control_;
  bool registered_ = false;
  uint8_t* command_list_;
  uint8_t* fis_;
};

class AhciHostDevice : public PciDevice {
 public:
  AhciHostDevice();
  ~AhciHostDevice();

  void Connect();
  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void CheckIrq();
  void ResetHost();

 private:
  int num_ports_;
  AHCIHostControlRegs host_control_;
  std::array<AhciPort*, AHCI_MAX_PORTS> ports_ = { 0 };
};

#endif // _MVISOR_DEVICES_AHCI_H
