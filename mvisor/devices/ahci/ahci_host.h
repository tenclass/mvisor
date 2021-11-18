#ifndef _MVISOR_DEVICES_AHCI_H
#define _MVISOR_DEVICES_AHCI_H

#include "pci_device.h"
#include "ahci_port.h"
#include <array>

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
  uint32_t    extended_capabilities;
  uint32_t    bohc; // BIOS/OS handoff control and status

	// 0x2C - 0x9F, Reserved
	uint8_t     reserved[0xA0-0x2C];
 
	// 0xA0 - 0xFF, Vendor specific registers
	uint8_t     vendor[0x100-0xA0];
} __attribute__((packed));

class AhciHost;

class AhciHost : public PciDevice {
 public:
  AhciHost();
  ~AhciHost();

  void Connect();
  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void CheckIrq();
  void ResetHost();

 private:
  int num_ports_;
  AHCIHostControlRegs host_control_;
  std::array<AhciPort*, 32> ports_ = { 0 };
};

#endif // _MVISOR_DEVICES_AHCI_H
