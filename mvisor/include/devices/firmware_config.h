#ifndef _MVISOR_FIRMWARE_CONFIG_H
#define _MVISOR_FIRMWARE_CONFIG_H

#include "device.h"
#include <string>
#include <map>

enum FirmwareConfigEntryType {
  kFirmwareConfigEntryTypeString,
  kFirmwareConfigEntryTypeUInt16,
  kFirmwareConfigEntryTypeUInt32,
  kFirmwareConfigEntryTypeUInt64,
  kFirmwareConfigEntryTypeBytes,
  kFirmwareConfigEntryTypeFile
};

struct FirmwareConfigEntry {
  FirmwareConfigEntryType type;
  std::string file;
  std::string bytes;
};

class FirmwareConfigDevice : public Device {
 public:
  FirmwareConfigDevice(DeviceManager* manager);
  void OnWrite(uint64_t base, uint8_t* data, uint32_t size);
  void OnRead(uint64_t base, uint8_t* data, uint32_t size);

 private:
  void InitializeConfig();
  void DmaTransfer();
  void SetConfigBytes(uint16_t index, std::string bytes);
  void SetConfigUInt32(uint16_t index, uint32_t value);
  void SetConfigUInt16(uint16_t index, uint16_t value);

  uint16_t current_index_ = 0;
  uint32_t current_offset_ = 0;
  uint64_t dma_address_ = 0;
  std::map<uint16_t, std::string> arch_config_;
  std::map<uint16_t, std::string> config_;
};

#endif // _MVISOR_FIRMWARE_CONFIG_H
