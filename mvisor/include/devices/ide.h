#ifndef _MVISOR_DEVICES_IDE_H
#define _MVISOR_DEVICES_IDE_H

#include "devices/device.h"
#include "devices/pci_device.h"
#include <cstdint>
#include <vector>

#define MAX_IDE_DRIVES 4


enum IdeStorageType {
  kIdeStorageTypeHarddisk,
  kIdeStorageTypeCdrom
};

class IdeStorageDevice : public StorageDevice {
 public:
  IdeStorageDevice(DiskImage* image);
  // void SetIndex(int index) { index_ = index; }
  void Reset();
  virtual void GetIdentityData(uint8_t* buffer, size_t buffer_size, ssize_t* nbytes) = 0;
  virtual void ProcessPacket(uint8_t* packet, ssize_t* nbytes);

  IdeStorageType type() { return type_; }
  // int index() { return index_; }
  // uint8_t status() { return status_; }
 protected:
  friend class AhciPort;
  IdeStorageType type_;
  // uint8_t status_;
  // uint8_t error_;
  // uint8_t index_;
};

class IdeCdromStorageDevice : public IdeStorageDevice {
 public:
  IdeCdromStorageDevice(DiskImage* image);
  virtual void GetIdentityData(uint8_t* buffer, size_t buffer_size, ssize_t* nbytes);
  virtual void ProcessPacket(uint8_t* packet, ssize_t* nbytes);
 private:
  uint8_t identify_data_[512];
  int drive_serial_;
  char drive_serial_string_[21];
  char drive_model_string_[41];
  char version_string_[41];
  uint64_t world_wide_name_;
};

class IdeHarddiskStorageDevice : public IdeStorageDevice {
 public:
  IdeHarddiskStorageDevice(DiskImage* image) : IdeStorageDevice(image) {
    type_ = kIdeStorageTypeHarddisk;
    name_ = "ide-harddisk";
  }
  virtual void GetIdentityData(uint8_t* buffer, size_t buffer_size, ssize_t* nbytes);
 private:
  uint8_t identify_data_[512];
};

struct IdePort {
  int index;
  uint8_t control;
  uint8_t status;
  uint8_t error;
  IdeStorageDevice* drive;
  uint16_t registers[14];
  size_t io_buffer_size;
  uint8_t* io_buffer;
  int io_buffer_index;
  ssize_t io_rw_count;
  int command;
};

class IdeControllerDevice : public PciDevice {
 public:
  IdeControllerDevice();
  void Connect();
  void Reset();
  void ResetPort(int index);
  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
 protected:
  IdePort ports_[2];
  std::vector<IdeStorageDevice*> drives_;
 private:
  void TrigerIrq(IdePort* port);
  void ProcessCommandBegin(IdePort* port);
  void ProcessCommandEnd(IdePort* port);
  void WriteControlPort(int index, uint64_t offset, uint8_t* data, uint32_t size);
  void ReadControlPort(int index, uint64_t offset, uint8_t* data, uint32_t size);
  void ReadIoBuffer(IdePort* port, uint8_t* data, uint32_t size);
  void WriteIoBuffer(IdePort* port, uint8_t* data, uint32_t size);
};

#endif // _MVISOR_DEVICES_IDE_H
