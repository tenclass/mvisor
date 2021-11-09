#ifndef _MVISOR_DEVICES_IDE_STORAGE_H
#define _MVISOR_DEVICES_IDE_STORAGE_H

#include "devices/device.h"
#include "devices/ide/ide_storage.h"

#define IDE_MAX_REGISTERS 18

union IdeRegisters
{
  struct {
    uint8_t data;
    uint8_t features;
    
    uint8_t sectors0;
    uint8_t lba0;
    uint8_t lba1;
    uint8_t lba2;
  
    uint8_t devsel;
    uint8_t command;
  
    uint8_t sectors1;
    uint8_t lba3;
    uint8_t lba4;
    uint8_t lba5;
  
    uint8_t control;
    uint8_t alternative_status;
  } __attribute__((packed));
  uint8_t values[IDE_MAX_REGISTERS];

  uint8_t error;
  uint8_t status;
} __attribute__((packed));

struct IdeIo {
  ssize_t buffer_size;
  uint8_t* buffer;
  ssize_t position;
  ssize_t nbytes;
};

enum IdeStorageType {
  kIdeStorageTypeHarddisk,
  kIdeStorageTypeCdrom
};

enum IdeTransferType {
  kIdeTransferToDevice,
  kIdeTransferToHost
};

class IdePort;

class IdeStorageDevice : public StorageDevice {
 public:
  IdeStorageDevice(DiskImage* image);
  void Reset();
  void BindPort(IdePort* port);

  virtual void StartCommand();
  virtual void EndCommand();
  virtual void AbortCommand();
  virtual void StartTransfer(IdeTransferType type);
  virtual void EndTransfer(IdeTransferType type);
  virtual void ResetSignature();

  IdeStorageType type() { return type_; }
 protected:
  /* disk or cdrom */
  IdeStorageType type_;
  /* port_ will be set if the drive is selected */
  IdePort* port_;
  IdeRegisters* registers_;
  IdeIo *io_;
};


class IdeCdromStorageDevice : public IdeStorageDevice {
 public:
  IdeCdromStorageDevice(DiskImage* image);
  void StartCommand();
  void EndCommand();
  void StartTransfer(IdeTransferType type);
  void EndTransfer(IdeTransferType type);
 private:
  void ParseCommandPacket();
  void Atapi_IdentifyData();
  void Atapi_Inquiry();
  void Atapi_Read();
  void Atapi_TableOfContent();
  void SetError(int sense_key, int asc);

  uint8_t identify_data_[512];
  int drive_serial_;
  char drive_serial_string_[21];
  char drive_model_string_[41];
  char version_string_[41];
  uint64_t world_wide_name_;
  int sense_key_;
  int asc_;
};


class IdeHarddiskStorageDevice : public IdeStorageDevice {
 public:
  IdeHarddiskStorageDevice(DiskImage* image) : IdeStorageDevice(image) {
    type_ = kIdeStorageTypeHarddisk;
    name_ = "ide-harddisk";
  }
  virtual void GetIdentityData();
 private:
  uint8_t identify_data_[512];
};

#endif // _MVISOR_DEVICES_IDE_STORAGE_H
