#ifndef _MVISOR_DEVICES_AHCI_STORAGE_H
#define _MVISOR_DEVICES_AHCI_STORAGE_H

#include "devices/device.h"
#include <string>

enum AhciStorageType {
  kAhciStorageTypeDisk,
  kAhciStorageTypeCdrom
};

/**
 * NCQFrame is the same as a Register H2D FIS (described in SATA 3.2),
 * but some fields have been re-mapped and re-purposed, as seen in
 * SATA 3.2 section 13.6.4.1 ("READ FPDMA QUEUED")
 *
 * cmd_fis[3], feature 7:0, becomes sector count 7:0.
 * cmd_fis[7], device 7:0, uses bit 7 as the Force Unit Access bit.
 * cmd_fis[11], feature 15:8, becomes sector count 15:8.
 * cmd_fis[12], count 7:0, becomes the NCQ TAG (7:3) and RARC bit (0)
 * cmd_fis[13], count 15:8, becomes the priority value (7:6)
 * bytes 16-19 become an le32 "auxiliary" field.
 */
struct NCQFrame {
  uint8_t fis_type;
  uint8_t c;
  uint8_t command;
  uint8_t sector_count_low;  /* (feature 7:0) */
  uint8_t lba0;
  uint8_t lba1;
  uint8_t lba2;
  uint8_t fua;               /* (device 7:0) */
  uint8_t lba3;
  uint8_t lba4;
  uint8_t lba5;
  uint8_t sector_count_high; /* (feature 15:8) */
  uint8_t tag;               /* (count 0:7) */
  uint8_t prio;              /* (count 15:8) */
  uint8_t icc;
  uint8_t control;
  uint8_t aux0;
  uint8_t aux1;
  uint8_t aux2;
  uint8_t aux3;
} __attribute__((packed));

class AHCICommandHeader;
class AhciPort;

class AhciStorageDevice : public StorageDevice {
 public:
  AhciStorageDevice(DiskImage* image);
  AhciStorageType type() { return type_; }
  
  void Reset();
  bool PioTransfer(AHCICommandHeader* cmd_header, NCQFrame* frame, uint8_t* buffer, size_t nbytes);
  virtual bool ProcessCommand(AHCICommandHeader* cmd_header, NCQFrame* frame);

 protected:
  friend class AhciPort;
  AhciStorageType type_;
  uint8_t status_;
  uint8_t error_;
  AhciPort* port_;
};

class AhciCdromStorageDevice : public AhciStorageDevice {
 public:
  AhciCdromStorageDevice(DiskImage* image);
  virtual bool ProcessCommand(AHCICommandHeader* cmd_header, NCQFrame* frame);

 protected:
  size_t io_buffer_size_ = 0;
  uint8_t* io_buffer_ = nullptr;

 private:
  bool AtapiIndentity();
  uint8_t identify_data_[512] = { 0 };
  int drive_serial_;
  char drive_serial_string_[21];
  char drive_model_string_[41];
  char version_string_[41];
  uint64_t world_wide_name_;
};

class AhciHarddiskStorageDevice : public AhciStorageDevice {
 public:
  AhciHarddiskStorageDevice(DiskImage* image);
  virtual bool ProcessCommand(AHCICommandHeader* cmd_header, NCQFrame* frame);
};

#endif // _MVISOR_DEVICES_AHCI_STORAGE_H
