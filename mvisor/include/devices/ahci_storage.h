#ifndef _MVISOR_DEVICES_AHCI_STORAGE_H
#define _MVISOR_DEVICES_AHCI_STORAGE_H

#include "devices/device.h"

class AhciStorageDevice : public StorageDevice {
 public:
  AhciStorageDevice(DiskImage* image);
};

class AhciCdromStorageDevice : public AhciStorageDevice {
 public:
  AhciCdromStorageDevice(DiskImage* image);
};

class AhciHarddiskStorageDevice : public AhciStorageDevice {
 public:
  AhciHarddiskStorageDevice(DiskImage* image);
};

#endif // _MVISOR_DEVICES_AHCI_STORAGE_H
