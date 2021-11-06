#include "devices/ahci_storage.h"
#include "logger.h"

AhciStorageDevice::AhciStorageDevice(DiskImage* image) : StorageDevice(image) {
  name_ = "ahci-storage";
};
