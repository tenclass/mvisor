#include "devices/ahci_storage.h"
#include "logger.h"

AhciHarddiskStorageDevice::AhciHarddiskStorageDevice(DiskImage* image) 
  : AhciStorageDevice(image)
{
  name_ = "harddisk";
}
