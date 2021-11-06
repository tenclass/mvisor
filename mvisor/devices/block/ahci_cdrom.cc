#include "devices/ahci_storage.h"
#include "logger.h"

AhciCdromStorageDevice::AhciCdromStorageDevice(DiskImage* image)
  : AhciStorageDevice(image)
{
  name_ = "cdrom";
};
