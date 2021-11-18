#include "storage_device.h"
#include "logger.h"

StorageDevice* StorageDevice::Create(const char* class_name, DiskImage* image) {
  StorageDevice* device = dynamic_cast<StorageDevice*>(Device::Create(class_name));
  MV_ASSERT(device);
  device->image_ = image;
  return device;
}

StorageDevice::StorageDevice() {

}

StorageDevice::~StorageDevice() {

}
