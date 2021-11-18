#ifndef _MVISOR_STORAGE_DEVICE_H
#define _MVISOR_STORAGE_DEVICE_H

#include "device.h"

class DiskImage;
class StorageDevice : public Device {
 public:
  StorageDevice();
  virtual ~StorageDevice();

  static StorageDevice* Create(const char* class_name, DiskImage* image);

 protected:
  DiskImage* image_;
};


#endif // _MVISOR_STORAGE_DEVICE_H
