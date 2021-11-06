#ifndef _MVISOR_IMAGES_RAW_H
#define _MVISOR_IMAGES_RAW_H

#include "images/image.h"

class RawDiskImage : public DiskImage {
 public:
  RawDiskImage(const std::string path, bool readonly = false);
  virtual ssize_t Read(void *buffer, uint64_t sector, int count);
  virtual ssize_t Write(void *buffer, uint64_t sector, int count);
  virtual void Flush();
};

#endif // _MVISOR_IMAGES_RAW_H
