#ifndef _MVISOR_IMAGE_H
#define _MVISOR_IMAGE_H

#include "utilities.h"
#include "object.h"
/* Use this macro at the end of .cc source file to declare your image format */
#define DECLARE_DISK_IMAGE(classname) __register_class(classname, 1)

#include <string>

struct ImageInformation {
  /* Disk size is block_size * total_blocks */
  size_t block_size;
  size_t total_blocks;
};

class DiskImage : public Object {
 public:
  DiskImage();
  virtual ~DiskImage();

  /* Always use this static method to create a DiskImage */
  static DiskImage* Open(const std::string format, const std::string path, bool readonly);

  /* Interfaces for a image format to implement */
  virtual ImageInformation information() = 0;
  virtual ssize_t Read(void *buffer, off_t block, size_t block_count) = 0;
  virtual ssize_t Write(void *buffer, off_t block, size_t block_count) = 0;
  virtual void Flush() = 0;

 protected:
  virtual void Initialize(const std::string& path, bool readonly) = 0;
};


#endif // _MVISOR_DISK_IMAGE_H
