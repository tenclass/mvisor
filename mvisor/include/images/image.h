#ifndef _MVISOR_IMAGE_H
#define _MVISOR_IMAGE_H

#include <string>

class DiskImage {
 public:
  DiskImage(const std::string path, bool readonly);
  virtual ~DiskImage();

  virtual ssize_t Read(void *buffer, uint64_t sector, int count) = 0;
  virtual ssize_t Write(void *buffer, uint64_t sector, int count);
  virtual void Flush();

 protected:
  std::string path_;
  bool readonly_;
  int fd_;
};


#endif // _MVISOR_DISK_IMAGE_H
