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

  void set_sector_size(size_t size) { sector_size_ = size; }
  size_t sector_size() { return sector_size_; }
 protected:
  std::string path_;
  bool readonly_ = true;
  size_t sector_size_ = 512;
  int fd_ = -1;
  size_t bytes_read = 0;
  size_t bytes_writen = 0;
};


#endif // _MVISOR_DISK_IMAGE_H
