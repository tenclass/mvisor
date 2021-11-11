#ifndef _MVISOR_IMAGE_H
#define _MVISOR_IMAGE_H

#include <string>

class DiskImage {
 public:
  DiskImage();
  virtual ~DiskImage();
  virtual bool Open(const std::string path, bool readonly);
  virtual ssize_t Read(void *buffer, uint64_t sector, int count) = 0;
  virtual ssize_t Write(void *buffer, uint64_t sector, int count);
  virtual void Flush();

  size_t disk_size() { return disk_size_; }
  size_t sector_size() { return sector_size_; }
  size_t sectors() { return sectors_; }
  void set_sector_size(size_t size) {
    sector_size_ = size;
    sectors_ = disk_size_ / sector_size_;
  }
 protected:
  std::string path_;
  bool readonly_ = true;
  size_t sector_size_ = 512;
  size_t sectors_ = 0;
  int fd_ = -1;
  size_t disk_size_ = 0;
  size_t bytes_read_ = 0;
  size_t bytes_writen_ = 0;
};


#endif // _MVISOR_DISK_IMAGE_H
