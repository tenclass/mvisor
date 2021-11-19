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
  size_t total_sectors() { return total_sectors_; }
  void set_sector_size(size_t size) {
    sector_size_ = size;
    total_sectors_ = disk_size_ / sector_size_;
  }

 protected:
  std::string path_;
  bool readonly_ = true;
  int fd_ = -1;
  size_t sector_size_ = 512;
  size_t total_sectors_ = 0;
  size_t disk_size_ = 0;
};

class RawDiskImage : public DiskImage {
 public:
  RawDiskImage(const std::string path, bool readonly = false);
  virtual ssize_t Read(void *buffer, uint64_t sector, int count);
  virtual ssize_t Write(void *buffer, uint64_t sector, int count);
  virtual void Flush();
};


#endif // _MVISOR_DISK_IMAGE_H
