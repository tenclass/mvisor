#include "images/raw.h"
#include <unistd.h>
#include "logger.h"


RawDiskImage::RawDiskImage(const std::string path, bool readonly)
  : DiskImage(path, readonly) {

}

ssize_t RawDiskImage::Read(void *buffer, uint64_t sector, int count) {
  off_t offset = sector * sector_size_;
  off_t nbytes = count * sector_size_;
  return pread(fd_, buffer, nbytes, offset);
}

ssize_t RawDiskImage::Write(void *buffer, uint64_t sector, int count) {
  off_t offset = sector * sector_size_;
  off_t nbytes = count * sector_size_;
  return pwrite(fd_, buffer, nbytes, offset);
}

void RawDiskImage::Flush() {
  if (fsync(fd_) < 0) {
    MV_PANIC("failed to sync file %s", path_.c_str());
  }
}
