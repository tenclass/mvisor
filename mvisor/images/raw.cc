#include "images/raw.h"
#include <unistd.h>
#include "logger.h"


RawDiskImage::RawDiskImage(const std::string path, bool readonly)
  : DiskImage(path, readonly) {

}

ssize_t RawDiskImage::Read(void *buffer, uint64_t sector, int count) {
  off_t offset = sector * sector_size_;
  off_t nbytes = count * sector_size_;
  bytes_read += nbytes;
  MV_LOG("read at sector=0x%x(0x%x) count=0x%x total_read=%ldMB",
    sector, offset, count, bytes_read >> 20);
  return pread(fd_, buffer, nbytes, offset);
}

ssize_t RawDiskImage::Write(void *buffer, uint64_t sector, int count) {
  off_t offset = sector * sector_size_;
  off_t nbytes = count * sector_size_;
  bytes_writen += nbytes;
  MV_LOG("read at sector=0x%x(0x%x) count=0x%x total_written=%ldMB",
    sector, offset, count, bytes_writen >> 20);
  return pwrite(fd_, buffer, nbytes, offset);
}

void RawDiskImage::Flush() {
  if (fsync(fd_) < 0) {
    MV_PANIC("failed to sync file %s", path_.c_str());
  }
}
