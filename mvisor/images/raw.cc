#include "images/raw.h"
#include <unistd.h>
#include <sys/stat.h>
#include "logger.h"


RawDiskImage::RawDiskImage(const std::string path, bool readonly) {
  MV_ASSERT(Open(path, readonly));

  struct stat st;
  fstat(fd_, &st);
  disk_size_ = st.st_size;

  /* cdrom may changed this or not ? */
  set_sector_size(512);
}

ssize_t RawDiskImage::Read(void *buffer, uint64_t sector, int count) {
  off_t offset = sector * sector_size_;
  off_t nbytes = count * sector_size_;
  bytes_read_ += nbytes;
  MV_LOG("read at sector=0x%x(0x%x) count=0x%x total_read=%ld MB",
    sector, offset, count, bytes_read_ >> 20);
  return pread(fd_, buffer, nbytes, offset);
}

ssize_t RawDiskImage::Write(void *buffer, uint64_t sector, int count) {
  off_t offset = sector * sector_size_;
  off_t nbytes = count * sector_size_;
  bytes_writen_ += nbytes;
  MV_LOG("read at sector=0x%x(0x%x) count=0x%x total_written=%ld MB",
    sector, offset, count, bytes_writen_ >> 20);
  return pwrite(fd_, buffer, nbytes, offset);
}

void RawDiskImage::Flush() {
  if (fsync(fd_) < 0) {
    MV_PANIC("failed to sync file %s", path_.c_str());
  }
}
