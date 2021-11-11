#include "images/image.h"
#include <fcntl.h>
#include <unistd.h>
#include "logger.h"


DiskImage::DiskImage() {
}

bool DiskImage::Open(const std::string path, bool readonly) {
  path_ = path;
  readonly_ = readonly;

  if (readonly) {
    fd_ = open(path.c_str(), O_RDONLY);
  } else {
    fd_ = open(path_.c_str(), O_RDWR);
  }
  if (fd_ < 0)
    return false;

  return true;
}

DiskImage::~DiskImage()
{
  if (fd_ >= 0) {
    close(fd_);
  }
}


ssize_t DiskImage::Write(void *buffer, uint64_t sector, int count) {
  if (readonly_) {
    MV_PANIC("disk is readonly");
  }
  return 0;
}

void DiskImage::Flush() {
  if (readonly_) {
    MV_PANIC("disk is readonly");
  }
}
