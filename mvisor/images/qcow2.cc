#include "disk_image.h"
#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include "logger.h"

class Qcow2 : public DiskImage {
 private:
  int fd_ = -1;
  bool readonly_ = true;
  size_t block_size_ = 512;
  size_t total_blocks_ = 0;
  size_t image_size_ = 0;

  ImageInformation information() {
    return ImageInformation {
      .block_size = block_size_,
      .total_blocks = total_blocks_
    };
  }

  void Initialize(const std::string& path, bool readonly) {
    readonly_ = readonly;

    if (readonly) {
      fd_ = open(path.c_str(), O_RDONLY);
    } else {
      fd_ = open(path.c_str(), O_RDWR);
    }
    if (fd_ < 0)
      MV_PANIC("disk file not found: %s", path.c_str());

    struct stat st;
    fstat(fd_, &st);
    image_size_ = st.st_size;
    ReadQcowHeader();
  }

  void ReadQcowHeader() {

  }


  ssize_t Read(void *buffer, off64_t block, size_t block_count) {
    MV_PANIC("not implemented");
    return 0;
  }

  ssize_t Write(void *buffer, off64_t block, size_t block_count) {
    MV_PANIC("not implemented");
    return 0;
  }

  void Flush() {
    int ret = fsync(fd_);
    if (ret < 0) {
      MV_PANIC("failed to sync disk image, ret=%d", ret);
    }
  }

};

DECLARE_DISK_IMAGE(Qcow2);
