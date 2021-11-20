#include "disk_image.h"
#include "logger.h"
#include "utilities.h"

DiskImage* DiskImage::Open(const std::string format, const std::string path, bool readonly) {
  DiskImage* image = dynamic_cast<DiskImage*>(realize_class(format.c_str()));
  if (image == nullptr) {
    MV_PANIC("image format %s is not supported", format.c_str());
  }

  image->Initialize(path, readonly);
  return image;
}

DiskImage::DiskImage() {
}

DiskImage::~DiskImage()
{
}
