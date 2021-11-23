/* 
 * MVisor
 * Copyright (C) 2021 Terrence <terrence@tenclass.com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "storage_device.h"
#include "disk_image.h"
#include "logger.h"

StorageDevice* StorageDevice::Create(const char* class_name, DiskImage* image) {
  StorageDevice* device = dynamic_cast<StorageDevice*>(Device::Create(class_name));
  MV_ASSERT(device);
  device->image_ = image;
  return device;
}

StorageDevice::StorageDevice() {

}

StorageDevice::~StorageDevice() {
  /* Maybe we should use shared_ptr here ? */
  if (image_) {
    delete image_;
    image_ = nullptr;
  }
}
