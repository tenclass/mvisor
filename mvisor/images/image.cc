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

#include "disk_image.h"
#include "logger.h"
#include "utilities.h"

DiskImage::DiskImage() {
}

DiskImage::~DiskImage()
{
}

DiskImage* DiskImage::Create(std::string path, bool readonly) {
  DiskImage* image;
  if (path.find(".qcow2") != std::string::npos) {
    image = dynamic_cast<DiskImage*>(Object::Create("qcow2-image"));
  } else {
    image = dynamic_cast<DiskImage*>(Object::Create("raw-image"));
  }
  MV_ASSERT(image);
  image->Initialize(path, readonly);
  return image;
}

void DiskImage::Connect() {
  if (!initialized_) {
    initialized_ = true;
    std::string path = std::get<std::string>(key_values_["path"]);
    bool readonly = std::get<bool>(key_values_["readonly"]);
    Initialize(path, readonly);
  }
}

