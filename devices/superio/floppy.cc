/* 
 * MVisor Floppy Disk
 * Copyright (C) 2022 Terrence <terrence@tenclass.com>
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


#include "floppy.h"

Floppy::Floppy() {
  set_default_parent_class("I82078Fdc");
}

void Floppy::Connect() {
  Device::Connect();

  /* Connect to backend image */
  bool readonly = has_key("readonly") && std::get<bool>(key_values_["readonly"]);
  bool snapshot = has_key("snapshot") && std::get<bool>(key_values_["snapshot"]);
  if (has_key("image")) {
    auto path = std::get<std::string>(key_values_["image"]);
    image_ = DiskImage::Create(dynamic_cast<Device*>((Object*)this->parent()), path, readonly, snapshot);
  }
}

void Floppy::Disconnect() {
  if (image_) {
    delete image_;
    image_ = nullptr;
  }
  Device::Disconnect();
}

void Floppy::Reset() {
  Device::Reset();

  perpendicular_ = 0;
  cylinder_ = 0;
  head_ = 0;
  sector_ = 1;
  sectors_per_cylinder_ = 18;
}

void Floppy::Seek(uint8_t cylinder, uint8_t head, uint8_t sector) {
  cylinder_ = cylinder;
  head_ = head;
  sector_ = sector;
}

uint Floppy::GetLba() {
  uint lba = (2 * cylinder_ + head_) * sectors_per_cylinder_ + sector_ - 1;
  return lba;
}

void Floppy::SetLba(uint lba) {
  cylinder_ = lba / (2 * sectors_per_cylinder_);
  head_ = (lba % (2 * sectors_per_cylinder_)) / sectors_per_cylinder_;
  sector_ = (lba % (2 * sectors_per_cylinder_)) % sectors_per_cylinder_ + 1;
}

DECLARE_DEVICE(Floppy);
