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

#ifndef _MVISOR_STORAGE_DEVICE_H
#define _MVISOR_STORAGE_DEVICE_H

#include "device.h"

class DiskImage;
class StorageDevice : public Device {
 public:
  StorageDevice();
  virtual ~StorageDevice();

  static StorageDevice* Create(const char* class_name, DiskImage* image);

 protected:
  DiskImage* image_ = nullptr;
};


#endif // _MVISOR_STORAGE_DEVICE_H
