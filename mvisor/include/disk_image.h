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

#ifndef _MVISOR_IMAGE_H
#define _MVISOR_IMAGE_H

#include "utilities.h"
#include "object.h"
/* Use this macro at the end of .cc source file to declare your image format */
#define DECLARE_DISK_IMAGE(classname) __register_class(classname, 1)

#include <string>

struct ImageInformation {
  /* Disk size is block_size * total_blocks */
  size_t block_size;
  size_t total_blocks;
};

class DiskImage : public Object {
 public:
  DiskImage();
  virtual ~DiskImage();
  virtual void Connect();
  bool readonly() { return readonly_; }

  /* Always use this static method to create a DiskImage */

  /* Interfaces for a image format to implement */
  virtual ImageInformation information() = 0;
  virtual ssize_t Read(void *buffer, off_t position, size_t length) = 0;
  virtual ssize_t Write(void *buffer, off_t position, size_t length) = 0;
  virtual void Flush() = 0;
  virtual void Trim(off_t position, size_t length) = 0;

 protected:
  bool initialized_ = false;
  bool readonly_ = false;
  virtual void Initialize(const std::string& path, bool readonly) = 0;
};


#endif // _MVISOR_DISK_IMAGE_H
