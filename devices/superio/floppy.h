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

#ifndef _MVISOR_DEVICES_FLOPPY_DISK_H
#define _MVISOR_DEVICES_FLOPPY_DISK_H

#include "disk_image.h"
#include "device.h"

class Floppy : public Device {
 private:
  DiskImage*  image_ = nullptr;
  uint8_t     perpendicular_;
  uint8_t     cylinder_;
  uint8_t     head_;
  uint8_t     sector_;
  uint8_t     sectors_per_cylinder_;

 public:
  Floppy();
  void Connect();
  void Disconnect();
  void Reset();

  void Seek(uint8_t cylinder, uint8_t head, uint8_t sector);
  uint GetLba();
  void SetLba(uint lba);

  void set_perpendicular(uint8_t perpendicular) { perpendicular_ = perpendicular; }
  void set_head(uint8_t head) { head_ = head; }

  inline DiskImage*   image() { return image_; }
  inline uint8_t      sectors_per_cylinder() { return sectors_per_cylinder_; }
  inline uint8_t      perpendicular() { return perpendicular_; }
  inline uint8_t      cylinder() { return cylinder_; }
  inline uint8_t      head() { return head_; }
  inline uint8_t      sector() { return sector_; }
};

#endif // _MVISOR_DEVICES_FLOPPY_DISK_H

