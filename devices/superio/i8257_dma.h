/* 
 * MVisor Intel 8257 DMA
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

#ifndef _MVISOR_DEVICES_I8257_DMA_H
#define _MVISOR_DEVICES_I8257_DMA_H

#include <functional>
#include "device.h"

struct I8257DmaChannel {
  uint8_t mode;
  uint8_t base[2];
  uint8_t count[2];
  uint8_t page;
};

struct I8257DmaController {
  uint8_t         mask;
  uint8_t         flip_flop;
  I8257DmaChannel channels[4];
};

typedef std::function<void(iovec iov)> BufferCallback;

class I8257Dma : public Device {
 private:
  I8257DmaController  controllers_[2];

  uint8_t ReadDmaControl(uint controller_index, uint64_t offset);
  void WriteDmaControl(uint controller_index, uint64_t offset, uint8_t value);
  void ReadWriteDmaPage(uint64_t offset, uint8_t* data, bool is_write);

 public:
  I8257Dma();
  void Reset();
  bool SaveState(MigrationWriter* writer);
  bool LoadState(MigrationReader* reader);
  void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);
  void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);

  void WaitForChannel(bool is_write, uint channel_id, BufferCallback callback);
};

#endif // _MVISOR_DEVICES_I8257_DMA_H
