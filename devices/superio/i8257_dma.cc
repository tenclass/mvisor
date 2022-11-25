/* 
 * MVisor Intel 8257 DMA
 * Copyright (C) 2022 Terrence <terrence@tenclass.com>
 * Reference: https://wiki.osdev.org/ISA_DMA
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

#include "i8257_dma.h"
#include "i8257_dma.pb.h"
#include "logger.h"
#include "device_manager.h"


I8257Dma::I8257Dma() {
  set_default_parent_class("Ich9Lpc", "Piix3");

  AddIoResource(kIoResourceTypePio, 0x00, 0x10, "DMA Controller 1");
  AddIoResource(kIoResourceTypePio, 0xC0, 0x20, "DMA Controller 2");
  AddIoResource(kIoResourceTypePio, 0x80, 0x10, "DMA Page");
}

void I8257Dma::Reset() {
  Device::Reset();
  bzero(&controllers_, sizeof(controllers_));
}

bool I8257Dma::SaveState(MigrationWriter* writer) {
  I8257DmaState state;
  
  for (auto& c: controllers_) {
    auto pc = state.add_controllers();
    pc->set_mask(c.mask);
    pc->set_flip_flop(c.flip_flop);
    for (auto i = 0; i < 4; i++) {
      auto& ch = c.channels[i];
      auto pch = pc->add_channels();
      pch->set_base((ch.base[1] << 8) | ch.base[0]);
      pch->set_count((ch.count[1] << 8) | ch.count[0]);
      pch->set_page(ch.page);
      pch->set_mode(ch.mode);
    }
  }

  writer->WriteProtobuf("DMA", state);
  return Device::SaveState(writer);
}

bool I8257Dma::LoadState(MigrationReader* reader) {
  if (!Device::LoadState(reader)) {
    return false;
  }

  I8257DmaState state;
  if (!reader->ReadProtobuf("DMA", state)) {
    return false;
  }

  for (auto i = 0; i < state.controllers_size() && i < 2; i++) {
    auto& c = controllers_[i];
    auto& d = state.controllers(i);
    c.mask = d.mask();
    c.flip_flop = d.flip_flop();
    for (auto j = 0; j < d.channels_size() && j < 4; j++) {
      auto& ch = c.channels[j];
      auto& dh = d.channels(j);
      ch.base[0] = dh.base() & 0xFF;
      ch.base[1] = (dh.base() >> 8) & 0xFF;
      ch.count[0] = dh.count() & 0xFF;
      ch.count[1] = (dh.count() >> 8) & 0xFF;
      ch.page = dh.page();
      ch.mode = dh.mode();
    }
  }
  return true;
}

uint8_t I8257Dma::ReadDmaControl(uint controller_index, uint64_t offset) {
  auto controller = &controllers_[controller_index];
  uint8_t ret = 0;

  switch (offset)
  {
  default:
    MV_PANIC("read controller=%x off=%x data=%x",
      controller_index, offset, ret);
  }

  MV_UNUSED(controller);
  return ret;
}

void I8257Dma::Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  if (resource->base == 0x00) {
    data[0] = ReadDmaControl(0, offset);
  } else if (resource->base == 0xC0) {
    data[0] = ReadDmaControl(1, offset / 2);
  } else if (resource->base == 0x80) {
    ReadWriteDmaPage(offset, data, false);
  } else {
    Device::Read(resource, offset, data, size);
  }
}

void I8257Dma::WriteDmaControl(uint controller_index, uint64_t offset, uint8_t value) {
  auto controller = &controllers_[controller_index];

  switch (offset)
  {
  case 0x00:
  case 0x02:
  case 0x04:
  case 0x06: {  // Start Address
    uint channel_index = offset / 2;
    controller->channels[channel_index].base[controller->flip_flop++ & 1] = value;
    break;
  }
  case 0x01:
  case 0x03:
  case 0x05:
  case 0x07: {  // Count
    uint channel_index = offset / 2;
    controller->channels[channel_index].count[controller->flip_flop++ & 1] = value;
    break;
  }
  case 0x0A: {  // Single Mask
    if (value & 4) {
      controller->mask |= 1 << (value & 3);
    } else {
      controller->mask &= ~(1 << (value & 3));
    }
    break;
  }
  case 0x0B: {  // Mode
    uint channel_index = value & 3;
    controller->channels[channel_index].mode = value;
    break;
  }
  case 0x0C:  // Clear Flip Flop
    controller->flip_flop = 0;
    break;
  case 0x0D:  // Reset
    bzero(controller, sizeof(*controller));
    break;
  default:
    MV_PANIC("write controller=%x off=%x data=%x",
      controller_index, offset, value);
  }
}

void I8257Dma::ReadWriteDmaPage(uint64_t offset, uint8_t* data, bool is_write) {
  I8257DmaChannel* channel = nullptr;
  switch (offset)
  {
  case 0x3:
    channel = &controllers_[0].channels[1];
    break;
  case 0x1:
    channel = &controllers_[0].channels[2];
    break;
  case 0x2:
    channel = &controllers_[0].channels[3];
    break;
  case 0xB:
    channel = &controllers_[1].channels[1];
    break;
  case 0x9:
    channel = &controllers_[1].channels[2];
    break;
  case 0xA:
    channel = &controllers_[1].channels[3];
    break;
  default:
    if (debug_) {
      MV_WARN("invalid RW page offset=0x%lx", offset);
    }
  }

  if (channel) {
    if (is_write) {
      channel->page = data[0];
    } else {
      data[0] = channel->page;
    }
  }
}

void I8257Dma::Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  if (resource->base == 0x00) {
    WriteDmaControl(0, offset, data[0]);
  } else if (resource->base == 0xC0) {
    WriteDmaControl(1, offset / 2, data[0]);
  } else if (resource->base == 0x80) {
    ReadWriteDmaPage(offset, data, true);
  } else {
    Device::Write(resource, offset, data, size);
  }
}

void I8257Dma::WaitForChannel(bool is_write, uint channel_id, BufferCallback callback) {
  MV_ASSERT(channel_id < 8);

  uint controller_index = channel_id / 4;
  uint channel_index = channel_id % 5;
  auto channel = &controllers_[controller_index].channels[channel_index];
  
  uint64_t gpa = (channel->page << 16) | (channel->base[1] << 8) | (channel->base[0]);
  uint64_t count = (channel->count[1] << 8) | (channel->count[0]);
  if (debug_) {
    MV_LOG("dma address gpa=%lx count=%lx", gpa, count);
  }
  
  auto iov = iovec {
    .iov_base = manager_->TranslateGuestMemory(gpa),
    .iov_len = count + 1
  };

  if (is_write) {
    manager_->AddDirtyMemory(gpa, iov.iov_len);   
  }
  callback(iov);
}

DECLARE_DEVICE(I8257Dma);
