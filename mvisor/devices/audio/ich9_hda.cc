/* 
 * MVisor ICH9 High Definition Audio Device
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

#include "ich9_hda.h"

#include <cstring>
#include <vector>
#include <chrono>

#include "logger.h"
#include "pci_device.h"
#include "device_manager.h"
#include "hda_codec.h"
#include "hda_internal.h"
#include "states/ich9_hda.pb.h"

using namespace std::chrono;


class Ich9Hda : public PciDevice {
 private:
  Ich9HdaRegisters  regs_;
  uint32_t          rirb_counter_;
  IoTimePoint       wall_clock_base_;

  std::vector<HdaCodecInterface*> codecs_;

  struct Ich9HdaStreamState {
    std::vector<HdaCodecBuffer> buffers;
    uint32_t buffers_index;
  } stream_states_[8];

 public:
  Ich9Hda() {
    devfn_ = PCI_MAKE_DEVFN(4, 0);
    
    pci_header_.vendor_id = 0x8086;
    pci_header_.device_id = 0x293E;
    pci_header_.class_code = 0x040300;
    pci_header_.revision_id = 3;
    pci_header_.header_type = PCI_HEADER_TYPE_NORMAL;
    pci_header_.subsys_vendor_id = 0x1AF4;
    pci_header_.subsys_id = 0x1100;
    pci_header_.irq_pin = 0;
    pci_header_.command = PCI_COMMAND_MEMORY;

    /* HDCTL off 0x40 bit 0 selects signaling mode (1-HDA, 0 - Ac97) 18.1.19 */
    pci_header_.data[0x40] = 0x01;

    /* Add MSI at 0x60 */
    next_capability_offset_ = 0x60;
    AddMsiCapability();
  
    /* Memory BAR */
    AddPciBar(0, 0x4000, kIoResourceTypeMmio);
  }

  virtual void Connect() {
    PciDevice::Connect();
    for (auto device : children_) {
      auto codec = dynamic_cast<HdaCodecInterface*>(device);
      if (codec) {
        codecs_.push_back(codec);
      }
    }
  }

  virtual void Disconnect() {
    codecs_.clear();
    PciDevice::Disconnect();
  }

  virtual void Reset() {
    PciDevice::Reset();

    MV_ASSERT(sizeof(regs_) == 0x180);
    rirb_counter_ = 0;
    wall_clock_base_ = steady_clock::now();
    bzero(&regs_, sizeof(regs_));
    regs_.global_capabilities = 0x4401; // 4 input, 4 output, 64bit supported
    regs_.major_version = 1;
    regs_.minor_version = 0;
    regs_.output_payload_capacity = 0x3C;
    regs_.input_payload_capacity = 0x1D;
    regs_.corb_size = 0x42; // 1KB, 256 entries
    regs_.rirb_size = 0x42;
    for (auto &stream : regs_.streams) {
      stream.fifo_size = 0x100;
    }
    // Setup available codecs
    for (size_t i = 0; i < codecs_.size(); i++) {
      regs_.state_change_status |= (1 << i);
    }
    CheckIrqLevel();
  }

  uint64_t GetWallClockCounter() {
    auto delta_ns = duration_cast<nanoseconds>(steady_clock::now() - wall_clock_base_);
    return delta_ns.count();
  }

  virtual bool SaveState(MigrationWriter* writer) {
    Ich9HdaState state;
    state.set_hda_registers(&regs_, sizeof(regs_));
    state.set_rirb_counter(rirb_counter_);
    state.set_wall_clock_counter(GetWallClockCounter());
    writer->WriteProtobuf("HDA", state);
    return PciDevice::SaveState(writer);
  }

  virtual bool LoadState(MigrationReader* reader) {
    if (!PciDevice::LoadState(reader)) {
      return false;
    }
    Ich9HdaState state;
    if (!reader->ReadProtobuf("HDA", state)) {
      return false;
    }
    auto& regs = state.hda_registers();
    memcpy(&regs_, regs.data(), sizeof(regs_));
    rirb_counter_ = state.rirb_counter();
    wall_clock_base_ = steady_clock::now() - nanoseconds(state.wall_clock_counter());

    /* When finished loading states, restart stream if started before */
    manager_->io()->Schedule([this]() {
      for (uint i = 0; i < 8; i++) {
        StartStopStream(i);
      }
    });
    return true;
  }

  void UpdateInterruptStatus() {
    uint32_t status = 0;
  
    /* update controller status */
    if ((regs_.rirb_status & ICH6_RBSTS_IRQ) || (regs_.rirb_status & ICH6_RBSTS_OVERRUN)) {
      // RIRB interrupt or overrun
      status |= 1 << 30;
    }
    if (regs_.state_change_status & regs_.wake_enable) {
      status |= 1 << 30;
    }
    /* update stream status */
    for (int i = 0; i < 8; i++) {
      if (regs_.streams[i].status & (1 << 2)) { // buffer completion interrupt
        status |= 1 << i;
      }
    }
    /* update global status */
    if (status & regs_.interrupt_control) {
      status |= 1 << 31;
    }
    regs_.interrupt_status = status;
  }

  void CheckIrqLevel() {
    UpdateInterruptStatus();
  
    uint32_t level = (regs_.interrupt_status & (1 << 31)) && (regs_.interrupt_control & (1 << 31));
    if (msi_config_.enabled) {
      if (level) {
        SignalMsi();
      }
    } else {
      manager_->SetIrq(pci_header_.irq_line, level);
    }
  }

  void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    MV_ASSERT(resource->base == pci_bars_[0].address);
    
    // Make memory 0x2000-0x4000 and 0x0000-0x2000 same data
    if (offset >= 0x2000) {
      offset -= 0x2000;
    }
    MV_ASSERT(offset + size <= sizeof(regs_));

    if (offset == offsetof(Ich9HdaRegisters, wall_clock_counter)) {
      regs_.wall_clock_counter = (uint32_t)GetWallClockCounter();
    }
    memcpy(data, (uint8_t*)&regs_ + offset, size);
    // MV_LOG("read %s at 0x%lx size=%x ret=0x%x", name_, offset, size, *(uint32_t*)data);
  }

  void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    MV_ASSERT(resource->base == pci_bars_[0].address);
    MV_ASSERT(offset >= 0 && (offset + size) <= sizeof(regs_));
    // MV_LOG("write %s at 0x%lx size=%x ret=0x%x", name_, offset, size, *(uint32_t*)data);

    if (offset >= 0x80 && offset < 0x180) {
      uint64_t stream_desc_index = (offset - 0x80) / 0x20;
      uint64_t stream_desc_offset = (offset - 0x80) % 0x20;
      // byte write
      for (uint i = 0; i < size; i++) {
        WriteStreamDescriptor(stream_desc_index, stream_desc_offset + i, &data[i], 1);
      }
      return;
    }

    switch (offset)
    {
    case offsetof(Ich9HdaRegisters, global_control):
      regs_.global_control = (*(uint32_t*)data) & 0x103;
      if ((regs_.global_control & ICH6_GCTL_RESET) == 0) {
        Reset();
      }
      break;
    case offsetof(Ich9HdaRegisters, state_change_status): {
      uint16_t value = *(uint16_t*)data;
      regs_.state_change_status &= ~(value & 0x7FFF);
      CheckIrqLevel();
      break;
    }
    case offsetof(Ich9HdaRegisters, interrupt_status): {
      uint32_t value = *(uint32_t*)data;
      regs_.interrupt_status &= ~(value & 0xC00000FF);
      break;
    }
    case offsetof(Ich9HdaRegisters, corb_status):
      regs_.corb_status &= ~(data[0] & 1);
      break;
    case offsetof(Ich9HdaRegisters, rirb_status): {
      uint8_t old = regs_.rirb_status;
      regs_.rirb_status &= ~(data[0] & 5);
      CheckIrqLevel();
      if ((old & ICH6_RBSTS_IRQ) && !(regs_.rirb_status & ICH6_RBSTS_IRQ)) {
        // cleared ICH6_RBSTS_IRQ
        rirb_counter_ = 0;
        PopCorbEntries();
      }
      break;
    }
    case offsetof(Ich9HdaRegisters, interrupt_control):
    case offsetof(Ich9HdaRegisters, wake_enable):
      memcpy((uint8_t*)&regs_ + offset, data, size);
      CheckIrqLevel();
      break;
    case offsetof(Ich9HdaRegisters, corb_write_pointer):
    case offsetof(Ich9HdaRegisters, corb_control):
      memcpy((uint8_t*)&regs_ + offset, data, size);
      PopCorbEntries();
      break;
    case offsetof(Ich9HdaRegisters, rirb_write_pointer):
      memcpy((uint8_t*)&regs_ + offset, data, size);
      if (regs_.rirb_write_pointer & ICH6_RIRBWP_RST) { // reset bit
        regs_.rirb_write_pointer = 0;
      }
      break;
    default:
      memcpy((uint8_t*)&regs_ + offset, data, size);
      break;
    }
  }

  void ParseBufferDescriptorList(uint64_t index) {
    auto &stream = regs_.streams[index];
    auto &stream_state = stream_states_[index];
    stream_state.buffers.clear();
    uint64_t addr = ((uint64_t)stream.bdl_base1 << 32) + stream.bdl_base0;
    MV_ASSERT((addr & 127) == 0); // aligned by 128 bytes

    Ich9HdaBdlEntry* entries = (Ich9HdaBdlEntry*)manager_->TranslateGuestMemory(addr);
    for (int i = 0; i <= stream.last_valid_index; i++) {
      stream_state.buffers.emplace_back(HdaCodecBuffer {
        .data = (uint8_t*)manager_->TranslateGuestMemory(entries[i].address),
        .length = entries[i].length,
        .interrupt_on_completion = (entries[i].flags & 1) == 1,
        .read_counter =0
      });
    }
    stream_state.buffers_index = 0;
    stream.link_position_in_buffer = 0;
  }

  size_t TransferStreamData(uint64_t index, uint8_t* destination, size_t length) {
    auto &stream = regs_.streams[index];
    auto &stream_state = stream_states_[index];
    MV_ASSERT(stream_state.buffers_index <= stream.last_valid_index);

    auto &entry = stream_state.buffers[stream_state.buffers_index];
    if (length > entry.length - entry.read_counter) {
      length = entry.length - entry.read_counter;
    }

    /* IN or OUT */
    if (index >= 4) {
      memcpy(destination, entry.data + entry.read_counter, length);
    } else {
      memcpy(entry.data + entry.read_counter, destination, length);
    }
    stream.link_position_in_buffer += length;
    entry.read_counter += length;

    /* Check if we should read next buffer */
    if (entry.read_counter == entry.length) {
      entry.read_counter = 0;
      stream_state.buffers_index++;

      if (stream.link_position_in_buffer >= stream.cyclic_buffer_length) {
        stream.link_position_in_buffer = 0;
        stream_state.buffers_index = 0;
      }
      if (entry.interrupt_on_completion) {
        stream.status |= 1 << 2;
        CheckIrqLevel();
      }
    }
    if (debug_) {
      MV_LOG("stream[%d] nr=%d dma transferred %d bytes", index, stream.stream_id, length);
    }
    return length;
  }

  void StartStopStream(uint64_t index) {
    auto &stream = regs_.streams[index];
    if (stream.control & 0x02) { // start
      if (debug_) {
        MV_LOG("stream[%d] nr=%d start, ring buf %d bytes", index, stream.stream_id, stream.cyclic_buffer_length);
      }
      ParseBufferDescriptorList(index);

      for (auto codec : codecs_) {
        codec->StartStream(stream.stream_id, index >= 4,
          [index, this](uint8_t* destination, size_t length) -> size_t {
          return TransferStreamData(index, destination, length);
        });
      }
    } else { // stop
      if (debug_) {
        MV_LOG("stream[%d] nr=%d stop", index, stream.stream_id);
      }
      for (auto codec : codecs_) {
        codec->StopStream(stream.stream_id, index >= 4);
      }
    }
  }

  void WriteStreamControl(uint64_t index, uint8_t control) {
    auto &stream = regs_.streams[index];
    uint8_t old_control = stream.control;
    stream.control = control;
    if (stream.control & 0x01) { // reset
      if (debug_) {
        MV_LOG("streams[%d] reset", index);
      }
      stream.status = 0x20; // FIFO ready
    }

    if ((stream.control & 0x02) != (old_control & 0x02)) {
      StartStopStream(index);
    }
    CheckIrqLevel();
  }

  void WriteStreamDescriptor(uint64_t index, uint64_t offset, uint8_t* data, uint32_t size) {
    auto &stream = regs_.streams[index];
    switch (offset)
    {
    case offsetof(Ich9HdaStream, control):
      MV_ASSERT(size == 1);
      WriteStreamControl(index, data[0]);
      break;
    case offsetof(Ich9HdaStream, status):
      stream.status &= ~(data[0] & 0x1C);
      CheckIrqLevel();
      break;
    default:
      memcpy((uint8_t*)&stream + offset, data, size);
      break;
    }
  }

  void PopCorbEntries() {
    while (true) {
      if (!(regs_.corb_control & ICH6_CORBCTL_RUN)) {
        return;
      }
      if ((regs_.corb_read_pointer & 0xFF) == regs_.corb_write_pointer) {
        return;
      }
      if (rirb_counter_ == regs_.rirb_interrupt_count) {
        MV_PANIC("rirb count reached");
        return;
      }

      uint32_t read_pointer = (regs_.corb_read_pointer + 1) & 0xFF;
      uint64_t addr = ((uint64_t)regs_.corb_base1 << 32) + regs_.corb_base0;
      uint32_t* ptr = (uint32_t*)manager_->TranslateGuestMemory(addr + 4 * read_pointer);
      regs_.corb_read_pointer = read_pointer;
      ParseCorbEntry(*ptr);
    }
  }

  void ParseCorbEntry(uint32_t entry) {
    uint8_t codec_index = (entry >> 28) & 0x0F;
    uint8_t node_id = (entry >> 20) & 0x7F;
    uint32_t data = entry & 0xFFFFF;
    MV_ASSERT(codec_index < codecs_.size());
    auto codec = codecs_[codec_index];
    codec->StartCommand(node_id, data, [=](uint32_t response) {
      PushRirbEntry(codec_index, response);
    });
  }

  void PushRirbEntry(uint8_t codec_index, uint32_t response) {
    if (!(regs_.rirb_control & (1 << 1))) {
      MV_PANIC("RIRB DMA is disabled");
      return;
    }
    bool solicited = true;
    uint32_t extended = (solicited ? 0 : (1 << 4)) | codec_index;
    uint32_t write_pointer = (regs_.rirb_write_pointer + 1) & 0xFF;
    uint64_t addr = ((uint64_t)regs_.rirb_base1 << 32) + regs_.rirb_base0;
    uint32_t* ptr = (uint32_t*)manager_->TranslateGuestMemory(addr + 8 * write_pointer);
    ptr[0] = response;
    ptr[1] = extended;
    regs_.rirb_write_pointer = write_pointer;
    rirb_counter_++;

    if (debug_) {
      MV_LOG("codec_index=%d response=0x%x", codec_index, response);
    }

    if (regs_.rirb_control & ICH6_RBCTL_IRQ_EN) {
      // only raise interrupt if CORB is empty or RIRB is full
      if ((regs_.corb_read_pointer & 0xFF) == regs_.corb_write_pointer ||
        rirb_counter_ == regs_.rirb_interrupt_count) {
        regs_.rirb_status |= ICH6_RBSTS_IRQ;
        CheckIrqLevel();
      }
    }
  }
};

DECLARE_DEVICE(Ich9Hda);
