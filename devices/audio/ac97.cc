/* 
 * MVisor AC97 Sound Card
 * Reference: https://wiki.osdev.org/AC97
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

#include "pci_device.h"
#include "device_interface.h"
#include "device_manager.h"
#include "ac97.pb.h"

/* Transfer status of bus master */
#define TS_HALTED               (1 << 0)
#define TS_END_OF_TRANSFER      (1 << 1)
#define TS_END_INTERRUPT        (1 << 2)
#define TS_COMPLETION_INTERRUPT (1 << 3)
#define TS_ERROR_INTERRUPT      (1 << 4)


struct Ac97Mixer {
  uint16_t  reset;
  uint16_t  master_volume_mute;
  uint16_t  headphone_volume_mute;
  uint16_t  master_volume_mono_mute;
  uint16_t  master_tone_rl;
  uint16_t  pc_beep_volume_mute;
  uint16_t  phone_volume_mute;
  uint16_t  mic_volume_mute;
  uint16_t  line_in_volume_mute;
  uint16_t  cd_volume_mute;
  uint16_t  video_volume_mute;
  uint16_t  aux_volume_mute;
  uint16_t  pcm_out_volume_mute;
  uint16_t  record_select;
  uint16_t  record_gain_mute;
  uint16_t  record_gain_mic_mute;
  uint16_t  general_purpose;
  uint16_t  control_3d;
  uint16_t  reserved;
  uint16_t  powerdown_ctrl_stat;
  uint16_t  extended_audio_id;
  uint16_t  extended_audio_ctrl_stat;
  uint16_t  pcm_front_dac_rate;
  uint16_t  pcm_surround_dac_rate;
  uint16_t  pcm_lfe_dac_rate;
  uint16_t  pcm_lr_adc_rate;
  uint16_t  mic_adc_rate;
  uint16_t  vendor_reserved[0x23];
  uint16_t  vendor_id1;
  uint16_t  vendor_id2;
};

struct Ac97BusMaster {
  uint32_t  bdl_address;
  uint8_t   bdl_current_index;
  uint8_t   bdl_last_valid_index;
  uint16_t  status;
  uint16_t  samples_left;
  uint8_t   bdl_prefetched_index;
  uint8_t   control;
} __attribute__((packed));

struct BufferDescriptor {
  uint32_t  data_address;
  uint16_t  samples;
  uint16_t  flags;
};


#define STREAM_FRAME_INTERVAL_MS    10

struct StreamState {
  uint              index;
  bool              output = false;
  bool              running = false;

  IoTimer*          timer = nullptr;
  IoTimePoint       start_time;
  size_t            position;
  size_t            bytes_per_second;
  size_t            bytes_per_frame;
  std::string       buffer;
  size_t            buffer_pointer = 0;
  BufferDescriptor* bdl_entries = nullptr;
  BufferDescriptor* bdl_current = nullptr;
  uint8_t*          bdl_current_data = nullptr;
};


class Ac97 : public PciDevice, public PlaybackInterface {
 private:
  Ac97Mixer                     mixer_;
  std::array<Ac97BusMaster, 3>  bus_masters_;
  std::array<StreamState, 3>    streams_;
  uint32_t                      global_control_;
  uint32_t                      global_status_;
  uint8_t                       codec_accessing_;
  std::list<PlaybackListener>   playback_listeners_;

 public:
  Ac97() {
    pci_header_.vendor_id = 0x8086;
    pci_header_.device_id = 0x2415;
    pci_header_.class_code = 0x040100;
    pci_header_.revision_id = 1;
    pci_header_.header_type = PCI_HEADER_TYPE_NORMAL;
    pci_header_.subsys_vendor_id = 0x1AF4;
    pci_header_.subsys_id = 0x1100;
    pci_header_.irq_pin = 1;
    pci_header_.status |= PCI_STATUS_FAST_BACK;

    SetupPciBar(0, 1024, kIoResourceTypePio); // Native Audio Mixer
    SetupPciBar(1, 256, kIoResourceTypePio);  // Native Audio Bus Master

    MV_ASSERT(sizeof(Ac97Mixer) == 0x80);

    for (size_t i = 0; i < streams_.size(); i++) {
      auto stream = &streams_[i];
      stream->index = i;
      stream->output = i == 1;
      stream->bytes_per_second = 2LL * 2 * 48000;
      stream->bytes_per_frame = stream->bytes_per_second / (1000 / STREAM_FRAME_INTERVAL_MS);
      stream->buffer.resize(stream->bytes_per_frame);
      bzero(stream->buffer.data(), stream->buffer.size());
    }
  }

  void Disconnect() {
    Reset();
    PciDevice::Disconnect();
  }

  bool SaveState(MigrationWriter* writer) {
    Ac97State state;
    state.set_mixer(&mixer_, sizeof(mixer_));
    for (size_t i = 0; i < bus_masters_.size(); i++) {
      auto &bm = bus_masters_[i];
      auto p = state.add_bus_masters();
      p->set_bdl_address(bm.bdl_address);
      p->set_bdl_current_index(bm.bdl_current_index);
      p->set_bdl_last_valid_index(bm.bdl_last_valid_index);
      p->set_bdl_prefetched_index(bm.bdl_prefetched_index);
      p->set_status(bm.status);
      p->set_control(bm.control);
    }

    writer->WriteProtobuf("AC97", state);
    return PciDevice::SaveState(writer);
  }

  bool LoadState(MigrationReader* reader) {
    if (!PciDevice::LoadState(reader)) {
      return false;
    }

    Ac97State state;
    if (!reader->ReadProtobuf("AC97", state)) {
      return false;
    }
    memcpy(&mixer_, state.mixer().data(), sizeof(mixer_));

    for (int i = 0; i < state.bus_masters_size(); i++) {
      auto &bm = bus_masters_[i];
      auto p = state.bus_masters(i);
      bm.bdl_address = p.bdl_address();
      bm.bdl_current_index = p.bdl_current_index();
      bm.bdl_last_valid_index = p.bdl_last_valid_index();
      bm.bdl_prefetched_index = p.bdl_prefetched_index();
      bm.status = p.status();
      bm.control = p.control();

      if (bm.control & 1) {
        Schedule([this, i]() {
          SetStreamRunning(i, true);
        });
      }
    }

    return true;
  }

  void Reset() {
    PciDevice::Reset();

    for (size_t i = 0; i < streams_.size(); i++) {
      if (streams_[i].running) {
        SetStreamRunning(i, false);
      }
    }

    bzero(&bus_masters_, sizeof(bus_masters_));

    global_status_ = (1 << 8);  // codec 0 is ready
    global_control_ = 0;
    codec_accessing_ = 0;

    ResetMixer();
  }

  void ResetMixer() {
    bzero(&mixer_, sizeof(mixer_));
    mixer_.vendor_id1 = 0x8384;
    mixer_.vendor_id2 = 0x7600;
    mixer_.powerdown_ctrl_stat = 0x000F; // vRef Analog DAC ADC ready

    /* AC97 2.3 compliant (https://hands.com/~lkcl/ac97_r23.pdf)
     * variable sample rate is not supported yet */
    mixer_.extended_audio_id = 0x0800;
    mixer_.extended_audio_ctrl_stat = 0x0000;

    mixer_.pcm_front_dac_rate = 48000;
    mixer_.pcm_lr_adc_rate = 48000;
    mixer_.mic_adc_rate = 48000;

    mixer_.record_select = 0;
    mixer_.master_volume_mute = 0x8000;
    mixer_.pcm_out_volume_mute = 0x8808;
    mixer_.record_gain_mute = 0x8808;
    mixer_.line_in_volume_mute = 0x8808;
  }

  void NotifyPlayback(PlaybackState state, void* data, size_t length) {
    for (auto& cb : playback_listeners_) {
      cb(state, iovec {
        .iov_base = data,
        .iov_len = length
      });
    }
  }

  void GetPlaybackFormat(uint* format, uint* channels, uint* frequency, uint* interval_ms) {
    *format = 2;
    *channels = 2;
    *frequency = 48000;
    *interval_ms = STREAM_FRAME_INTERVAL_MS;
  }

  void GetVolume(uint16_t value, uint16_t mask, bool reverse, bool* mute, int* left, int* right) {
    *mute = (value >> 15) & 1;
    *right = 256 * (value & mask) / mask;
    *left = 256 * ((value >> 8) & mask) / mask;
    if (reverse) {
      *right = 256 - *right;
      *left = 256 - *left;
    }
  }

  void ApplyStreamVolume(StreamState* stream) {
    bool mute;
    int left, right;

    if (stream->output) {
      bool master_mute;
      int master_left, master_right;
      GetVolume(mixer_.master_volume_mute, 0x3F, true, &master_mute, &master_left, &master_right);
      GetVolume(mixer_.pcm_out_volume_mute, 0x1F, true, &mute, &left, &right);
      mute = mute || master_mute;
      left = left * master_left / 256;
      right = right * master_right / 256;
    } else {
      GetVolume(mixer_.line_in_volume_mute, 0x1F, false, &mute, &left, &right);
    }

    auto begin = (int16_t*)stream->buffer.data();
    auto end = (int16_t*)(stream->buffer.data() + stream->buffer_pointer);
    if (mute) {
      bzero(begin, stream->buffer_pointer);
    } else {
      for (auto ptr = begin; ptr < end; ptr += 2) {
        ptr[0] = (left * ptr[0]) >> 8;
        ptr[1] = (right * ptr[1]) >> 8;
      }
    }
  }

  void OnStreamTimer(StreamState* stream) {
    if (stream->output) {
      auto to_transfer = stream->buffer.size() - stream->buffer_pointer;
      auto ptr = (uint8_t*)&stream->buffer.data()[stream->buffer_pointer];
      size_t bytes = TransferData(stream->index, ptr, to_transfer);
      stream->position += bytes;
      stream->buffer_pointer += bytes;

      if (stream->buffer_pointer >= stream->buffer.size()) {
        ApplyStreamVolume(stream);
        NotifyPlayback(kPlaybackData, stream->buffer.data(), stream->buffer_pointer);
        stream->buffer_pointer = 0;
      }
    } else {
      /* input silence if record buffer is unavailable */
      TransferData(stream->index, (uint8_t*)stream->buffer.data(), stream->buffer.size());
      stream->position += stream->buffer.size();
    }

    auto started_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - stream->start_time).count();
    auto current_ms = stream->position * 1000 / stream->bytes_per_second;
    int64_t next_interval_ms = current_ms - started_ms;
    
    if (next_interval_ms < 1) {
      /* If we cannot catch up, reset timer */
      if (next_interval_ms < -100) {
        MV_LOG("stream[%d] reset timer, current_ms=%dms started_ms=%dms", stream->index, current_ms, started_ms);
        stream->position = 0;
        stream->start_time = std::chrono::steady_clock::now();
      } 
      next_interval_ms = 1;
    }

    int64_t next_interval_ns = 1000000LL * next_interval_ms;
    if (stream->timer->interval_ns != next_interval_ns) {
      ModifyTimer(stream->timer, next_interval_ns);
    }
  }

  void SetStreamRunning(uint index, bool running) {
    auto stream = &streams_[index];
    if (stream->running == running) {
      return;
    }
    if (debug_) {
      MV_LOG("set stream[%u] running=%d", index, running);
    }

    stream->running = running;
    if (running) {
      if (stream->output) {
        NotifyPlayback(kPlaybackStart, nullptr, 0);
      }

      ParseBufferDescriptorList(index);
      stream->position = 0;
      stream->start_time = std::chrono::steady_clock::now();
      MV_ASSERT(stream->timer == nullptr);
      stream->timer = AddTimer(1, true, [this, stream]() {
        OnStreamTimer(stream);
      });
    } else {
      RemoveTimer(&stream->timer);

      if (stream->output) {
        NotifyPlayback(kPlaybackStop, nullptr, 0);
      }
    }
  }

  void CheckIrqLevel() {
    for (size_t i = 0; i < bus_masters_.size(); i++) {
      auto &bm = bus_masters_[i];
      uint int_mask = 1 << (i + 5);
      if ((bm.status & TS_END_INTERRUPT) && (bm.control & (1 << 2))) {
        global_status_ |= int_mask;
      } else if ((bm.status & TS_COMPLETION_INTERRUPT) && (bm.control & (1 << 4))){
        global_status_ |= int_mask;
      } else {
        global_status_ &= ~int_mask;
      }
    }

    uint level = !!(global_status_ & ((1 << 5) | (1 << 6) | (1 << 7)));
    SetIrq(level);
  }

  void PrefetchBuffer(uint index) {
    auto &bm = bus_masters_[index];
    auto stream = &streams_[index];

    if (bm.bdl_current_index == bm.bdl_last_valid_index) {
      bm.status |= TS_HALTED;
      return;
    }

    bm.bdl_current_index = bm.bdl_prefetched_index;
    bm.bdl_prefetched_index = (bm.bdl_prefetched_index + 1) % 32;

    auto &bdl_entry = stream->bdl_entries[bm.bdl_current_index];
    stream->bdl_current_data = (uint8_t*)manager_->TranslateGuestMemory(bdl_entry.data_address);
    bm.samples_left = bdl_entry.samples;
    bm.status &= ~TS_END_OF_TRANSFER;
    
    if (!stream->output) {
      manager_->AddDirtyMemory(bdl_entry.data_address, 2ul * bm.samples_left);
    }
  }

  size_t TransferData(uint index, uint8_t* destination, size_t length) {
    auto &bm = bus_masters_[index];
    auto stream = &streams_[index];

    if (bm.status & TS_HALTED) { // halted
      // MV_LOG("halted");
      return length;
    }

    bool interrupt = false;
    size_t copied = 0;
    while (copied < length && !interrupt) {
      size_t to_copy = length - copied;

      auto &bdl_entry = stream->bdl_entries[bm.bdl_current_index];

      if (to_copy > 2ul * bm.samples_left) {
        to_copy = 2ul * bm.samples_left;
      }

      /* IN or OUT */
      if (stream->output) {
        memcpy(&destination[copied], stream->bdl_current_data, to_copy);
      } else {
        memcpy(stream->bdl_current_data, &destination[copied], to_copy);
      }
      stream->bdl_current_data += to_copy;
      bm.samples_left -= to_copy >> 1;
      copied += to_copy;

      /* Check if we should read next buffer */
      if (bm.samples_left == 0) {
        if (bdl_entry.flags & (1 << 15)) {  // Interrupt on completion
          bm.status |= TS_COMPLETION_INTERRUPT;
          interrupt = true;
        }

        if (bm.bdl_current_index == bm.bdl_last_valid_index) {
          bm.status |= TS_HALTED | TS_END_OF_TRANSFER | TS_END_INTERRUPT;
          interrupt = true;
        } else {
          PrefetchBuffer(index);
        }
      }
    }

    if (interrupt) {
      CheckIrqLevel();
    }

    if (debug_) {
      MV_LOG("stream[%u] transfer data %lu/%lu", index, copied, length);
    }
    return copied;
  }

  void ParseBufferDescriptorList(uint index) {
    auto &bm = bus_masters_[index];
    auto stream = &streams_[index];
    stream->bdl_entries = (BufferDescriptor*)manager_->TranslateGuestMemory(bm.bdl_address);
    PrefetchBuffer(index);
  }

  void WriteMixer(uint64_t offset, uint8_t* data, uint32_t size) {
    MV_ASSERT(size == 2);
    uint16_t value = *(uint16_t*)data;
    switch (offset)
    {
    case offsetof(Ac97Mixer, reset):
      ResetMixer();
      break;
    case offsetof(Ac97Mixer, master_volume_mute):
    case offsetof(Ac97Mixer, pcm_out_volume_mute):
    case offsetof(Ac97Mixer, line_in_volume_mute):
    case offsetof(Ac97Mixer, record_gain_mute):
      memcpy((uint8_t*)&mixer_ + offset, data, size);
      break;
    case offsetof(Ac97Mixer, record_select):
      if (debug_) {
        MV_LOG("record select value=0x%x", value);
      }
      break;
    case offsetof(Ac97Mixer, powerdown_ctrl_stat):
      mixer_.powerdown_ctrl_stat = (value & ~0x800F) | (mixer_.powerdown_ctrl_stat & 0xF);
      break;
    case offsetof(Ac97Mixer, extended_audio_ctrl_stat):
      mixer_.extended_audio_ctrl_stat = value;
      break;
    default:
      if (debug_) {
        MV_LOG("ignore write mixer offset=0x%lx data=%x size=%d", offset, value, size);
      }
    }
  }

  void WriteBusMaster(uint index, uint64_t offset, uint8_t* data, uint32_t size) {
    auto &bm = bus_masters_[index];
    switch (offset)
    {
    case offsetof(Ac97BusMaster, bdl_address):
      memcpy((uint8_t*)&bm + offset, data, size);
      break;
    case offsetof(Ac97BusMaster, bdl_last_valid_index):
      if ((bm.control & 1) && (bm.status & TS_HALTED)) {  // running but halted, try to restart
        PrefetchBuffer(index);
        bm.status &= ~(TS_HALTED | TS_END_OF_TRANSFER);
      }
      bm.bdl_last_valid_index = data[0] & 0x1F;
      break;
    case offsetof(Ac97BusMaster, status):
      bm.status &= ~(data[0] & (TS_END_INTERRUPT | TS_COMPLETION_INTERRUPT | TS_ERROR_INTERRUPT));
      CheckIrqLevel();
      break;
    case offsetof(Ac97BusMaster, control):
      if (data[0] & 2) { // reset bus master
        bm.bdl_address = 0;
        bm.bdl_current_index = 0;
        bm.bdl_last_valid_index = 0;
        bm.bdl_prefetched_index = 0;
        bm.samples_left = 0;
        bm.status = TS_HALTED;
        bm.control = bm.control & ~3;
        SetStreamRunning(index, false);
        CheckIrqLevel();
      } else {
        bm.control = data[0];
        if (data[0] & 1) {  // run
          bm.status &= ~TS_HALTED;
          SetStreamRunning(index, true);
        } else {  // stop
          SetStreamRunning(index, false);
          bm.status |= TS_HALTED;
        }
      }
      break;
    }
  }

  void ReadBusMasterControl(uint64_t offset, uint8_t* data, uint32_t size) {
    switch (offset)
    {
    case 0x00 ... 0x2B: {
      uint index = (offset >> 4) & 3;
      offset &= 0xF;
      memcpy(data, (uint8_t*)&bus_masters_[index] + offset, size);
      break;
    }
    case 0x2C:  // global control
      memcpy(data, &global_control_, size);
      break;
    case 0x30:  // global status
      memcpy(data, &global_status_, size);
      break;
    case 0x34:  // codec access semaphore (0: no access in progress 1: busy)
      data[0] = codec_accessing_;
      codec_accessing_ = 1;
      break;
    default:
      memset(data, 0xFF, size);
      MV_ERROR("unhandled read bus master offset=0x%lx size=%u", offset, size);
    }
  }

  void WriteBusMasterControl(uint64_t offset, uint8_t* data, uint32_t size) {
    switch (offset)
    {
    case 0x00 ... 0x2B: {
      uint index = (offset >> 4) & 3;
      offset &= 0xF;
      WriteBusMaster(index, offset, data, size);
      break;
    }
    case 0x2C: { // global control
      MV_ASSERT(size == 4);
      uint32_t value = *(uint32_t*)data;
      uint32_t valid_mask = (1 << 6) - 1;
      if (value & 2) {  // cold reset
        Reset();
        if (debug_) {
          MV_LOG("cold reset");
        }
      } else if (value & 4) { // soft reset
        MV_ERROR("soft reset not supported yet");
      } else {
        global_control_ = value & valid_mask;
        if (debug_) {
          MV_LOG("set global control=%x", value);
        }
      }
      break;
    }
    case 0x30: {  // global status (support writing 1 to clear bits)
      uint32_t valid_mask = (1 << 0) | (1 << 10) | (1 << 15);
      uint32_t value = 0;
      memcpy(&value, data, size);
      global_status_ &= ~(value & valid_mask);
      break;
    }
    default:
      MV_ERROR("unhandled write bus master offset=0x%lx data=%x size=%u", offset, *(uint32_t*)data, size);
    }
  }

  void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    if (resource->base == pci_bars_[0].address) {
      if (offset + size > sizeof(mixer_)) {
        memset(data, 0xFF, size);
      } else {
        memcpy(data, (uint8_t*)&mixer_ + offset, size);
        codec_accessing_ = 0;
      }
      if (debug_) {
        MV_LOG("read mixer offset=0x%lx data=%x size=%u", offset, *(uint16_t*)data, size);
      }
    } else if (resource->base == pci_bars_[1].address) {
      ReadBusMasterControl(offset, data, size);
    } else {
      PciDevice::Read(resource, offset, data, size);
    }
  }

  void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    if (resource->base == pci_bars_[0].address) {
      MV_ASSERT(offset + size <= sizeof(mixer_));
      WriteMixer(offset, data, size);
      codec_accessing_ = 0;
      if (debug_) {
        MV_LOG("write mixer offset=0x%lx data=%x size=%u", offset, *(uint16_t*)data, size);
      }
    } else if (resource->base == pci_bars_[1].address) {
      WriteBusMasterControl(offset, data, size);
    } else {
      PciDevice::Write(resource, offset, data, size);
    }
  }

  std::list<PlaybackListener>::iterator RegisterPlaybackListener(PlaybackListener callback) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return playback_listeners_.emplace(playback_listeners_.end(), callback);
  }

  void UnregisterPlaybackListener(std::list<PlaybackListener>::iterator it) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    playback_listeners_.erase(it);
  }
};

DECLARE_DEVICE(Ac97);
