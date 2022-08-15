/* 
 * MVisor HDA Codec
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

#include "hda_codec.h"

#include <cstring>
#include <cstdio>
#include <cmath>

#include <string>
#include <map>
#include <vector>
#include <chrono>
#include <array>

#include "device.h"
#include "hda_internal.h"
#include "device_manager.h"
#include "device_interface.h"
#include "hda_duplex.pb.h"
#include "logger.h"

#define HDA_TIMER_INTERVAL_MS   (10)
#define HDA_STREAM_BUFFER_SIZE  (480 * 20)

struct HdaStream {
  uint32_t  id;
  uint32_t  channel;
  uint32_t  format;
  uint32_t  gain_left, gain_right;
  bool      mute_left, mute_right;

  bool      output;
  bool      running = false;
  size_t    position;
  uint32_t  nchannels;
  uint32_t  frequency;
  size_t    bytes_per_second;
  size_t    bytes_per_frame;

  IoTimer*  timer = nullptr;
  IoTimePoint start_time;
  TransferCallback transfer_callback;
  uint8_t   buffer[HDA_STREAM_BUFFER_SIZE];
  size_t    buffer_pointer = 0;
};

struct HdaNode {
  uint32_t    id;
  std::string name;
  uint32_t    config = 0;
  uint32_t    pin_control = 0;
  uint32_t    stream_index = 0;
  std::map<uint32_t, uint32_t> parameters;
  std::vector<uint32_t> connection;
  HdaStream*  stream = nullptr;
};

class HdaDuplex : public Device, public HdaCodecInterface, public PlaybackInterface, public RecordInterface {
 private:
  uint32_t                  subsystem_id_;
  uint32_t                  pcm_formats_;
  std::vector<HdaNode>      nodes_;
  std::array<HdaStream, 2>  streams_;
  std::vector<PlaybackListener> playback_listeners_;
  std::vector<RecordListener>   record_listeners_;
  std::deque<std::string>        record_buffer_;

 public:
  HdaDuplex() {
    set_parent_name("ich9-hda");
  }

  virtual ~HdaDuplex() {
  }

  void Connect() {
    Device::Connect();
  }

  virtual void Disconnect() {
    for (auto& stream : streams_) {
      if (stream.running) {
        SetStreamRunning(&stream, false);
      }
    }
    Device::Disconnect();
  }

  void Reset() {
    for (auto& stream : streams_) {
      if (stream.running) {
        SetStreamRunning(&stream, false);
      }
    }
    InitializeCodec();
  }

  bool SaveState(MigrationWriter* writer) {
    HdaDuplexState state;
    for (size_t i = 0; i < streams_.size(); i++) {
      auto s = state.add_streams();
      s->set_id(streams_[i].id);
      s->set_channel(streams_[i].channel);
      s->set_format(streams_[i].format);
      s->set_gain_left(streams_[i].gain_left);
      s->set_gain_right(streams_[i].gain_right);
      s->set_mute_left(streams_[i].mute_left);
      s->set_mute_right(streams_[i].mute_right);
    }
    writer->WriteProtobuf("HDA_DUPLEX_CODEC", state);
    return Device::SaveState(writer);
  }

  bool LoadState(MigrationReader* reader) {
    if (!Device::LoadState(reader))
      return false;
    HdaDuplexState state;
    if (!reader->ReadProtobuf("HDA_DUPLEX_CODEC", state)) {
      return false;
    }
    for (size_t i = 0; i < streams_.size(); i++) {
      auto& s = state.streams(i);
      streams_[i].id = s.id();
      streams_[i].channel = s.channel();
      streams_[i].format = s.format();
      streams_[i].gain_left = s.gain_left();
      streams_[i].gain_right = s.gain_right();
      streams_[i].mute_left = s.mute_left();
      streams_[i].mute_right = s.mute_right();
      SetupStream(&streams_[i]);
    }
    return true;
  }

  void InitializeCodec() {
    nodes_.clear();
    subsystem_id_ = (0x1AF4 << 16) | 0x21;  // duplex, no mixer
    pcm_formats_ = AC_SUPPCM_BITS_16 | (1 << 6); // 48000 Hz
    
    // root node
    HdaNode root;
    root.id = AC_NODE_ROOT;
    root.name = "root";
    root.parameters[AC_PAR_VENDOR_ID] = subsystem_id_;
    root.parameters[AC_PAR_SUBSYSTEM_ID] = subsystem_id_;
    root.parameters[AC_PAR_REV_ID] = 0x00100101;
    root.parameters[AC_PAR_NODE_COUNT] = 0x00010001;
    nodes_.push_back(root);

    // audio node
    HdaNode audio;
    audio.id = 1;
    audio.name = "func";
    audio.parameters[AC_PAR_SUBSYSTEM_ID] = subsystem_id_;
    audio.parameters[AC_PAR_FUNCTION_TYPE] = AC_GRP_AUDIO_FUNCTION;
    audio.parameters[AC_PAR_NODE_COUNT] = 0x00020004;
    audio.parameters[AC_PAR_PCM] = pcm_formats_;
    audio.parameters[AC_PAR_STREAM] = AC_SUPFMT_PCM;
    audio.parameters[AC_PAR_GPIO_CAP] = 0;
    audio.parameters[AC_PAR_AUDIO_FG_CAP] = 0x00000808;
    audio.parameters[AC_PAR_AMP_IN_CAP] = 0;
    audio.parameters[AC_PAR_AMP_OUT_CAP] = 0;
    audio.parameters[AC_PAR_POWER_STATE] = 0;
    audio.parameters[AC_PAR_AUDIO_WIDGET_CAP] = 0;
    audio.parameters[AC_PAR_PIN_CAP] = 0;
    audio.parameters[AC_PAR_CONNLIST_LEN] = 0;
    audio.parameters[AC_PAR_PROC_CAP] = 0;
    audio.parameters[AC_PAR_VOL_KNB_CAP] = 0;
    nodes_.push_back(audio);

    // dac node
    HdaNode dac;
    dac.id = 2;
    dac.name = "dac";
    dac.stream_index = 0;
    dac.stream = &streams_[0];
    dac.stream->output = true;
    dac.stream->gain_left = dac.stream->gain_right = 0x4A;
    dac.stream->format = AC_FMT_TYPE_PCM | AC_FMT_BITS_16 | (1 << AC_FMT_CHAN_SHIFT);
    dac.parameters[AC_PAR_AUDIO_WIDGET_CAP] = ((AC_WID_AUD_OUT << AC_WCAP_TYPE_SHIFT) |
      AC_WCAP_FORMAT_OVRD | AC_WCAP_STEREO);
    dac.parameters[AC_PAR_PCM] = pcm_formats_;
    dac.parameters[AC_PAR_STREAM] = AC_SUPFMT_PCM;
    nodes_.push_back(dac);

    // out node
    HdaNode out;
    out.id = 3;
    out.name = "out";
    out.config = (AC_JACK_PORT_COMPLEX << AC_DEFCFG_PORT_CONN_SHIFT) |
      (AC_JACK_LINE_OUT << AC_DEFCFG_DEVICE_SHIFT) |
      (AC_JACK_CONN_UNKNOWN << AC_DEFCFG_CONN_TYPE_SHIFT) |
      (AC_JACK_COLOR_GREEN << AC_DEFCFG_COLOR_SHIFT) | 0x10;
    out.connection.push_back(2);
    out.pin_control = AC_PINCTL_OUT_EN;
    out.parameters[AC_PAR_AUDIO_WIDGET_CAP] = (AC_WID_PIN << AC_WCAP_TYPE_SHIFT) |
      AC_WCAP_CONN_LIST | AC_WCAP_STEREO;
    out.parameters[AC_PAR_PIN_CAP] = AC_PINCAP_OUT;
    out.parameters[AC_PAR_CONNLIST_LEN] = out.connection.size();
    nodes_.push_back(out);

    // adc node
    HdaNode adc;
    adc.id = 4;
    adc.name = "adc";
    adc.stream_index = 1;
    adc.stream = &streams_[1];
    adc.stream->output = false;
    adc.stream->format = AC_FMT_TYPE_PCM | AC_FMT_BITS_16 | (1 << AC_FMT_CHAN_SHIFT);
    adc.connection.push_back(5);
    adc.parameters[AC_PAR_AUDIO_WIDGET_CAP] = (AC_WID_AUD_IN << AC_WCAP_TYPE_SHIFT) |
      AC_WCAP_CONN_LIST | AC_WCAP_FORMAT_OVRD | AC_WCAP_STEREO;
    adc.parameters[AC_PAR_CONNLIST_LEN] = adc.connection.size();
    adc.parameters[AC_PAR_PCM] = pcm_formats_;
    adc.parameters[AC_PAR_STREAM] = AC_SUPFMT_PCM;
    adc.parameters[AC_PAR_AMP_OUT_CAP] = 0;
    nodes_.push_back(adc);

    // in
    HdaNode in;
    in.id = 5;
    in.name = "in";
    in.config = (AC_JACK_PORT_COMPLEX << AC_DEFCFG_PORT_CONN_SHIFT) |
      (AC_JACK_LINE_IN << AC_DEFCFG_DEVICE_SHIFT) |
      (AC_JACK_CONN_UNKNOWN << AC_DEFCFG_CONN_TYPE_SHIFT) |
      (AC_JACK_COLOR_RED << AC_DEFCFG_COLOR_SHIFT) | 0x20;
    in.pin_control = AC_PINCTL_IN_EN;
    in.parameters[AC_PAR_AUDIO_WIDGET_CAP] = (AC_WID_PIN << AC_WCAP_TYPE_SHIFT) | AC_WCAP_STEREO;
    in.parameters[AC_PAR_PIN_CAP] = AC_PINCAP_IN;
    nodes_.push_back(in);

    // setup with default format
    for (auto &stream : streams_) {
      SetupStream(&stream);
    }
  }

  void StartCommand(uint8_t node_id, uint32_t data, ResponseCallback callback) {
    uint32_t verb, payload;
    if ((data & 0x70000) == 0x70000) { // id/payload 12/8
      verb = (data >> 8) & 0xFFF;
      payload = data & 0x00FF;
    } else { // id/payload 4/16
      verb = (data >> 8) & 0xF00;
      payload = data & 0xFFFF;
    }

    MV_ASSERT(node_id < nodes_.size());
    HdaNode& node = nodes_[node_id];
    if (debug_) {
      MV_LOG("node=%d verb=0x%x payload=0x%x", node_id, verb, payload);
    }

    switch (verb)
    {
    case AC_VERB_PARAMETERS:
      callback(FindNodeParameter(node, payload));
      break;
    case AC_VERB_GET_STREAM_FORMAT:
      MV_ASSERT(node.stream);
      callback(node.stream->format);
      break;
    case AC_VERB_GET_CONV:
      MV_ASSERT(node.stream);
      callback((node.stream->id << 4) | node.stream->channel);
      break;
    case AC_VERB_SET_CHANNEL_STREAMID:
      MV_ASSERT(node.stream);
      SetStreamRunning(node.stream, false);
      node.stream->id = (payload >> 4) & 0xF;
      node.stream->channel = payload & 0xF;
      callback(0);
      if (debug_) {
        MV_LOG("node[%d] %s stream_id=%d channel=%d", node.id, node.name.c_str(),
          node.stream->id, node.stream->channel);
      }
      break;
    case AC_VERB_SET_STREAM_FORMAT:
      node.stream->format = payload;
      SetupStream(node.stream);
      if (debug_) {
        MV_LOG("node[%d] %s format=0x%x nchannels=%d frequency=%d", node.id, node.name.c_str(),
          node.stream->format, node.stream->nchannels, node.stream->frequency);
      }
      callback(0);
      break;
    case AC_VERB_GET_PIN_WIDGET_CONTROL:
      callback(node.pin_control);
      break;
    case AC_VERB_SET_PIN_WIDGET_CONTROL:
      /* If OS volume is muted, pin_control is set zero. */
      node.pin_control = payload;
      callback(0);
      break;
    case AC_VERB_GET_CONFIG_DEFAULT:
      callback(node.config);
      break;
    case AC_VERB_GET_CONNECT_LIST: {
      uint32_t count = FindNodeParameter(node, AC_PAR_CONNLIST_LEN);
      uint32_t response = 0, shift = 0;
      while (payload < count && shift < 32) {
        response |= node.connection[payload] << shift;
        payload++;
        shift += 8;
      }
      callback(response);
      break;
    }
    case AC_VERB_GET_AMP_GAIN_MUTE:
      MV_ASSERT(node.stream);
      if (payload & AC_AMP_GET_LEFT) {
        callback(node.stream->gain_left | (node.stream->mute_left ? AC_AMP_MUTE : 0));
      } else {
        callback(node.stream->gain_right | (node.stream->mute_right ? AC_AMP_MUTE : 0));
      }
      break;
    case AC_VERB_SET_AMP_GAIN_MUTE:
      MV_ASSERT(node.stream);
      if (payload & AC_AMP_SET_LEFT) {
        node.stream->gain_left = payload & AC_AMP_GAIN;
        node.stream->mute_left = payload & AC_AMP_MUTE;
      }
      if (payload & AC_AMP_SET_RIGHT) {
        node.stream->gain_right = payload & AC_AMP_GAIN;
        node.stream->mute_right = payload & AC_AMP_MUTE;
      }
      callback(0);
      break;
    /* not supported */
    case AC_VERB_GET_SDI_SELECT:
    case AC_VERB_SET_CODEC_RESET:
    case AC_VERB_GET_POWER_STATE:
      callback(0);
      break;
    default:
      if (debug_) {
        MV_LOG("unhandled verb 0x%x payload=0x%x", verb, payload);
      }
      callback(0);
      break;
    }
  }

  uint32_t FindNodeParameter(const HdaNode& node, uint32_t id) {
    auto it = node.parameters.find(id);
    if (it == node.parameters.end()) {
      MV_PANIC("failed to find node %s parameter 0x%x", node.name.c_str(), id);
      return 0;
    }
    return it->second;
  }

  void SetupStream(HdaStream* stream) {
    stream->buffer_pointer = 0;
    stream->frequency = 0;
    stream->channel = 0;
    stream->bytes_per_second = stream->bytes_per_frame = 0;
    if (stream->format & AC_FMT_TYPE_NON_PCM) {
      return;
    }
    if ((stream->format & AC_FMT_BITS_MASK) != AC_FMT_BITS_16) {
      if (debug_) {
        MV_LOG("not supported audio format 0x%x", stream->format);
      }
      return;
    }

    stream->frequency = (stream->format & AC_FMT_BASE_44K) ? 44100 : 48000;

    uint32_t mul = (stream->format & AC_FMT_MULT_MASK) >> AC_FMT_MULT_SHIFT;
    uint32_t div = (stream->format & AC_FMT_DIV_MASK) >> AC_FMT_DIV_SHIFT;
    stream->frequency = stream->frequency * (mul + 1) / (div + 1);
    stream->nchannels = ((stream->format & AC_FMT_CHAN_MASK) >> AC_FMT_CHAN_SHIFT) + 1;
    stream->bytes_per_second = 2LL * stream->nchannels * stream->frequency;
    stream->bytes_per_frame = stream->bytes_per_second / (1000 / HDA_TIMER_INTERVAL_MS);
  }

  void OnStreamTimer(HdaStream* stream) {
    if (stream->output) {
      auto buffer_size = stream->bytes_per_frame;
      if (buffer_size > HDA_STREAM_BUFFER_SIZE)
        buffer_size = HDA_STREAM_BUFFER_SIZE;
      auto to_transfer = buffer_size - stream->buffer_pointer;
      size_t bytes = stream->transfer_callback(&stream->buffer[stream->buffer_pointer], to_transfer);
      stream->position += bytes;
      stream->buffer_pointer += bytes;

      if (stream->buffer_pointer >= buffer_size) {
        NotifyPlayback(kPlaybackData, stream->buffer, stream->buffer_pointer);
        stream->buffer_pointer = 0;
      }
    } else {
      if(!record_buffer_.empty()) {
        auto buffer = record_buffer_.front();
        record_buffer_.pop_front();
        stream->transfer_callback((uint8_t*)buffer.data(), buffer.size());
        stream->position += buffer.size();
      } else {
        auto buffer_size = stream->bytes_per_frame;
        uint8_t zero[buffer_size] = { 0 };
        stream->transfer_callback(zero, buffer_size);
        stream->position += buffer_size;
      }
    }

    auto started_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - stream->start_time).count();
    auto current_ms = stream->position * 1000 / stream->bytes_per_second;
    int64_t next_interval_ms = current_ms - started_ms;
    
    if (next_interval_ms < 1) {
      /* If we cannot catch up, reset timer */
      if (next_interval_ms < -100) {
        MV_LOG("stream[%d] reset timer, current_ms=%dms started_ms=%dms", stream->id, current_ms, started_ms);
        stream->position = 0;
        stream->start_time = std::chrono::steady_clock::now();
      } 
      next_interval_ms = 1;
    }

    int64_t next_interval_ns = 1000000LL * next_interval_ms;
    if (stream->timer->interval_ns != next_interval_ns) {
      manager_->io()->ModifyTimer(stream->timer, next_interval_ns);
    }
  }

  void SetStreamRunning(HdaStream* stream, bool running) {
    if (stream->running == running) {
      return;
    }
    stream->running = running;
    if (debug_) {
      MV_LOG("set stream[%d] running=%d", stream->id, running ? 1 : 0);
    }

    if (running) {
      if (stream->output) {
        NotifyPlayback(kPlaybackStart, nullptr, 0);
      } else {
        NotifyRecordEvent(kRecordStart);
        if(!record_buffer_.empty()) {
          record_buffer_.clear();
        }
      }
      stream->position = 0;
      stream->start_time = std::chrono::steady_clock::now();
      MV_ASSERT(stream->timer == nullptr);
      // FIXME: When should the first timer event happen?
      stream->timer = manager_->io()->AddTimer(NS_PER_SECOND / 1000LL, true, [this, stream]() {
        OnStreamTimer(stream);
      });
    } else {
      MV_ASSERT(stream->timer);
      manager_->io()->RemoveTimer(stream->timer);
      stream->timer = nullptr;
      stream->transfer_callback = nullptr;
      if (stream->output) {
        NotifyPlayback(kPlaybackStop, nullptr, 0);
      } else {
        if(!record_buffer_.empty()) {
          record_buffer_.clear();
        }
        NotifyRecordEvent(kRecordStop);
      }
    }
  }

  HdaStream* FindStreamById(uint32_t stream_id, bool output) {
    for (auto& stream : streams_) {
      if (stream.output == output && stream.id == stream_id) {
        return &stream;
      }
    }
    return nullptr;
  }
  
  void StartStream(uint32_t stream_id, bool output, TransferCallback callback) {
    HdaStream* stream = FindStreamById(stream_id, output);
    if (stream) {
      stream->transfer_callback = callback;
      SetStreamRunning(stream, true);
    }
  }

  void StopStream(uint32_t stream_id, bool output) {
    HdaStream* stream = FindStreamById(stream_id, output);
    if (stream) {
      SetStreamRunning(stream, false);
    }
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
    auto& stream = streams_[0];
    *format = 2;
    *channels = stream.nchannels;
    *frequency = stream.frequency;
    *interval_ms = HDA_TIMER_INTERVAL_MS;
  }

  void RegisterPlaybackListener(PlaybackListener callback) {
    playback_listeners_.push_back(callback);
  }

  void WriteStreamToSharedBuffer(HdaStream* stream, const std::string& record_data) {
    size_t frame_bytes = stream->bytes_per_frame;
    size_t write_pos = 0;
    size_t size = record_data.size();
    std::string buffer;
    buffer.resize(frame_bytes);
    while (size >= frame_bytes) {
      memcpy(buffer.data(), record_data.data() + write_pos, frame_bytes);
      record_buffer_.push_back(buffer);
      write_pos += frame_bytes;
      size -= frame_bytes;
    }
  }

  void WriteRecordDataToDevice(const std::string& record_data) {
    HdaStream* stream = FindStreamById(1, false);
    if(stream == nullptr) {
      MV_ERROR("can't find stream by id");
      return;
    }

    manager_->io()->Schedule([this, stream, record_data]() {
        WriteStreamToSharedBuffer(stream, record_data);
    });
  }

  void NotifyRecordEvent(RecordState state) {
    for (auto& cb : record_listeners_) {
      cb(state);
    }
  }

  void RegisterRecordListener(RecordListener callback) {
    record_listeners_.push_back(callback);
  }
};

DECLARE_DEVICE(HdaDuplex);
