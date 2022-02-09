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
#include <string>
#include <map>
#include <vector>
#include <chrono>
#include <array>
#include <cstdio>
#include <cmath>
#include "device.h"
#include "hda_internal.h"
#include "device_manager.h"

#define HDA_TIMER_INTERVAL_MS (10) // 10ms
#define HDA_STREAM_BUFFER_SIZE 8192

struct HdaStream {
  uint32_t id;
  uint32_t channel;
  uint32_t format;
  uint32_t gain_left, gain_right;
  bool mute_left, mute_right;
  bool output;
  bool running;
  size_t position;
  uint32_t nchannels;
  uint32_t frequency;
  size_t bytes_per_second;
  IoTimer* timer;
  IoTimePoint start_time;
  TransferCallback transfer_callback;
  uint8_t buffer[HDA_STREAM_BUFFER_SIZE];
};

struct HdaNode {
  uint32_t    id;
  std::string name;
  uint32_t    config;
  uint32_t    pin_control;
  uint32_t    stream_index;
  std::map<uint32_t, uint32_t> parameters;
  std::vector<uint32_t> connection;
  HdaStream*  stream;
};

class HdaDuplex : public Device, public HdaCodecInterface {
 private:
  uint32_t subsystem_id_;
  uint32_t pcm_formats_;
  std::vector<HdaNode> nodes_;
  std::array<HdaStream, 2> streams_;
  FILE* fp_output_ = nullptr;

 public:
  void Connect() {
    Device::Connect();
    if (debug_) {
      fp_output_ = fopen("/tmp/sound.dat", "wb");
      MV_ASSERT(fp_output_);
    }
  }

  virtual void Disconnect() {
    if (fp_output_) {
      fclose(fp_output_);
    }
    Device::Disconnect();
  }

  void Reset() {
    InitializeCodec();
  }

  void InitializeCodec() {
    bzero(streams_.data(), streams_.size() * sizeof(HdaStream));
    nodes_.clear();
    subsystem_id_ = (0x1AF4 << 16) | 0x21;  // duplex, no mixer
    pcm_formats_ = AC_SUPPCM_BITS_16 | 0x1FC;
    
    // root node
    HdaNode root = { .id = AC_NODE_ROOT, .name = "root" };
    root.parameters[AC_PAR_VENDOR_ID] = subsystem_id_;
    root.parameters[AC_PAR_SUBSYSTEM_ID] = subsystem_id_;
    root.parameters[AC_PAR_REV_ID] = 0x00100101;
    root.parameters[AC_PAR_NODE_COUNT] = 0x00010001;
    nodes_.push_back(root);

    // audio node
    HdaNode audio = { .id = 1, .name = "func" };
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
    HdaNode dac = { .id = 2, .name = "dac" };
    dac.stream_index = 0;
    dac.stream = &streams_[0];
    dac.stream->output = true;
    dac.stream->gain_left = dac.stream->gain_right = 0x4A;
    dac.stream->format = AC_FMT_TYPE_PCM | AC_FMT_BITS_16 | (1 << AC_FMT_CHAN_SHIFT);
    dac.parameters[AC_PAR_AUDIO_WIDGET_CAP] = ((AC_WID_AUD_OUT << AC_WCAP_TYPE_SHIFT) |
      AC_WCAP_FORMAT_OVRD | AC_WCAP_AMP_OVRD | AC_WCAP_STEREO);
    dac.parameters[AC_PAR_PCM] = pcm_formats_;
    dac.parameters[AC_PAR_STREAM] = AC_SUPFMT_PCM;
    nodes_.push_back(dac);

    // out node
    HdaNode out = { .id = 3, .name = "out" };
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
    HdaNode adc = { .id = 4, .name = "adc" };
    adc.stream_index = 1;
    adc.stream = &streams_[1];
    adc.stream->output = false;
    adc.stream->format = AC_FMT_TYPE_PCM | AC_FMT_BITS_16 | (1 << AC_FMT_CHAN_SHIFT);
    adc.connection.push_back(5);
    adc.parameters[AC_PAR_AUDIO_WIDGET_CAP] = (AC_WID_AUD_IN | AC_WCAP_TYPE_SHIFT) |
      AC_WCAP_CONN_LIST | AC_WCAP_FORMAT_OVRD | AC_WCAP_AMP_OVRD | AC_WCAP_STEREO;
    adc.parameters[AC_PAR_CONNLIST_LEN] = adc.connection.size();
    adc.parameters[AC_PAR_PCM] = pcm_formats_;
    adc.parameters[AC_PAR_STREAM] = AC_SUPFMT_PCM;
    adc.parameters[AC_PAR_AMP_OUT_CAP] = 0;
    nodes_.push_back(adc);

    // in
    HdaNode in = { .id = 5, .name = "in" };
    in.config = (AC_JACK_PORT_COMPLEX << AC_DEFCFG_PORT_CONN_SHIFT) |
      (AC_JACK_LINE_IN << AC_DEFCFG_DEVICE_SHIFT) |
      (AC_JACK_CONN_UNKNOWN << AC_DEFCFG_CONN_TYPE_SHIFT) |
      (AC_JACK_COLOR_RED << AC_DEFCFG_COLOR_SHIFT) | 0x20;
    in.pin_control = AC_PINCTL_IN_EN;
    in.parameters[AC_PAR_AUDIO_WIDGET_CAP] = (AC_WID_PIN << AC_WCAP_TYPE_SHIFT) | AC_WCAP_STEREO;
    in.parameters[AC_PAR_PIN_CAP] = AC_PINCAP_IN;
    nodes_.push_back(in);
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
      callback(FindNodeParamter(node, payload));
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
      break;
    case AC_VERB_SET_STREAM_FORMAT:
      node.stream->format = payload;
      SetupStream(node.stream);
      if (debug_) {
        MV_LOG("node[%d] %s format=0x%x nchannels=%d frequency=%d", node.id, node.name.c_str(),\
          node.stream->format, node.stream->nchannels, node.stream->frequency);
      }
      callback(0);
      break;
    case AC_VERB_GET_PIN_WIDGET_CONTROL:
      callback(node.pin_control);
      break;
    case AC_VERB_SET_PIN_WIDGET_CONTROL:
      MV_ASSERT(node.pin_control == payload);
      callback(0);
      break;
    case AC_VERB_GET_CONFIG_DEFAULT:
      callback(node.config);
      break;
    case AC_VERB_GET_CONNECT_LIST: {
      uint32_t count = FindNodeParamter(node, AC_PAR_CONNLIST_LEN);
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
      MV_PANIC("unhandled verb 0x%x", verb);
      break;
    }
  }

  uint32_t FindNodeParamter(const HdaNode& node, uint32_t id) {
    auto it = node.parameters.find(id);
    if (it == node.parameters.end()) {
      MV_PANIC("failed to find node %s parameter 0x%x", node.name.c_str(), id);
      return 0;
    }
    return it->second;
  }

  void SetupStream(HdaStream* stream) {
    if (stream->format & AC_FMT_TYPE_NON_PCM) {
      return;
    }
    MV_ASSERT((stream->format & AC_FMT_BITS_MASK) == AC_FMT_BITS_16);

    stream->frequency = (stream->format & AC_FMT_BASE_44K) ? 44100 : 48000;

    uint32_t mul = (stream->format & AC_FMT_MULT_MASK) >> AC_FMT_MULT_SHIFT;
    uint32_t div = (stream->format & AC_FMT_DIV_MASK) >> AC_FMT_DIV_SHIFT;
    stream->frequency = stream->frequency * (mul + 1) / (div + 1);
    stream->nchannels = ((stream->format & AC_FMT_CHAN_MASK) >> AC_FMT_CHAN_SHIFT) + 1;
    stream->bytes_per_second = 2LL * stream->nchannels * stream->frequency;
  }

  void OnStreamTimer(HdaStream* stream) {
    size_t transferred = stream->transfer_callback(stream->buffer, HDA_STREAM_BUFFER_SIZE);
    stream->position += transferred;
    if (fp_output_) {
      fwrite(stream->buffer, transferred, 1, fp_output_);
    }

    auto written_ms = stream->position * 1000 / stream->bytes_per_second;
    auto started_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - stream->start_time).count();
    int next_interval = written_ms - started_ms;
    if (next_interval < 0) {
      next_interval = 1;
    }

    if (stream->timer->interval_ms != next_interval) {
      manager_->ModifyIoTimer(stream->timer, next_interval);
    }
  }

  void SetStreamRunning(HdaStream* stream, bool running) {
    if (stream->running == running) {
      return;
    }
    stream->running = running;
    if (debug_) {
      MV_LOG("set stream running: %d", running ? 1 : 0);
    }

    if (running) {
      stream->position = 0;
      stream->start_time = std::chrono::steady_clock::now();
      MV_ASSERT(stream->timer == nullptr);
      stream->timer = manager_->RegisterIoTimer(this, 1, true, [this, stream]() {
        OnStreamTimer(stream);
      });
    } else {
      MV_ASSERT(stream->timer);
      manager_->UnregisterIoTimer(stream->timer);
      stream->timer = nullptr;
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
};

DECLARE_DEVICE(HdaDuplex);
