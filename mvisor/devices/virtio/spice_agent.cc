/* 
 * MVisor Spice VDAgent
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

#include "virtio_console.h"
#include <cstring>
#include <chrono>
#include "logger.h"
#include "device_interface.h"

using namespace std::chrono;

/* The device uses the server port to send message */
enum VdiSourcePort{
  kVdiClientPort = 1,
  kVdiServerPort
};

struct VdiChunkHeader {
  uint32_t  port;
  uint32_t  size;
} __attribute__((packed));

enum SpiceAgentMessageType {
  kAgentMessageMouseState = 1,
  kAgentMessageMonitorsConfig,
  kAgentMessageReply,
  kAgentMessageClipboard,
  kAgentMessageDisplayConfig,
  kAgentMessageAnnounceCapabilities,
  kAgentMessageClipboardGrab,
  kAgentMessageClipboardRequest,
  kAgentMessageClipboardRelease,
  kAgentMessageFileXferStart,
  kAgentMessageFileXferStatus,
  kAgentMessageFileXferData,
  kAgentMessageClientDisconnected,
  kAgentMessageMaxClipboard,
  kAgentMessageAudioVolumeSync,
  kAgentMessageGraphicsDeviceInfo,
  kAgentMessageEndMessage
};

struct SpiceAgentMouseState {
  uint32_t x;
  uint32_t y;
  uint32_t buttons;
  uint8_t display_id;
} __attribute__((packed));

struct SpiceAgentMessage {
  uint32_t  protocol;
  uint32_t  type;
  uint64_t  opaque;
  uint32_t  size;
  uint8_t   data[];
} __attribute__((packed));

struct SpiceAgentMonitorConfig {
    /*
     * Note a width and height of 0 can be used to indicate a disabled
     * monitor, this may only be used with agents with the
     * VD_AGENT_CAP_SPARSE_MONITORS_CONFIG capability.
     */
    uint32_t height;
    uint32_t width;
    uint32_t depth;
    int32_t x;
    int32_t y;
} __attribute__((packed));

struct SpiceAgentMonitorsConfig {
    uint32_t num_of_monitors;
#define VD_AGENT_CONFIG_MONITORS_FLAG_USE_POS         (1 << 0)
#define VD_AGENT_CONFIG_MONITORS_FLAG_PHYSICAL_SIZE   (1 << 1)
    uint32_t flags;
    SpiceAgentMonitorConfig monitors[1];
    /* only sent if the FLAG_PHYSICAL_SIZE is present: */
    /* VDAgentMonitorMM physical_sizes[0]; */
} __attribute__((packed));

/* Limit send mouse frequency */
const auto kSendMouseInterval = milliseconds(20);

class SpiceAgent : public VirtioConsolePort, public SpiceAgentInterface {
 private:
  bool    pending_resize_event_;
  SpiceAgentMouseState  last_mouse_state_;
  steady_clock::time_point last_send_mouse_time_;
  uint32_t width_, height_;

 public:
  SpiceAgent() {
    strcpy(port_name_, "com.redhat.spice.0");
  }

  void Reset() {
    pending_resize_event_ = false;
    last_send_mouse_time_ = steady_clock::now();
  }

  void OnMessage(uint8_t* data, size_t size) {
    if (size < sizeof(VdiChunkHeader) + sizeof(SpiceAgentMessage)) {
      MV_PANIC("Chunk too small, size=0x%lx", size);
      return;
    }
    VdiChunkHeader* chunk_header = (VdiChunkHeader*)data;
    if (chunk_header->size + sizeof(VdiChunkHeader) != size) {
      MV_PANIC("Invalid chunk size=0x%lx", size);
      return;
    }
    SpiceAgentMessage* message = (SpiceAgentMessage*)(data + sizeof(VdiChunkHeader));
    HandleAgentMessage(message);
  }

  void OnGuestWritable() {
    guest_writable_ = true;
  }

  void HandleAgentMessage(SpiceAgentMessage* message) {
    switch (message->type)
    {
    case kAgentMessageAnnounceCapabilities: {
      /* ui effects & color depth */
      uint8_t display_config[8] = { 0 };
      SendAgentMessage(kVdiClientPort, kAgentMessageDisplayConfig, display_config, sizeof(display_config));
      uint32_t max_clipboard = 0x06400000;
      SendAgentMessage(kVdiClientPort, kAgentMessageMaxClipboard, &max_clipboard, sizeof(max_clipboard));
      if (pending_resize_event_) {
        Resize(width_, height_);
      }
      break;
    }
    default:
      MV_LOG("Unhandled agent message type=0x%x", message->type);
      DumpHex(message, sizeof(*message) + message->size);
      break;
    }
  }

  void SendAgentMessage(VdiSourcePort port, SpiceAgentMessageType type, void* data, size_t length) {
    size_t buffer_size = sizeof(VdiChunkHeader) + sizeof(SpiceAgentMessage) + length;
    uint8_t* buffer = new uint8_t[buffer_size];
    VdiChunkHeader* chunk_header = (VdiChunkHeader*)buffer;
    chunk_header->port = port;
    chunk_header->size = sizeof(SpiceAgentMessage) + length;
    SpiceAgentMessage* agent_message = (SpiceAgentMessage*)(buffer + sizeof(VdiChunkHeader));
    agent_message->type = type;
    agent_message->protocol = 1;
    agent_message->opaque = 0UL;
    agent_message->size = length;
    memcpy(agent_message->data, data, length);
    console_->SendPortMessage(this, buffer, buffer_size);
    delete buffer;
  }

  bool CanAcceptInput() {
    return guest_connected_;
  }

  void QueuePointerEvent(uint32_t buttons, uint32_t x, uint32_t y) {
    if (!guest_connected_) {
      return;
    }
    steady_clock::time_point now = steady_clock::now();
    if (last_mouse_state_.buttons == buttons && now - last_send_mouse_time_ < kSendMouseInterval) {
      return;
    }
    last_mouse_state_.display_id = 0;
    last_mouse_state_.buttons = buttons;
    last_mouse_state_.x = x;
    last_mouse_state_.y = y;
    last_send_mouse_time_ = now;
    SendAgentMessage(kVdiServerPort, kAgentMessageMouseState, &last_mouse_state_, sizeof(last_mouse_state_));
  }

  void Resize(uint32_t width, uint32_t height) {
    width_ = width;
    height_ = height;
    if (!guest_connected_) {
      pending_resize_event_ = true;
      return;
    }
    MV_LOG("Resize %ux%u", width_, height_);

    SpiceAgentMonitorsConfig config = { 0 };
    config.num_of_monitors = 1;
    config.monitors[0].depth = 32;
    config.monitors[0].width = width_;
    config.monitors[0].height = height_;
    SendAgentMessage(kVdiClientPort, kAgentMessageMonitorsConfig, &config, sizeof(config));
  }
};

DECLARE_DEVICE(SpiceAgent);
