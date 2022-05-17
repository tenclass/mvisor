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

/*
 * Reference: https://www.spice-space.org/agent-protocol.html
 */

#include <cstring>
#include <chrono>
#include <cstdlib>

#include "object.h"
#include "utilities.h"
#include "logger.h"
#include "device_interface.h"
#include "spice/vd_agent.h"
#include "spice/enums.h"

using namespace std::chrono;

/* Limit send mouse frequency */
const auto kSendMouseInterval = milliseconds(20);

class SpiceAgent : public Object, public SerialPortInterface,
  public DisplayResizeInterface, public PointerInputInterface,
  public ClipboardInterface
{
 private:
  bool                            pending_resize_event_;
  VDAgentMouseState               last_mouse_state_;
  steady_clock::time_point        last_send_mouse_time_;
  uint32_t                        width_, height_;
  int                             num_monitors_;
  std::string                     last_send_clipboard_;
  std::vector<ClipboardListener>  clipboard_listeners_;
  bool                            clipboard_enabled_;

 public:
  SpiceAgent() {
    set_parent_name("virtio-console");
  
    strcpy(port_name_, "com.redhat.spice.0");
    pending_resize_event_ = false;
    last_send_mouse_time_ = steady_clock::now();
    num_monitors_ = 1;
    clipboard_enabled_ = false;
  }

  virtual void OnMessage(uint8_t* data, size_t size) {
    if (size < sizeof(VDIChunkHeader) + sizeof(VDAgentMessage)) {
      MV_PANIC("Chunk too small, size=0x%lx", size);
      return;
    }
    VDIChunkHeader* chunk_header = (VDIChunkHeader*)data;
    if (chunk_header->size + sizeof(VDIChunkHeader) != size) {
      MV_PANIC("Invalid chunk size=0x%lx", size);
      return;
    }
    VDAgentMessage* message = (VDAgentMessage*)(data + sizeof(VDIChunkHeader));
    HandleAgentMessage(message);
  }

  void HandleAgentMessage(VDAgentMessage* message) {
    switch (message->type)
    {
    case VD_AGENT_ANNOUNCE_CAPABILITIES: {
      /* ui effects & color depth */
      VDAgentDisplayConfig display_config = {
        .flags = 0
      };
      SendAgentMessage(VDP_CLIENT_PORT, VD_AGENT_DISPLAY_CONFIG, &display_config, sizeof(display_config));

      uint32_t max_clipboard = 0x06400000;
      SendAgentMessage(VDP_CLIENT_PORT, VD_AGENT_MAX_CLIPBOARD, &max_clipboard, sizeof(max_clipboard));

      if (pending_resize_event_) {
        Resize(width_, height_);
      }

      if(!clipboard_enabled_) { 
        SendAgentCapabilities();
      }
      break;
    }
    case VD_AGENT_REPLY: {
      /* Handle reply of monitor config and display config */ 
      auto reply = (VDAgentReply*)message->data;
      if (reply->error != VD_AGENT_SUCCESS) {
        MV_LOG("agent reply type=%u error=%u", reply->type, reply->error);
      }
      break;
    }
    case VD_AGENT_CLIPBOARD_GRAB: {
      auto grab = (VDAgentClipboardGrab*)message->data; 
      VDAgentClipboardRequest request = {
        .type = grab->types[0]
      };
      SendAgentMessage(VDP_SERVER_PORT, VD_AGENT_CLIPBOARD_REQUEST, &request, sizeof(request));
      break;
    }
    case VD_AGENT_CLIPBOARD: {
      auto clipboard = (VDAgentClipboard*) message->data;
      uint32_t msg_size = message->size - sizeof(message->type);
      NotifyClipboardEvent(clipboard, msg_size);
      break;
    }
    case VD_AGENT_CLIPBOARD_REQUEST:
      SendAgentMessage(VDP_SERVER_PORT, VD_AGENT_CLIPBOARD, last_send_clipboard_.data(), last_send_clipboard_.size());
      break;
    default:
      MV_LOG("Unhandled agent message type=0x%x", message->type);
      DumpHex(message, sizeof(*message) + message->size);
      break;
    }
  }

  void SendAgentMessage(int port, int type, void* data, size_t length) {
    size_t buffer_size = sizeof(VDIChunkHeader) + sizeof(VDAgentMessage) + length;
    uint8_t* buffer = new uint8_t[buffer_size];
    VDIChunkHeader* chunk_header = (VDIChunkHeader*)buffer;
    chunk_header->port = port;
    chunk_header->size = sizeof(VDAgentMessage) + length;
    VDAgentMessage* agent_message = (VDAgentMessage*)(buffer + sizeof(VDIChunkHeader));
    agent_message->type = type;
    agent_message->protocol = 1;
    agent_message->opaque = 0UL;
    agent_message->size = length;
    memcpy(agent_message->data, data, length);
    device_->SendMessage(this, buffer, buffer_size);
    delete buffer;
  }

  void QueueEvent(uint buttons, int x, int y) {
    steady_clock::time_point now = steady_clock::now();
    if (last_mouse_state_.buttons == buttons && now - last_send_mouse_time_ < kSendMouseInterval) {
      return;
    }
    last_mouse_state_.display_id = 0;
    last_mouse_state_.buttons = buttons;
    last_mouse_state_.x = x;
    last_mouse_state_.y = y;
    last_send_mouse_time_ = now;
    SendAgentMessage(VDP_SERVER_PORT, VD_AGENT_MOUSE_STATE, &last_mouse_state_, sizeof(last_mouse_state_));
  }

  virtual bool InputAcceptable() {
    return ready_;
  }

  virtual bool QueuePointerEvent(PointerEvent event) {
    if (!ready_) {
      return false;
    }
    if (event.z > 0) {
      QueueEvent(event.buttons | (1 << SPICE_MOUSE_BUTTON_UP), event.x, event.y);
      QueueEvent(event.buttons, event.x, event.y);
    } else if (event.z < 0) {
      QueueEvent(event.buttons | (1 << SPICE_MOUSE_BUTTON_DOWN), event.x, event.y);
      QueueEvent(event.buttons, event.x, event.y);
    } else {
      QueueEvent(event.buttons, event.x, event.y);
    }
    return true;
  }

  virtual bool Resize(uint32_t width, uint32_t height) {
    /* For H264, resolution must be multiple of 2 */
    if (width & 1)
      width++;
    if (height & 1)
      height++;

    width_ = width;
    height_ = height;
    if (!ready_) {
      pending_resize_event_ = true;
      return false;
    }

    size_t config_size = sizeof(VDAgentMonitorsConfig) + sizeof(VDAgentMonConfig);
    VDAgentMonitorsConfig* config = (VDAgentMonitorsConfig*)malloc(config_size);
    bzero(config, config_size);
    config->num_of_monitors = 1;
    config->monitors[0].depth = 32;
    config->monitors[0].width = width_;
    config->monitors[0].height = height_;
    SendAgentMessage(VDP_CLIENT_PORT, VD_AGENT_MONITORS_CONFIG, config, config_size);
    free(config);
    return true;
  }

  void SendAgentCapabilities() {
    size_t size = sizeof(VDAgentAnnounceCapabilities) + VD_AGENT_CAPS_BYTES;
    VDAgentAnnounceCapabilities * caps = (VDAgentAnnounceCapabilities*)malloc(size);
    caps->request = true;
    VD_AGENT_SET_CAPABILITY(caps->caps, VD_AGENT_CAP_MOUSE_STATE);
    VD_AGENT_SET_CAPABILITY(caps->caps, VD_AGENT_CAP_MONITORS_CONFIG);
    VD_AGENT_SET_CAPABILITY(caps->caps, VD_AGENT_CAP_REPLY);
    VD_AGENT_SET_CAPABILITY(caps->caps, VD_AGENT_CAP_DISPLAY_CONFIG);
    VD_AGENT_SET_CAPABILITY(caps->caps, VD_AGENT_CAP_CLIPBOARD_BY_DEMAND);
    VD_AGENT_SET_CAPABILITY(caps->caps, VD_AGENT_CAP_CLIPBOARD_SELECTION);
    VD_AGENT_SET_CAPABILITY(caps->caps, VD_AGENT_CAP_MONITORS_CONFIG_POSITION);
    VD_AGENT_SET_CAPABILITY(caps->caps, VD_AGENT_CAP_FILE_XFER_DETAILED_ERRORS);
    VD_AGENT_SET_CAPABILITY(caps->caps, VD_AGENT_CAP_CLIPBOARD_NO_RELEASE_ON_REGRAB);
    VD_AGENT_SET_CAPABILITY(caps->caps, VD_AGENT_CAP_CLIPBOARD_GRAB_SERIAL);
    SendAgentMessage(VDP_SERVER_PORT, VD_AGENT_ANNOUNCE_CAPABILITIES, caps, size);
    free(caps);
    clipboard_enabled_ = true;
  }

  void SendAgentClipboardGrab(uint type) {
    size_t size = sizeof(VDAgentClipboardGrab) + sizeof(uint32_t)* 2;
    VDAgentClipboardGrab* grab = (VDAgentClipboardGrab *) malloc(size);
    grab->types[0] = VD_AGENT_CLIPBOARD_UTF8_TEXT;
    SendAgentMessage(VDP_SERVER_PORT, VD_AGENT_CLIPBOARD_GRAB, grab, size);
    free(grab);
  }

  void SetAgentClipboardData(uint type, uint32_t msg_size, void* msg_data) {
    size_t size = sizeof(VDAgentClipboard) + msg_size;
    last_send_clipboard_.resize(size);  
    VDAgentClipboard* clipboard = (VDAgentClipboard*) last_send_clipboard_.data();
    clipboard->type = type;
    memcpy(clipboard->data, msg_data, msg_size);
  }

  virtual bool ClipboardDataToGuest(uint type, uint32_t msg_size, void* msg_data) {
    SendAgentCapabilities();

    SetAgentClipboardData(type, msg_size, msg_data);

    SendAgentClipboardGrab(type);

    return true;
  }

  void NotifyClipboardEvent(VDAgentClipboard* clipboard, uint32_t msg_size) {
    for (auto& cb : clipboard_listeners_) {
      cb(ClipboardData {
        .type = clipboard->type,
        .msg_size = msg_size,
        .msg_data = clipboard->data
      });
    }
  }

  void RegisterClipboardListener(ClipboardListener callback) {
    clipboard_listeners_.push_back(callback);
  }
};

DECLARE_AGENT(SpiceAgent);
