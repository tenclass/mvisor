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

#include "device.h"
#include "device_manager.h"
#include "device_interface.h"
#include "spice/vd_agent.h"
#include "spice/enums.h"
#include "utilities.h"
#include "logger.h"

using namespace std::chrono;

class SpiceAgent : public Device, public SerialPortInterface,
  public DisplayResizeInterface, public PointerInputInterface,
  public ClipboardInterface
{
 private:
  int                             num_monitors_ = 1;
  /* Limit mouse event frequency */
  VDAgentMouseState               last_mouse_state_;
  /* Cache clipboard data */
  std::string                     outgoing_clipboard_;
  std::vector<ClipboardListener>  clipboard_listeners_;
  /* Buffer to build incoming message */
  uint                            incoming_message_written_ = 0;
  std::string                     incoming_message_;
  /* Max clipboard size, default is 1MB */
  uint32_t                        max_clipboard_ = 1024 * 1024;

 public:
  SpiceAgent() {
    set_parent_name("virtio-console");
  
    strcpy(port_name_, "com.redhat.spice.0");
  }

  /* The message may consists of more than one chunk */
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
    auto chunk_data = data + sizeof(VDIChunkHeader);

    if (incoming_message_written_ == 0) {
      VDAgentMessage* message = (VDAgentMessage*)chunk_data;
      incoming_message_.resize(sizeof(VDAgentMessage) + message->size);
    }
    if (incoming_message_written_ + chunk_header->size > incoming_message_.size()) {
      MV_PANIC("incoming message buffer overflow message=%lu chunk=%u written=%u",
        incoming_message_.size(), chunk_header->size, incoming_message_written_);
    }

    memcpy(incoming_message_.data() + incoming_message_written_, chunk_data, chunk_header->size);
    incoming_message_written_ += chunk_header->size;
    if (incoming_message_written_ == incoming_message_.size()) {
      HandleAgentMessage((VDAgentMessage*)incoming_message_.data());
      incoming_message_written_ = 0;
    }
  }

  void HandleAgentMessage(VDAgentMessage* message) {
    switch (message->type)
    {
    case VD_AGENT_ANNOUNCE_CAPABILITIES: {
      /* This is the first message from VDAgent */
      auto announce_caps = (VDAgentAnnounceCapabilities*)message->data;
      auto caps_size = message->size - sizeof(announce_caps->request);
      auto caps = announce_caps->caps;
      if (announce_caps->request) {
        SendAgentCapabilities();
      }

      if (VD_AGENT_HAS_CAPABILITY(caps, caps_size, VD_AGENT_CAP_MAX_CLIPBOARD)) {
        if (has_key("max_clipboard")) {
          /* Read max_clipboard from configuration */
          max_clipboard_ = (uint32_t)std::get<uint64_t>(key_values_["max_clipboard"]);
        }
        SendAgentMessage(VDP_CLIENT_PORT, VD_AGENT_MAX_CLIPBOARD, &max_clipboard_, sizeof(max_clipboard_));
      }

      if (VD_AGENT_HAS_CAPABILITY(caps, caps_size, VD_AGENT_CAP_DISPLAY_CONFIG)) {
        /* ui effects & color depth */
        VDAgentDisplayConfig display_config = {
          .flags = 0,
          .depth = 0
        };
        SendAgentMessage(VDP_CLIENT_PORT, VD_AGENT_DISPLAY_CONFIG, &display_config, sizeof(display_config));
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
      SendAgentMessage(VDP_CLIENT_PORT, VD_AGENT_CLIPBOARD_REQUEST, &request, sizeof(request));
      break;
    }
    case VD_AGENT_CLIPBOARD: {
      auto clipboard = (VDAgentClipboard*)message->data;
      auto clipboard_data = ClipboardData {
        .type = clipboard->type,
        .data = std::string((const char*)clipboard->data, message->size - sizeof(VDAgentClipboard)),
        .file_name = ""
      };
      NotifyClipboardEvent(clipboard_data);
      break;
    }
    case VD_AGENT_CLIPBOARD_REQUEST:
      SendAgentMessage(VDP_CLIENT_PORT, VD_AGENT_CLIPBOARD, outgoing_clipboard_.data(), outgoing_clipboard_.size());
      break;
    case VD_AGENT_CLIPBOARD_RELEASE: {
      /* VM asks us to clear the clipboard */
      auto clipboard_data = ClipboardData {
        .type = VD_AGENT_CLIPBOARD_NONE,
        .data = "",
        .file_name = ""
      };
      NotifyClipboardEvent(clipboard_data);
      break;
    }
    default:
      MV_ERROR("Unhandled agent message type=0x%x size=%d", message->type, message->size);
      break;
    }
  }

  void SendMessage(int port, const std::string& message) {
    std::string buffer;
    size_t bytes_sent = 0;
    while (bytes_sent < message.size()) {
      size_t chunk_size = message.size() - bytes_sent;
      if (chunk_size > VD_AGENT_MAX_DATA_SIZE - sizeof(VDIChunkHeader)) {
        chunk_size = VD_AGENT_MAX_DATA_SIZE - sizeof(VDIChunkHeader);
      }
      buffer.resize(sizeof(VDIChunkHeader) + chunk_size);
      auto chunk_header = (VDIChunkHeader*)buffer.data();
      chunk_header->port = port;
      chunk_header->size = chunk_size;
      memcpy(buffer.data() + sizeof(VDIChunkHeader), message.data() + bytes_sent, chunk_size);
      bytes_sent += chunk_size;
      device_->SendMessage(this, (uint8_t*)buffer.data(), buffer.size());
    }
  }

  void SendAgentMessage(int port, int type, void* data, size_t length) {
    std::string outgoing_message;
    outgoing_message.resize(sizeof(VDAgentMessage) + length);
    VDAgentMessage* agent_message = (VDAgentMessage*)outgoing_message.data();
    agent_message->type = type;
    agent_message->protocol = 1;
    agent_message->opaque = 0UL;
    agent_message->size = length;
    memcpy(agent_message->data, data, length);
    SendMessage(port, outgoing_message);
  }

  void QueueEvent(uint buttons, int x, int y) {
    if (last_mouse_state_.buttons == buttons && last_mouse_state_.x == (uint)x && last_mouse_state_.y == (uint)y) {
      return;
    }

    /* This is a hack, flip the last bit to avoid guest frequency limitation */
    if (!(last_mouse_state_.buttons & (1 << 31))) {
      buttons |= 1 << 31;
    }

    last_mouse_state_.display_id = 0;
    last_mouse_state_.buttons = buttons;
    last_mouse_state_.x = x;
    last_mouse_state_.y = y;
    SendAgentMessage(VDP_SERVER_PORT, VD_AGENT_MOUSE_STATE, &last_mouse_state_, sizeof(last_mouse_state_));
  }

  virtual bool InputAcceptable() {
    return ready_;
  }

  /* This interface function is called by UI thread */
  virtual bool QueuePointerEvent(PointerEvent event) {
    if (!ready_) {
      return false;
    }

    manager_->io()->Schedule([this, event]() {
      if (event.z > 0) {
        QueueEvent(event.buttons | (1 << SPICE_MOUSE_BUTTON_UP), event.x, event.y);
        QueueEvent(event.buttons, event.x, event.y);
      } else if (event.z < 0) {
        QueueEvent(event.buttons | (1 << SPICE_MOUSE_BUTTON_DOWN), event.x, event.y);
        QueueEvent(event.buttons, event.x, event.y);
      } else {
        QueueEvent(event.buttons, event.x, event.y);
      }
    });
    return true;
  }

  /* This interface function is called by UI thread */
  virtual bool Resize(uint32_t width, uint32_t height) {
    if (!ready_) {
      return false;
    }
    /* For H264, resolution must be multiple of 2 */
    if (width & 1)
      width++;
    if (height & 1)
      height++;

    manager_->io()->Schedule([=]() {
      auto buffer = std::string(sizeof(VDAgentMonitorsConfig) + sizeof(VDAgentMonConfig), '\0');
      auto config = (VDAgentMonitorsConfig*)buffer.data();
      config->num_of_monitors = 1;
      config->monitors[0].depth = 32;
      config->monitors[0].width = width;
      config->monitors[0].height = height;
      SendAgentMessage(VDP_CLIENT_PORT, VD_AGENT_MONITORS_CONFIG, buffer.data(), buffer.size());
    });
    return true;
  }

  void SendAgentCapabilities() {
    auto buffer = std::string(sizeof(uint32_t) + VD_AGENT_CAPS_BYTES, '\0');
    auto caps = (VDAgentAnnounceCapabilities*)buffer.data();
    caps->request = false;
    VD_AGENT_SET_CAPABILITY(caps->caps, VD_AGENT_CAP_MOUSE_STATE);
    VD_AGENT_SET_CAPABILITY(caps->caps, VD_AGENT_CAP_MONITORS_CONFIG);
    VD_AGENT_SET_CAPABILITY(caps->caps, VD_AGENT_CAP_SPARSE_MONITORS_CONFIG);
    VD_AGENT_SET_CAPABILITY(caps->caps, VD_AGENT_CAP_REPLY);
    VD_AGENT_SET_CAPABILITY(caps->caps, VD_AGENT_CAP_DISPLAY_CONFIG);
    VD_AGENT_SET_CAPABILITY(caps->caps, VD_AGENT_CAP_CLIPBOARD_BY_DEMAND);
    SendAgentMessage(VDP_SERVER_PORT, VD_AGENT_ANNOUNCE_CAPABILITIES, buffer.data(), buffer.size());
  }

  void SendAgentClipboardGrab(uint type) {
    if (type == VD_AGENT_CLIPBOARD_UTF8_TEXT) {
      auto buffer = std::string(sizeof(VDAgentClipboardGrab), '\0');
      auto grab = (VDAgentClipboardGrab*)buffer.data();
      grab->types[0] = VD_AGENT_CLIPBOARD_UTF8_TEXT;
      SendAgentMessage(VDP_CLIENT_PORT, VD_AGENT_CLIPBOARD_GRAB, buffer.data(), buffer.size());
    } else {
      MV_LOG("unsupported clipboard type=0x%x", type);
    }
  }

  /* This interface function is called by UI thread */
  virtual bool ClipboardDataToGuest(uint type, const std::string& data) {
    if (!ready_) {
      return false;
    }
    outgoing_clipboard_.resize(sizeof(VDAgentClipboard) + data.size());  
    auto clipboard = (VDAgentClipboard*)outgoing_clipboard_.data();
    clipboard->type = type;
    memcpy(clipboard->data, data.data(), data.size());

    manager_->io()->Schedule([this, type]() {
      SendAgentClipboardGrab(type);
    });
    return true;
  }

  void NotifyClipboardEvent(const ClipboardData& data) {
    for (auto& cb : clipboard_listeners_) {
      cb(data);
    }
  }

  void RegisterClipboardListener(ClipboardListener callback) {
    clipboard_listeners_.push_back(callback);
  }
};

DECLARE_DEVICE(SpiceAgent);
