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
  public DisplayResizeInterface, public PointerInputInterface
{
 private:
  bool                      pending_resize_event_;
  VDAgentMouseState         last_mouse_state_;
  steady_clock::time_point  last_send_mouse_time_;
  uint32_t                  width_, height_;
  int                       num_monitors_;

 public:
  SpiceAgent() {
    set_parent_name("virtio-console");
  
    strcpy(port_name_, "com.redhat.spice.0");
    pending_resize_event_ = false;
    last_send_mouse_time_ = steady_clock::now();
    num_monitors_ = 1;
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

  void SendMonitorConfig() {
    size_t buffer_size = sizeof(VDAgentMonitorsConfig) + sizeof(VDAgentMonConfig) * num_monitors_;
    uint8_t buffer[buffer_size] = { 0 };
    auto config = (VDAgentMonitorsConfig*)buffer;
    config->num_of_monitors = num_monitors_;
    config->flags = VD_AGENT_CONFIG_MONITORS_FLAG_PHYSICAL_SIZE;
  
    for (int i = 0; i < num_monitors_; i++) {
      auto &mon = config->monitors[i];
      mon.depth = 32;
      mon.width = 1024;
      mon.height = 768;
    }
    SendAgentMessage(VDP_CLIENT_PORT, VD_AGENT_MONITORS_CONFIG, buffer, buffer_size);
  }

  void HandleAgentMessage(VDAgentMessage* message) {
    switch (message->type)
    {
    case VD_AGENT_ANNOUNCE_CAPABILITIES: {
      /* control the initial resolution when OS started */
      // SendMonitorConfig();

      /* ui effects & color depth */
      VDAgentDisplayConfig display_config = {
        .flags = VD_AGENT_DISPLAY_CONFIG_FLAG_DISABLE_ANIMATION
      };
      SendAgentMessage(VDP_CLIENT_PORT, VD_AGENT_DISPLAY_CONFIG, &display_config, sizeof(display_config));

      uint32_t max_clipboard = 0x06400000;
      SendAgentMessage(VDP_CLIENT_PORT, VD_AGENT_MAX_CLIPBOARD, &max_clipboard, sizeof(max_clipboard));

      if (pending_resize_event_) {
        Resize(width_, height_);
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
};

DECLARE_AGENT(SpiceAgent);
