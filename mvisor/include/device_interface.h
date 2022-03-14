/* 
 * MVisor
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

#ifndef _MVISOR_DEVICE_INTERFACES_H
#define _MVISOR_DEVICE_INTERFACES_H

#include <functional>
#include <vector>
#include <cstring>
#include <deque>
#include <sys/uio.h>

class KeyboardInputInterface {
 public:
  virtual void QueueKeyboardEvent(uint8_t scancode[10]) = 0;
  virtual void QueueMouseEvent(uint buttons, int rel_x, int rel_y, int rel_z) = 0;
  virtual bool InputAcceptable() = 0;
};

struct PointerEvent {
  uint buttons; 
  int x;
  int y;
  int z;
  uint screen_width;
  uint screen_height;
};
class PointerInputInterface {
 public:
  virtual void QueuePointerEvent(PointerEvent event) = 0;
  virtual bool InputAcceptable() = 0;
};

class SpiceAgentInterface {
 public:
  virtual void Resize(uint32_t width, uint32_t height) = 0;
};


enum CursorUpdateCommand {
  kDisplayCursorUpdateHide,
  kDisplayCursorUpdateSet,
  kDisplayCursorUpdateMove
};
typedef std::function <void()> ReleaseDisplayResourceCallback;
struct DisplayPartialData {
  uint8_t*    data;
  size_t      size;
};
struct DisplayPartialBitmap {
  std::vector<DisplayPartialData> vector;
  int         stride;
  int         width;
  int         height;
  int         x;
  int         y;
  bool        flip;
  ReleaseDisplayResourceCallback  Release;
};
struct DisplayCursorUpdate {
  CursorUpdateCommand   command;
  union {
    struct {
      uint16_t x;
      uint16_t y;
    } move;
    struct {
      uint8_t   visible;
      int       x;
      int       y;
      uint16_t  type;
      uint16_t  width;
      uint16_t  height;
      uint16_t  hotspot_x;
      uint16_t  hotspot_y;
      uint8_t*  data;
      size_t    size;
    } set;
  };
  ReleaseDisplayResourceCallback Release;
};

typedef std::function <void(void)> DisplayChangeListener;
typedef std::function <void(const DisplayPartialBitmap*)> DisplayRenderCallback;
typedef std::function <void(const DisplayCursorUpdate*)> DisplayCursorUpdateCallback;
class DisplayInterface {
 public:
  virtual void GetDisplayMode(uint16_t* w, uint16_t* h, uint16_t* bpp) = 0;
  virtual const uint8_t* GetPallete() const = 0;
  virtual void RegisterDisplayChangeListener(DisplayChangeListener callback) = 0;
  virtual void RegisterDisplayRenderer(DisplayRenderCallback draw_callback,
    DisplayCursorUpdateCallback cursor_callback) = 0;
};



class SerialPortInterface;
class SerialDeviceInterface {
 public:
  virtual void SendMessage(SerialPortInterface* port, uint8_t* data, size_t size) = 0;
};

class SerialPortInterface {
 public:
  virtual void OnMessage(uint8_t* data, size_t size) = 0;
  virtual void OnWritable() = 0;

  void Initialize(SerialDeviceInterface* device, uint32_t id) {
    device_ = device;
    port_id_ = id;
  }

  virtual void set_ready(bool ready) {
    ready_ = ready;
  }

  inline uint32_t     port_id() const { return port_id_; }
  inline const char*  port_name() const { return port_name_; }
  inline bool         ready() const { return ready_; }

 protected:
  SerialDeviceInterface* device_;
  uint32_t  port_id_;
  char      port_name_[100];
  bool      ready_ = false;
  bool      writable_ = false;
};


struct MacAddress {
  union {
    uint8_t   data[6];
    uint64_t  value : 48;
  };
  bool operator < (const MacAddress& a) const {
    return memcmp(data, a.data, 6) < 0;
  }
};
class NetworkDeviceInterface {
 public:
  virtual bool WriteBuffer(void* buffer, size_t size) = 0;
};
struct Ipv4Packet;
class NetworkBackendInterface {
 public:
  virtual void Initialize(NetworkDeviceInterface* device, MacAddress& mac) = 0;
  virtual void Reset() = 0;
  virtual void OnFrameFromGuest(std::deque<iovec>& vector) = 0;
  virtual bool OnPacketFromHost(Ipv4Packet* packet) = 0;
  virtual Ipv4Packet* AllocatePacket(bool urgent) = 0;
  virtual void OnReceiveAvailable() = 0;

  NetworkDeviceInterface* device() { return device_; }
 protected:
  NetworkDeviceInterface* device_;
  MacAddress guest_mac_;
};

#endif // _MVISOR_DEVICE_INTERFACES_H
