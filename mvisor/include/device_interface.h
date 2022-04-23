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
#include <string>
#include <deque>
#include <sys/uio.h>

class KeyboardInputInterface {
 public:
  virtual bool QueueKeyboardEvent(uint8_t scancode[10], uint8_t modifiers) = 0;
  virtual bool QueueMouseEvent(uint buttons, int rel_x, int rel_y, int rel_z) = 0;
  virtual bool InputAcceptable() = 0;
};

struct PointerEvent {
  uint  buttons; 
  int   x;
  int   y;
  int   z;
  uint  screen_width;
  uint  screen_height;
};
class PointerInputInterface {
 public:
  virtual bool QueuePointerEvent(PointerEvent event) = 0;
  virtual bool InputAcceptable() = 0;
};

class DisplayResizeInterface {
 public:
  virtual bool Resize(uint width, uint height) = 0;
};


struct DisplayPartialBitmap {
  std::vector<iovec>  vector;
  uint                stride;
  uint                bpp;
  uint                width;
  uint                height;
  uint                x;
  uint                y;
  bool                flip;
  const uint8_t*      pallete;
};
struct DisplayMouseCursor {
  uint8_t       visible;
  uint          x;
  uint          y;
  uint64_t      update_timestamp;
  struct {
    uint64_t    id;
    uint16_t    type;
    uint16_t    width;
    uint16_t    height;
    uint16_t    hotspot_x;
    uint16_t    hotspot_y;
    std::string data;
  } shape;
};
struct DisplayUpdate {
  std::vector<DisplayPartialBitmap>  partials;
  DisplayMouseCursor                 cursor;
};

typedef std::function <void(void)> DisplayModeChangeListener;
typedef std::function <void(void)> DisplayUpdateListener;
class DisplayInterface {
 public:
  virtual void GetDisplayMode(uint* w, uint* h, uint* bpp, uint* stride) = 0;
  virtual const uint8_t* GetPallete() const = 0;
  virtual void Redraw() = 0;
  virtual bool AcquireUpdate(DisplayUpdate& update) = 0;
  virtual void ReleaseUpdate() = 0;
  virtual void RegisterDisplayModeChangeListener(DisplayModeChangeListener callback) = 0;
  virtual void RegisterDisplayUpdateListener(DisplayUpdateListener callback) = 0;
};


struct PlaybackFormat {
  uint format;
  uint channels;
  uint frequency;
  uint interval_ms;
};

enum PlaybackState {
  kPlaybackStart,
  kPlaybackStop,
  kPlaybackData
};

typedef std::function <void(PlaybackState state, struct iovec iov)> PlaybackListener;
class PlaybackInterface {
 public:
  virtual void GetPlaybackFormat(uint* format, uint* channels, uint* frequency, uint* interval_ms) = 0;
  virtual void RegisterPlaybackListener(PlaybackListener callback) = 0;
};


class SerialPortInterface;
class SerialDeviceInterface {
 public:
  virtual void SendMessage(SerialPortInterface* port, uint8_t* data, size_t size) = 0;
};

class SerialPortInterface {
 public:
  virtual void OnMessage(uint8_t* data, size_t size) {
    if (callback_)
      callback_(data, size);
  }
  virtual void OnWritable() {
    writable_ = true;
  }
  virtual void SendMessage(uint8_t* data, size_t size) {
    device_->SendMessage(this, data, size);
  }

  void Initialize(SerialDeviceInterface* device, uint32_t id) {
    device_ = device;
    port_id_ = id;
  }

  virtual void set_ready(bool ready) {
    ready_ = ready;
  }

  virtual void set_callback(std::function<void(uint8_t*, size_t)> callback) {
    callback_ = callback;
  }

  inline uint32_t     port_id() const { return port_id_; }
  inline const char*  port_name() const { return port_name_; }
  inline bool         ready() const { return ready_; }

 protected:
  SerialDeviceInterface* device_;
  std::function<void(uint8_t*, size_t)> callback_;
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
  virtual void Initialize(NetworkDeviceInterface* device, MacAddress& mac, int mtu) = 0;
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


class PowerDownInterface {
 public:
  virtual void PowerDown() = 0;
};

#endif // _MVISOR_DEVICE_INTERFACES_H
