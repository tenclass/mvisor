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
#include <list>
#include <cstring>
#include <string>
#include <deque>
#include <sys/uio.h>
#include <cstdint>

class KeyboardInputInterface {
 public:
  virtual ~KeyboardInputInterface() = default;
  virtual bool QueueKeyboardEvent(uint8_t scancode[10], uint8_t modifiers) = 0;
  virtual bool QueueMouseEvent(uint button_state, int rel_x, int rel_y, int rel_z) = 0;
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
  virtual ~PointerInputInterface() = default;
  virtual bool QueuePointerEvent(PointerEvent event) = 0;
  virtual bool InputAcceptable() = 0;
};

struct MidiEvent {
  uint8_t cable_code_index;
  uint8_t midi_0;
  uint8_t midi_1;
  uint8_t midi_2;
};

class MidiInputInterface {
 public:
  virtual ~MidiInputInterface() = default;
  virtual bool QueueMidiEvent(MidiEvent event) = 0;
  virtual bool InputAcceptable() = 0;
  virtual void Start() = 0;
  virtual void Stop() = 0;
};

struct WacomEvent {
  double x;
  double y;
  double pressure;
  uint32_t buttons;
  uint32_t tilt_x;
  uint32_t tilt_y;
};

class WacomInputInterface {
 public:
  virtual ~WacomInputInterface() = default;
  virtual bool QueueWacomEvent(WacomEvent event) = 0;
  virtual bool InputAcceptable() = 0;
  virtual void Start() = 0;
  virtual void Stop() = 0;
};

class DisplayResizeInterface {
 public:
  virtual ~DisplayResizeInterface() = default;
  virtual bool Resize(int width, int height) = 0;
};

typedef std::function <void()> VirtioFsListener;
class VirtioFsInterface {
 public:
  virtual ~VirtioFsInterface() = default;
  virtual std::list<VirtioFsListener>::iterator RegisterVirtioFsListener(VirtioFsListener callback) = 0;
  virtual void UnregisterVirtioFsListener(std::list<VirtioFsListener>::iterator it) = 0;
};

struct ClipboardData {
  uint32_t        type;
  std::string     data;
  std::string     file_name;
};

typedef std::function <void(const ClipboardData clipboard_data)> ClipboardListener;
class ClipboardInterface {
 public:
  virtual ~ClipboardInterface() = default;
  virtual std::list<ClipboardListener>::iterator RegisterClipboardListener(ClipboardListener callback) = 0;
  virtual void UnregisterClipboardListener(std::list<ClipboardListener>::iterator it) = 0;
  virtual bool ClipboardDataToGuest(uint type, const std::string& data) = 0;
};


struct DisplayPartialBitmap {
  int           bpp;
  int           width;
  int           height;
  int           stride;
  int           x;
  int           y;
  uint8_t*      data;
  uint8_t*      palette;
};
struct DisplayMouseCursor {
  uint8_t       visible;
  int           x;
  int           y;
  uint64_t      update_timestamp;
  struct {
    uint64_t    id;
    int16_t     type;
    int16_t     width;
    int16_t     height;
    int16_t     hotspot_x;
    int16_t     hotspot_y;
    std::string data;
  } shape;
};
struct DisplayUpdate {
  std::vector<DisplayPartialBitmap>  partials;
  DisplayMouseCursor                 cursor;
};

typedef std::function <void(void)> DisplayModeChangeListener;
typedef std::function <void(const DisplayUpdate&)> DisplayUpdateListener;
class DisplayInterface {
 public:
  virtual ~DisplayInterface() = default;
  virtual void GetDisplayMode(int* w, int* h, int* bpp, int* stride) = 0;
  virtual void GetPalette(const uint8_t** palette, int* count, bool* dac_8bit) = 0;
  virtual void Refresh() = 0;
  virtual std::list<DisplayModeChangeListener>::iterator RegisterDisplayModeChangeListener(DisplayModeChangeListener callback) = 0;
  virtual void UnregisterDisplayModeChangeListener(std::list<DisplayModeChangeListener>::iterator it) = 0;
  virtual std::list<DisplayUpdateListener>::iterator RegisterDisplayUpdateListener(DisplayUpdateListener callback) = 0;
  virtual void UnregisterDisplayUpdateListener(std::list<DisplayUpdateListener>::iterator it) = 0;
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
  virtual ~PlaybackInterface() = default;
  virtual void GetPlaybackFormat(uint* format, uint* channels, uint* frequency, uint* interval_ms) = 0;
  virtual std::list<PlaybackListener>::iterator RegisterPlaybackListener(PlaybackListener callback) = 0;
  virtual void UnregisterPlaybackListener(std::list<PlaybackListener>::iterator it) = 0;
};

struct RecordFormat {
  int frequency = 48000;
  int channels = 2;
};

enum RecordState {
  kRecordStart,
  kRecordStop
};

typedef std::function <void(RecordState state)>   RecordListener;
class RecordInterface {
  public:
  virtual ~RecordInterface() = default;
  virtual void WriteRecordDataToDevice(const std::string& record_data) = 0;
  virtual std::list<RecordListener>::iterator RegisterRecordListener(RecordListener callback) = 0;
  virtual void UnregisterRecordListener(std::list<RecordListener>::iterator it) = 0;
};


class SerialPortInterface;
class SerialDeviceInterface {
 public:
  virtual ~SerialDeviceInterface() = default;
  virtual void SendMessage(SerialPortInterface* port, uint8_t* data, size_t size) = 0;
};

enum SerialPortEvent {
  kSerialPortStatusChanged,
  kSerialPortData
};

class SerialPortInterface {
 public:
  virtual ~SerialPortInterface() = default;
  virtual void OnMessage(uint8_t* data, size_t size) {
    if (callback_)
      callback_(kSerialPortData, data, size);
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
    if (callback_) {
      callback_(kSerialPortStatusChanged, nullptr, 0);
    }
  }

  virtual void set_callback(std::function<void(SerialPortEvent, uint8_t*, size_t)> callback) {
    callback_ = callback;
  }

  inline uint32_t     port_id() const { return port_id_; }
  inline const char*  port_name() const { return port_name_; }
  inline bool         ready() const { return ready_; }

 protected:
  SerialDeviceInterface* device_;
  std::function<void(SerialPortEvent, uint8_t*, size_t)> callback_;
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
  virtual bool offload_checksum() = 0;
  virtual bool offload_segmentation() = 0;
  virtual ~NetworkDeviceInterface() = default;
  virtual bool WriteBuffer(void* buffer, size_t size) = 0;
};

class NetworkBackendInterface {
 public:
  virtual ~NetworkBackendInterface() = default;
  virtual void Initialize(NetworkDeviceInterface* device, MacAddress& mac) = 0;
  virtual void SetMtu(int mtu) = 0;
  virtual void Reset() = 0;
  virtual void OnFrameFromGuest(std::deque<iovec>& vector) = 0;
  virtual void OnReceiveAvailable() = 0;

  inline NetworkDeviceInterface* device() { return device_; }

 protected:
  NetworkDeviceInterface*   device_;
};


class PowerDownInterface {
 public:
  virtual ~PowerDownInterface() = default;
  virtual void PowerDown() = 0;
};

class CmosDataInterface {
 public:
  virtual ~CmosDataInterface() = default;
  virtual void SetData(uint8_t index, uint8_t data) = 0;
};

#endif // _MVISOR_DEVICE_INTERFACES_H
