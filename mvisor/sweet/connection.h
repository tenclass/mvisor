/* 
 * MVisor - Sweet Connections
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


#ifndef _MVISOR_SWEET_CONNECTION_H
#define _MVISOR_SWEET_CONNECTION_H

#include <string>
#include <vector>
#include <mutex>

#include "machine.h"
#include "device_interface.h"
#include "sweet/server.h"
#include "pb/sweet.pb.h"

using namespace SweetProtocol;

/* 16 bytes header for every sweet packet */
struct SweetPacketHeader {
  uint32_t  type;
  uint32_t  length;
  uint32_t  flags;
  uint32_t  reserved;
};


class SweetConnection {
 public:
  SweetConnection(SweetServer* server, int fd);
  ~SweetConnection();

  int fd() { return fd_; }
  bool OnReceive();

  /* Not thread-safe. The caller must be sweet server thread */
  bool Send(uint32_t type);
  bool Send(uint32_t type, const Message& message);
  bool Send(uint32_t type, void* data, size_t length);
  bool Send(uint32_t type, const std::string& data);

  void SendDisplayStreamStartEvent(uint w, uint h);
  void UpdateCursor(const DisplayMouseCursor* cursor_update);
  void SendPlaybackStreamStartEvent(std::string codec, uint format, uint channels, uint frequency);
  void SendClipboardStreamDataEvent(const ClipboardData& clipboard_data);
  void SendSerialPortStatusEvent(std::string name, bool ready);

 private:
  void ParsePacket(SweetPacketHeader* header);
  void OnQueryStatus();
  void OnKeyboardInput();
  void OnSendPointerInput();
  void OnConfigMonitors();
  void OnStartDisplayStream();
  void OnStartPlaybackStream();
  void OnQueryScreenshot();
  void OnSaveMachine();
  void OnClipboardDataToGuest();
  void OnStartClipboardStream();
  void OnStopClipboardStream();
  void OnStartVirtioFs();
  void OnStopVirtioFs();
  void OnMidiInput();
  void OnStartMidi();
  void OnStopMidi();

  void OnStartWacom();
  void OnStopWacom();
  void OnWacomInput();
  void OnStartRecordStream();
  void OnStopRecordStream();
  void OnSendRecordStreamData();


  Machine*      machine_;
  SweetServer*  server_;
  int           fd_ = -1;
  std::string   buffer_;
  std::mutex    mutex_;

  bool          cursor_visible_ = false;
  uint64_t      cursor_shape_id_ = 0;
};

#endif // _MVISOR_SWEET_CONNECTION_H
