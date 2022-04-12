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

  void SendDisplayStreamStartEvent(uint w, uint h);
  void SendDisplayStreamStopEvent();
  void SendDisplayStreamDataEvent(void* data, size_t length);

 private:
  bool Send(uint32_t type);
  bool Send(uint32_t type, Message& message);
  bool Send(uint32_t type, void* data, size_t length);
  void ParsePacket(SweetPacketHeader* header);
  void OnQueryStatus(); 
  void OnKeyboardInput();
  void OnSendPointerInput();
  void OnConfigMonitors();
  void OnStartDisplayStream();

  Machine*      machine_;
  SweetServer*  server_;
  int           fd_ = -1;
  std::string   buffer_;
  std::mutex    mutex_;
};

#endif // _MVISOR_SWEET_CONNECTION_H
