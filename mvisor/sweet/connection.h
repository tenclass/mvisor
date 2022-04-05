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

#include <thread>
#include <string>

#include "machine.h"
#include "device_interface.h"
#include "sweet/server.h"
#include "sweet/sweet.h"
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
  void Kick();

 private:
  void Process();
  bool Send(uint32_t type, Message& message);
  void OnReceive(SweetPacketHeader* header);
  void OnQueryStatus();
  void OnStartDisplayStream();

  Machine*      machine_;
  SweetServer*  server_;
  int           fd_ = -1;
  int           event_fd_ = -1;
  std::thread   thread_;
  std::string   buffer_;
};

#endif // _MVISOR_SWEET_CONNECTION_H
