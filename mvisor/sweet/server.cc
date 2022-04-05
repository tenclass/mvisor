/* 
 * MVisor - Sweet Server
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

#include "sweet/server.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "connection.h"
#include "logger.h"

SweetServer::SweetServer(Machine* machine, std::string unix_path) :
  machine_(machine), unix_path_(unix_path) {
  
  server_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
  MV_ASSERT(server_fd_ != -1);

  /* in case the unix path already exists */
  unlink(unix_path_.c_str());

  struct sockaddr_un un = { .sun_family = AF_UNIX };
  strncpy(un.sun_path, unix_path_.c_str(), sizeof(un.sun_path) - 1);

  if (bind(server_fd_, (sockaddr*)&un, sizeof(un)) < 0) {
    MV_PANIC("failed to bind unix socket %s", unix_path_.c_str());
  }

  MV_ASSERT(listen(server_fd_, 1) == 0);
}

SweetServer::~SweetServer() {
  for (auto conn : connections_) {
    conn->Kick();
    delete conn;
  }

  safe_close(&server_fd_);
}

int SweetServer::MainLoop() {
  SetThreadName("sweet-server");

  while (machine_->IsValid()) {
    struct sockaddr_un un;
    socklen_t len;
    int child_fd = accept(server_fd_, (struct sockaddr *)&un, &len);
    if (child_fd < 0) {
      MV_LOG("child_fd=%d errno=%d", child_fd, errno);
      break;
    }

    auto conn = new SweetConnection(this, child_fd);
    connections_.push_back(conn);
  }
  return 0;
}

void SweetServer::Close() {
  /* if server_fd_ is valid, accept() will return -1 and error=9 (EBADF) */
  safe_close(&server_fd_);
}
