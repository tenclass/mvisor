/* 
 * MVisor - Sweet Connection
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

#include <sys/eventfd.h>
#include <sys/poll.h>

#include "connection.h"
#include "utilities.h"
#include "logger.h"


SweetConnection::SweetConnection(SweetServer* server, int fd) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  server_ = server;
  machine_ = server->machine();
  fd_ = fd;
  MV_LOG("sweet connection created fd=%d", fd);

  /* event_fd_ is for waking up the connection thread */
  event_fd_ = eventfd(0, 0);
  thread_ = std::thread(&SweetConnection::Process, this);
}

SweetConnection::~SweetConnection() {
  if (thread_.joinable())
    thread_.join();

  safe_close(&event_fd_);
  safe_close(&fd_);
}

void SweetConnection::Kick() {
  if (event_fd_ > 0) {
    uint64_t tmp = 1;
    write(event_fd_, &tmp, sizeof(tmp));
  }
}

void SweetConnection::Process() {
  SetThreadName("sweet-connection");

  pollfd fds[2] = {
    {
      .fd = event_fd_,
      .events = POLLIN | POLLERR
    }, {
      .fd = fd_,
      .events = POLLIN | POLLERR
    }
  };

  while (machine_->IsValid()) {
    int ret = poll(fds, 2, -1);
    if (ret < 0) {
      MV_PANIC("poll ret=%d", ret);
    }

    if (fds[0].revents & POLLIN) {
      uint64_t tmp;
      read(event_fd_, &tmp, sizeof(tmp));
    }

    if (fds[1].revents & POLLERR) {
      MV_LOG("fd error, break now");
      break;
    }
    if (fds[1].revents & POLLIN) {
      SweetPacketHeader header;
      int nbytes = recv(fd_, &header, sizeof(header), MSG_WAITALL);
      if (nbytes <= 0) {
        MV_LOG("recv nbytes=%ld", nbytes);
        break;
      }
      
      OnReceive(&header);
    }
  }

  if (machine_->debug()) {
    MV_LOG("ended");
  }
}

bool SweetConnection::Send(uint32_t type, Message& message) {
  /* Reuse the buffer */
  if (!message.SerializeToString(&buffer_)) {
    MV_LOG("failed to serialize message type=0x%x", type);
    return false;
  }

  SweetPacketHeader header = {
    .type = type,
    .length = (uint32_t)buffer_.size()
  };
  int ret = send(fd_, &header, sizeof(header), 0);
  if (ret != sizeof(header)) {
    MV_LOG("failed to send message, ret=%d", ret);
    return false;
  }

  ret = send(fd_, buffer_.data(), buffer_.size(), 0);
  if (ret != (int)buffer_.size()) {
    MV_LOG("failed to send message, ret=%d", ret);
    return false;
  }
  return true;
}

void SweetConnection::OnReceive(SweetPacketHeader* header) {
  switch (header->type)
  {
  case kSweetCmdQueryStatus:
    OnQueryStatus();
    break;
  case kSweetCmdStartDisplayStream:
    OnStartDisplayStream();
    break;
  default:
    MV_LOG("unhandled sweet type=0x%x", header->type);
  }
}

void SweetConnection::OnQueryStatus() {
  QueryStatusResponse response;  
  response.set_debug(machine_->debug());
  response.set_paused(machine_->IsPaused());
  response.set_hypervisor(machine_->hypervisor());
  response.set_vcpu_count(machine_->num_vcpus());
  response.set_memory_size(machine_->ram_size());
  response.set_vm_uuid(machine_->vm_uuid());
  response.set_vm_name(machine_->vm_name());
  response.set_config_path(machine_->configuration()->path());

  auto spice_agent = dynamic_cast<SerialPortInterface*>(machine_->LookupObjectByClass("SpiceAgent"));
  if (spice_agent && spice_agent->ready()) {
    response.set_spice_agent(true);
  }

  Send(kSweetResQueryStatus, response);
}

void SweetConnection::OnStartDisplayStream() {
  MV_PANIC("start display stream");
}

