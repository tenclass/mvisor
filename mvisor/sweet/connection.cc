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


#include "connection.h"
#include "utilities.h"
#include "logger.h"
#include "pb/sweet.pb.h"


SweetConnection::SweetConnection(SweetServer* server, int fd) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  server_ = server;
  machine_ = server->machine();
  fd_ = fd;
  MV_LOG("sweet connection created fd=%d", fd);
}

SweetConnection::~SweetConnection() {
  safe_close(&fd_);
}

bool SweetConnection::OnReceive() {
  SweetPacketHeader header;
  int nbytes = recv(fd_, &header, sizeof(header), MSG_WAITALL);
  if (nbytes <= 0) {
    MV_LOG("recv nbytes=%ld", nbytes);
    return false;
  }
  
  ParsePacket(&header);
  return true;
}

void SweetConnection::ParsePacket(SweetPacketHeader* header) {
  if (header->length) {
    buffer_.resize(header->length);
    int ret = recv(fd_, buffer_.data(), header->length, MSG_WAITALL);
    if (ret != (int)header->length) {
      MV_LOG("failed to recv %u bytes, ret=%d", header->length, ret);
      return;
    }
  }

  switch (header->type)
  {
  case kQueryStatus:
    OnQueryStatus();
    break;
  case kStartDisplayStream:
    OnStartDisplayStream();
    break;
  case kStopDisplayStream:
    server_->StopDisplayStream();
    break;
  default:
    MV_LOG("unhandled sweet type=0x%x", header->type);
  }
}

bool SweetConnection::Send(uint32_t type, void* data, size_t length) {
  /* Send() is called by encoder thread to send encoded frames */
  std::lock_guard<std::mutex> lock(mutex_);

  SweetPacketHeader header = {
    .type = type,
    .length = (uint32_t)length
  };
  int ret = send(fd_, &header, sizeof(header), 0);
  if (ret != sizeof(header)) {
    MV_LOG("failed to send message, ret=%d", ret);
    return false;
  }

  if (length > 0) {
    ret = send(fd_, data, length, 0);
    if (ret != (int)length) {
      MV_LOG("failed to send message, ret=%d", ret);
      return false;
    }
  }
  return true;
}

bool SweetConnection::Send(uint32_t type, Message& message) {
  /* Reuse the buffer */
  if (!message.SerializeToString(&buffer_)) {
    MV_LOG("failed to serialize message type=0x%x", type);
    return false;
  }
  return Send(type, buffer_.data(), buffer_.size());
}

bool SweetConnection::Send(uint32_t type) {
  return Send(type, nullptr, 0);
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

  Send(kQueryStatusResponse, response);
}

void SweetConnection::OnStartDisplayStream() {
  DisplayStreamConfig config;
  if (config.ParseFromString(buffer_)) {
    server_->StartDisplayStreamOnConnection(this, &config);
  }
}

void SweetConnection::SendDisplayStreamStartEvent(uint w, uint h) {
  DisplayStreamStartEvent event;
  event.set_width(w);
  event.set_height(h);
  Send(kDisplayStreamStartEvent, event);
}

void SweetConnection::SendDisplayStreamStopEvent() {
  Send(kDisplayStreamStopEvent);
}

void SweetConnection::SendDisplayStreamDataEvent(void* data, size_t length) {
  Send(kDisplayStreamDataEvent, data, length);
}

