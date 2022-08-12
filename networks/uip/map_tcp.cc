/* 
 * MVisor TCP Port Mapping Service
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

#include "uip.h"

#include "utilities.h"
#include "logger.h"


MapTcpSocket::MapTcpSocket(NetworkBackendInterface* backend, uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport, int fd)
  : RedirectTcpSocket(backend, sip, dip, sport, dport)
{
  fd_ = fd;

  io_->StartPolling(fd_, EPOLLIN | EPOLLOUT | EPOLLET, [this](auto events) {
    can_read_ = events & EPOLLIN;
    can_write_ = events & EPOLLOUT;
  
    if (can_write()) {
      StartWriting();
    }
    if (can_read()) {
      StartReading();
    }
  });

  auto packet = AllocatePacket(true);
  if (packet) {
    OnDataFromHost(packet, TCP_FLAG_SYN);
    seq_host_ += 1;
  }
}

void MapTcpSocket::InitializeRedirect(Ipv4Packet* packet) {
  if (debug_) {
    MV_LOG("TCP fd=%d %x:%u -> %x:%u", fd_, dip_, dport_, sip_, sport_);
  }

  SynchronizeTcp(packet->tcp);
  
  // Initialize redirect states
  connected_ = true;
  if (can_write()) {
    StartWriting();
  }
  if (can_read()) {
    StartReading();
  }
}

