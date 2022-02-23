/* 
 * MVisor UDP Redirect
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
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include "logger.h"

RedirectUdpSocket::~RedirectUdpSocket() {
  if (fd_ >= 0) {
    io_->StopPolling(fd_);
    close(fd_);
    fd_ = -1;
  }
  if (io_ && wait_timer_) {
    io_->RemoveTimer(wait_timer_);
  }
}

bool RedirectUdpSocket::IsActive() {
  // Kill timedout
  if (time(nullptr) - active_time_ >= REDIRECT_TIMEOUT_SECONDS) {
    return false;
  }
  return true;
}

void RedirectUdpSocket::InitializeRedirect() {
  fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  MV_ASSERT(fd_ >= 0);

  // Set non-blocking
  fcntl(fd_, F_SETFL, fcntl(fd_, F_GETFL, 0) | O_NONBLOCK);

  sockaddr_in daddr = {
    .sin_family = AF_INET,
    .sin_port = htons(dport_),
    .sin_addr = {
      .s_addr = htonl(dip_)
    }
  };
  auto ret = connect(fd_, (struct sockaddr*)&daddr, sizeof(daddr));
  MV_ASSERT(ret == 0);

  io_->StartPolling(fd_, EPOLLIN | EPOLLET, [this](auto events) {
    if (events & EPOLLIN) {
      StartReading();
    }
  });

  if (debug_) {
    MV_LOG("UDP fd=%d %x:%u -> %x:%u", fd_, sip_, sport_, dip_, dport_);
  }
}

void RedirectUdpSocket::StartReading() {
  while (fd_ != -1) {
    auto packet = AllocatePacket(false);
    if (packet == nullptr) {
      /* FIXME: This code is not elegantly */
      wait_timer_ = io_->AddTimer(10, false, [this]() {
        wait_timer_ = nullptr;
        StartReading();
      });
      if (debug_) {
        MV_LOG("UDP fd=%d failed to allocate packet, retry later", fd_, this);
      }
      return;
    }

    int ret = recv(fd_, packet->data, UIP_MAX_UDP_PAYLOAD, 0);
    if (ret < 0) {
      packet->Release();
      return;
    }
    
    packet->data_length = ret;
    OnDataFromHost(packet);
    active_time_ = time(nullptr);
  }
}

void RedirectUdpSocket::OnPacketFromGuest(Ipv4Packet* packet) {
  if (fd_ == -1) {
    packet->Release();
    return;
  }

  int ret = send(fd_, packet->data, packet->data_length, 0);
  packet->Release();
  if (ret < 0) {
    return;
  }
  MV_ASSERT(ret == (int)packet->data_length);
  active_time_ = time(nullptr);
}

