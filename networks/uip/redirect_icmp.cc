/* 
 * MVisor ICMP Redirect
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

#include "redirect_icmp.h"

#include <fcntl.h>
#include <arpa/inet.h>

#include "logger.h"
#include "utilities.h"

RedirectIcmpSocket::~RedirectIcmpSocket() {
  if (fd_ != -1) {
    device_->StopPolling(fd_);
    safe_close(&fd_);
  }
}

bool RedirectIcmpSocket::active() {
  if (fd_ < 0) {
    return false;
  }
  // Kill timed out
  if (time(nullptr) - active_time_ >= REDIRECT_TIMEOUT_SECONDS) {
    return false;
  }
  return true;
}

void RedirectIcmpSocket::InitializeRedirect() {
  fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_ICMP);
  if (fd_ < 0) {
    fd_ = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
    if (fd_ < 0) {
      MV_ERROR("ICMP failed to create socket");
      return;
    }
    raw_mode_ = true;
  }

  if (debug_) {
    MV_LOG("ICMP%s fd=%d %x -> %x", raw_mode_ ? "(RAW)" : "", fd_, sip_, dip_);
  }

  // Set non-blocking
  fcntl(fd_, F_SETFL, fcntl(fd_, F_GETFL, 0) | O_NONBLOCK);

  sockaddr_in daddr = {
    .sin_family = AF_INET,
    .sin_port = 0,
    .sin_addr = {
      .s_addr = htonl(dip_)
    },
    .sin_zero = {0}
  };

  auto ret = connect(fd_, (struct sockaddr*)&daddr, sizeof(daddr));
  if (ret < 0) {
    if (debug_) {
      MV_ERROR("ICMP fd=%d failed to connect socket", fd_);
    }
    safe_close(&fd_);
    return;
  }

  device_->StartPolling(fd_, EPOLLIN | EPOLLET, [this](auto events) {
    can_read_ = events & EPOLLIN;
    if (can_read()) {
      StartReading();
    }
  });
}

void RedirectIcmpSocket::OnGuestBufferAvaialble() {
  if (can_read()) {
    StartReading();
  }
}

void RedirectIcmpSocket::StartReading() {
  while (can_read()) {
    auto packet = AllocatePacket(false);
    if (packet == nullptr) {
      return;
    }

    /* FIXME: Limit packet size for Linux driver */
    auto recv_size = IPV4_MAX_UDP_PAYLOAD(packet);

    int ret = recv(fd_, packet->data, recv_size, 0);
    if (ret < 0) {
      packet->Release();
      can_read_ = false;
      return;
    }

    if (raw_mode_) {
      auto ip = (iphdr*)packet->data;
      auto icmp = (uint8_t*)packet->data + ip->ihl * 4;
      packet->data_length = ret - ip->ihl * 4;
      memmove(packet->data, icmp, packet->data_length);
    } else {
      packet->data_length = ret;
    }
    
    OnDataFromHost(packet);
    active_time_ = time(nullptr);
  }
}

void RedirectIcmpSocket::OnPacketFromGuest(Ipv4Packet* packet) {
  if (fd_ < 0) {
    packet->Release();
    return;
  }

  sockaddr_in daddr = {
    .sin_family = AF_INET,
    .sin_port = 0,
    .sin_addr = {
      .s_addr = htonl(dip_)
    },
    .sin_zero = {0}
  };

  int ret = sendto(fd_, packet->data, packet->data_length, 0, (sockaddr*)&daddr, sizeof daddr);
  packet->Release();
  if (ret < 0) {
    return;
  }
  MV_ASSERT(ret == (int)packet->data_length);
  active_time_ = time(nullptr);
}

