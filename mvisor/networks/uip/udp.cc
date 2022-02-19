/* 
 * MVisor UDP Redirect
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

#include "uip.h"
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include "logger.h"
#include "device_manager.h"


UdpSocket::UdpSocket(NetworkBackendInterface* backend, ethhdr* eth, iphdr* ip, udphdr* udp) :
  Ipv4Socket(backend, eth, ip) {
  sport_ = ntohs(udp->source);
  dport_ = ntohs(udp->dest);
}

Ipv4Packet* UdpSocket::AllocatePacket(bool urgent) {
  Ipv4Packet* packet = Ipv4Socket::AllocatePacket(urgent);
  if (packet) {
    packet->udp = (udphdr*)packet->data;
    packet->data = (void*)&packet->udp[1];
  }
  return packet;
}

uint16_t UdpSocket::CalculateUdpChecksum(Ipv4Packet* packet) {
  PseudoHeader hdr;
  auto ip = packet->ip;
  auto udp = packet->udp;
  int udp_len;
  uint8_t *pad;

  ip = packet->ip;

  hdr.sip = ip->saddr;
  hdr.dip = ip->daddr;
  hdr.zero = 0;
  hdr.protocol = ip->protocol;
  hdr.length = udp->len;

  udp_len = ntohs(udp->len);

  if (udp_len % 2) {
    pad = (uint8_t *)udp + udp_len;
    *pad = 0;
    memcpy((uint8_t *)udp + udp_len + 1, &hdr, sizeof(hdr));
    return CalculateChecksum((uint8_t *)udp, udp_len + 1 + sizeof(hdr));
  } else {
    memcpy((uint8_t *)udp + udp_len, &hdr, sizeof(hdr));
    return CalculateChecksum((uint8_t *)udp, udp_len + sizeof(hdr));
  }
}

void UdpSocket::OnDataFromHost(Ipv4Packet* packet) {
  auto udp = packet->udp;
  udp->check = 0;
  udp->source = htons(dport_);
  udp->dest = htons(sport_);
  udp->len = htons(sizeof(udphdr) + packet->data_length);

  auto ip = packet->ip;
  ip->version = 4;
  ip->ihl = 5;
  ip->tos = 0;
  ip->tot_len = htons(sizeof(udphdr) + sizeof(iphdr) + packet->data_length);
  ip->id = 0;
  ip->frag_off = htons(0x4000);
  ip->ttl = 128;
  ip->protocol = 0x11;
  ip->check = 0;
  ip->saddr = htonl(dip_);
  ip->daddr = htonl(sip_);

  // checksum
  ip->check = CalculateChecksum((uint8_t*)ip, ip->ihl * 4);
  udp->check = CalculateUdpChecksum(packet);

  backend_->OnPacketFromHost(packet);

  active_time_ = time(nullptr);
}

RedirectUdpSocket::~RedirectUdpSocket() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
  if (polling_request_) {
    auto device = dynamic_cast<Device*>(backend_->device());
    device->manager()->io()->StopPolling(polling_request_);
  }
}

bool RedirectUdpSocket::IsActive() {
  // Kill timedout
  if (time(nullptr) - active_time_ >= REDIRECT_TIMEOUT_SECONDS) {
    return false;
  }
  return UdpSocket::IsActive();
}

void RedirectUdpSocket::InitializeRedirect() {
  fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  MV_ASSERT(fd_ >= 0);

  // Set non-blocking
  int flags = fcntl(fd_, F_GETFL, 0);
  flags |= O_NONBLOCK;
  fcntl(fd_, F_SETFL, flags);

  auto device = dynamic_cast<Device*>(backend_->device());
  polling_request_ = device->manager()->io()->StartPolling(fd_, POLLIN, [=](uint events) {
    if (events & POLLIN) {
      OnRemoteDataAvailable();
    }
  });

  debug_ = device->debug();
  if (debug_) {
    MV_LOG("UDP fd=%d %x:%u -> %x:%u", fd_, sip_, sport_, dip_, dport_);
  }
}

void RedirectUdpSocket::OnRemoteDataAvailable() {
  auto packet = AllocatePacket(false);
  if (packet == nullptr) {
    if (debug_) {
      MV_LOG("UDP fd=%d failed to allocate packet", fd_, this);
    }
    auto device = dynamic_cast<Device*>(backend_->device());
    auto io = device->manager()->io();
    io->ModifyPolling(polling_request_, 0);
    io->AddTimer(10, false, [io, this]() {
      io->ModifyPolling(polling_request_, POLLIN);
    });
    active_time_ = time(nullptr);
    return;
  }

  int ret = recvfrom(fd_, packet->data, UIP_MAX_UDP_PAYLOAD, 0, nullptr, nullptr);
  if (ret < 0) {
    packet->Release();
    return;
  }
  
  packet->data_length = ret;
  OnDataFromHost(packet);
}

void RedirectUdpSocket::OnDataFromGuest(void* data, size_t length) {
  sockaddr_in daddr = {
    .sin_family = AF_INET,
    .sin_port = htons(dport_),
    .sin_addr = {
      .s_addr = htonl(dip_)
    }
  };
  int ret = sendto(fd_, data, length, 0, (struct sockaddr*)&daddr, sizeof(daddr));
  if (ret != (int)length) {
    if (ret < 0) {
      return;
    }
    MV_PANIC("failed to send all UDP data, length=%lu, ret=%d", length, ret);
  }
}
