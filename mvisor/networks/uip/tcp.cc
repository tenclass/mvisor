/* 
 * MVisor TCP Redirect
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
#include <unistd.h>
#include <poll.h>
#include "logger.h"
#include "device_manager.h"


TcpSocket::TcpSocket(NetworkBackendInterface* backend, ethhdr* eth, iphdr* ip, tcphdr* tcp) :
  Ipv4Socket(backend, eth, ip) {
  sport_ = ntohs(tcp->source);
  dport_ = ntohs(tcp->dest);

  mss_ = 1460;
  sack_permitted_ = false;
  window_scale_ = 0;
  window_size_ = ntohs(tcp->window);
  guest_acked_ = 0;

  // setup ISN
  isn_guest_ = ntohl(tcp->seq);
  isn_host_ = 10000001;
  seq_host_ = isn_host_;
  ack_host_ = isn_guest_ + 1;

  ParseTcpOptions(tcp);
}

void TcpSocket::ParseTcpOptions(tcphdr* tcp) {
  uint8_t* options = (uint8_t*)tcp + sizeof(*tcp);
  uint8_t* tcp_data = (uint8_t*)tcp + (tcp->doff * 4);
  for (uint8_t* p = options; p < tcp_data;) {
    switch (*p++)
    {
    case 0: // END
      p = tcp_data;
      break;
    case 1: // NOP
      break;
    case 2: // MSS
      ++p;
      mss_ = ntohs(*(uint16_t*)p);
      p += 2;
      break;
    case 3: // Window scale
      ++p;
      window_scale_ = *p;
      ++p;
      break;
    case 4: // SACK permitted
      ++p;
      sack_permitted_ = true;
      break;
    default:
      if (debug_) {
        MV_LOG("unknown TCP option %d", *(p - 1));
      }
      p += *p - 1;
      break;
    }
  }
}

void TcpSocket::FillTcpOptions(tcphdr* tcp) {
  uint8_t* options = (uint8_t*)tcp + sizeof(*tcp);
  uint8_t* tcp_data = (uint8_t*)tcp + (tcp->doff * 4);
  bzero(options, tcp_data - options);
  uint8_t* p = options;
  // Window scale = 0
  *p++ = 3;
  *p++ = 3;
  *p++ = 0;
}

bool TcpSocket::UpdateGuestAck(tcphdr* tcp) {
  if (ntohl(tcp->seq) == ack_host_) {
    window_size_ = ntohs(tcp->window) << window_scale_;
    guest_acked_ = ntohl(tcp->ack_seq);
    return true;
  }

  // Keep alive
  active_time_ = time(nullptr);
  auto packet = AllocatePacket(true);
  if (packet) {
    OnDataFromHost(packet, TCP_FLAG_ACK);
  }
  return false;
}

Ipv4Packet* TcpSocket::AllocatePacket(bool urgent) {
  Ipv4Packet* packet = Ipv4Socket::AllocatePacket(urgent);
  if (packet) {
    packet->tcp = (tcphdr*)packet->data;
    packet->data = (void*)&packet->tcp[1];
  }
  return packet;
}

uint16_t TcpSocket::CalculateTcpChecksum(Ipv4Packet* packet) {
  PseudoHeader hdr;
  auto ip = packet->ip;
  auto tcp = packet->tcp;
  int tcp_len;
  uint8_t *pad;

  ip = packet->ip;
  tcp_len = ntohs(ip->tot_len) - ip->ihl * 4;
  MV_ASSERT(tcp_len <= UIP_MAX_TCP_PAYLOAD + 20);

  hdr.sip = ip->saddr;
  hdr.dip  = ip->daddr;
  hdr.zero = 0;
  hdr.protocol = ip->protocol;
  hdr.length = htons(tcp_len);

  if (tcp_len % 2) {
    pad = (uint8_t *)tcp + tcp_len;
    *pad = 0;
    memcpy((uint8_t *)tcp + tcp_len + 1, &hdr, sizeof(hdr));
    return CalculateChecksum((uint8_t *)tcp, tcp_len + 1 + sizeof(hdr));
  } else {
    memcpy((uint8_t *)tcp + tcp_len, &hdr, sizeof(hdr));
    return CalculateChecksum((uint8_t *)tcp, tcp_len + sizeof(hdr));
  }
}

void TcpSocket::OnDataFromHost(Ipv4Packet* packet, uint32_t flags) {
  tcphdr* tcp = packet->tcp;
  bzero(packet->tcp, sizeof(tcphdr));
  tcp->source = htons(dport_);
  tcp->dest = htons(sport_);
  tcp->seq = htonl(seq_host_);
  tcp->ack_seq = htonl(ack_host_);

  // clear flags
  ((uint16_t*)tcp)[6] = 0;
  if (flags & TCP_FLAG_ACK) {
    tcp->ack = 1;
  }
  if (flags & TCP_FLAG_FIN) {
    tcp->fin = 1;
  }
  if (flags & TCP_FLAG_SYN) {
    tcp->syn = 1;
    tcp->doff = 8;
    // To get peer window scale work, add options
    FillTcpOptions(tcp);
  } else {
    tcp->doff = 5;
  }

  // fixed window size, no more than 64K, window scale = 0
  tcp->window = htons(UIP_MAX_TCP_PAYLOAD);
  tcp->check = 0;
  tcp->urg_ptr = 0;

  auto ip = packet->ip;
  ip->version = 4;
  ip->ihl = 5;
  ip->tos = 0;
  ip->tot_len = htons(tcp->doff * 4 + sizeof(iphdr) + packet->data_length);
  ip->id = 0;
  ip->frag_off = htons(0x4000);
  ip->ttl = 128;
  ip->protocol = 0x06;
  ip->check = 0;
  ip->saddr = htonl(dip_);
  ip->daddr = htonl(sip_);

  // checksum
  ip->check = CalculateChecksum((uint8_t*)ip, ip->ihl * 4);
  tcp->check = CalculateTcpChecksum(packet);

  backend_->OnPacketFromHost(packet);

  active_time_ = time(nullptr);
}


RedirectTcpSocket::RedirectTcpSocket(NetworkBackendInterface* backend, ethhdr* eth, iphdr* ip, tcphdr* tcp) :
  TcpSocket(backend, eth, ip, tcp) {
  fd_ = -1;
  InitializeRedirect();
}

RedirectTcpSocket::~RedirectTcpSocket() {
  if (polling_request_) {
    auto device = dynamic_cast<Device*>(backend_->device());
    device->manager()->io()->StopPolling(polling_request_);
  }
  if (fd_ > 0) {
    close(fd_);
    fd_ = -1;
  }
}

bool RedirectTcpSocket::IsActive() {
  // Kill half closed
  if ((read_done_ || write_done_) && time(nullptr) - active_time_ >= REDIRECT_TIMEOUT_SECONDS) {
    return false;
  }
  return TcpSocket::IsActive();
}

void RedirectTcpSocket::Shutdown(int how) {
  connected_ = false;

  if (how == SHUT_WR) { // Guest sent FIN
    if (debug_) {
      MV_LOG("shutdown SHUT_WR TCP %d %x:%u -> %x:%u, errno=%d", fd_, sip_, sport_, dip_, dport_, errno);
    }
    if (!write_done_) {
      shutdown(fd_, how);
      write_done_ = true;

      ack_host_ += 1;
      auto packet = AllocatePacket(true);
      if (packet) {
        OnDataFromHost(packet, TCP_FLAG_ACK);
      }
    }
  } else if (how == SHUT_RD) {
    if (debug_) {
      MV_LOG("shutdown SHUT_RD TCP %d %x:%u -> %x:%u, errno=%d", fd_, sip_, sport_, dip_, dport_, errno);
    }
    if (polling_request_) {
      auto device = dynamic_cast<Device*>(backend_->device());
      device->manager()->io()->StopPolling(polling_request_);
      polling_request_ = nullptr;
    }
    if (!read_done_) {
      read_done_ = true;

      auto packet = AllocatePacket(true);
      if (packet) {
        OnDataFromHost(packet, TCP_FLAG_FIN | TCP_FLAG_ACK);
      }
      seq_host_ += 1;
    }
  }

  if (read_done_ && write_done_) {
    // Release resources
    close(fd_);
    fd_ = -1;
    closed_ = true;
  }
}

void RedirectTcpSocket::InitializeRedirect() {
  fd_ = socket(AF_INET, SOCK_STREAM, 0);
  MV_ASSERT(fd_ >= 0);

  // Set non-blocking
  int flags = fcntl(fd_, F_GETFL, 0);
  flags |= O_NONBLOCK;
  fcntl(fd_, F_SETFL, flags);

  closed_ = false;
  read_done_ = false;
  write_done_ = false;
  connected_ = false;
  sockaddr_in daddr = {
    .sin_family = AF_INET,
    .sin_port = htons(dport_),
    .sin_addr = {
      .s_addr = htonl(dip_)
    }
  };
  int ret = connect(fd_, (struct sockaddr*)&daddr, sizeof(daddr));
  MV_ASSERT(ret == -1 && errno == EINPROGRESS);

  auto device = dynamic_cast<Device*>(backend_->device());
  polling_request_ = device->manager()->io()->StartPolling(fd_, POLLIN | POLLOUT, [=](uint events) {
    if (events & POLLOUT) { // socket is connected or send buffer is not full
      device->manager()->io()->ModifyPolling(polling_request_, POLLIN);
      if (!connected_) {
        connected_ = true;
        OnRemoteConnected();
      }
    }
    if (events & POLLIN) {
      OnRemoteDataAvailable();
    }
    if (events & POLLRDHUP) {
      Shutdown(SHUT_RD);
    }
  });

  debug_ = device->debug();
  if (debug_) {
    MV_LOG("TCP fd=%d %x:%u -> %x:%u", fd_, sip_, sport_, dip_, dport_);
  }
}

void RedirectTcpSocket::OnRemoteConnected() {
  connected_ = true;
  auto packet = AllocatePacket(true);
  if (packet) {
    OnDataFromHost(packet, TCP_FLAG_SYN | TCP_FLAG_ACK);
  }
  seq_host_ += 1;
}

bool RedirectTcpSocket::UpdateGuestAck(tcphdr* tcp) {
  if (TcpSocket::UpdateGuestAck(tcp)) {
    if (window_size_ > 0 && polling_request_ && polling_request_->poll_mask == 0) {
      auto device = dynamic_cast<Device*>(backend_->device());
      auto io = device->manager()->io();
      io->ModifyPolling(polling_request_, POLLIN);
    }
    return true;
  }
  return false;
}

void RedirectTcpSocket::OnRemoteDataAvailable() {
  auto device = dynamic_cast<Device*>(backend_->device());
  auto io = device->manager()->io();

  int available = (int)(window_size_ - (seq_host_ - guest_acked_));
  if (available > UIP_MAX_TCP_PAYLOAD) {
    available = UIP_MAX_TCP_PAYLOAD;
  }
  if (available <= 0) {
    /* disable receiving packets now */
    io->ModifyPolling(polling_request_, 0);
    return;
  }
  auto packet = AllocatePacket(false);
  if (packet == nullptr) {
    if (debug_) {
      MV_LOG("TCP fd=%d failed to allocate packet", fd_);
    }
    io->ModifyPolling(polling_request_, 0);
    active_time_ = time(nullptr);
    return;
  }
  int ret = recv(fd_, packet->data, available, 0);
  if (ret <= 0) {
    if (errno != EAGAIN) {
      Shutdown(SHUT_RD);
    }
    packet->Release();
  } else {
    packet->data_length = ret;
    OnDataFromHost(packet, TCP_FLAG_ACK);
    seq_host_ += packet->data_length;
    guest_overflow_ = ret == available;
  }
}

void RedirectTcpSocket::OnDataFromGuest(void* data, size_t length) {
  if (write_done_) {
    return;
  }

  int ret = send(fd_, data, length, 0);
  if (ret != (int)length) {
    if (ret < 0) {
      MV_LOG("ERROR TCP %d %x:%u -> %x:%u is already closed. length=%d ret=%d",
        fd_, sip_, sport_, dip_, dport_, length, ret);
    } else {
      MV_PANIC("TCP %d invalid result length=%d ret=%d", fd_, length, ret);
    }
    return;
  }

  ack_host_ += ret;
  auto packet = AllocatePacket(true);
  if (packet) {
    OnDataFromHost(packet, TCP_FLAG_ACK);
  }
}
