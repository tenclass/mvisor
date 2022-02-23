/* 
 * MVisor TCP Redirect
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
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include "logger.h"


RedirectTcpSocket::RedirectTcpSocket(NetworkBackendInterface* backend, Ipv4Packet* packet) :
  TcpSocket(backend, packet) {
  fd_ = -1;
  InitializeRedirect();
}

RedirectTcpSocket::~RedirectTcpSocket() {
  for (auto packet : send_queue_) {
    packet->Release();
  }

  if (fd_ > 0) {
    io_->StopPolling(fd_);
    close(fd_);
    fd_ = -1;
  }
}

bool RedirectTcpSocket::IsActive() {
  // Kill half closed
  if ((read_done_ || write_done_) && time(nullptr) - active_time_ >= REDIRECT_TIMEOUT_SECONDS) {
    return false;
  }
  return true;
}

void RedirectTcpSocket::Shutdown(int how) {
  if (fd_ == -1)
    return;

  if (how == SHUT_WR) { // Guest sent FIN
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
    if (!read_done_) {
      read_done_ = true;

      auto packet = AllocatePacket(true);
      if (packet) {
        OnDataFromHost(packet, TCP_FLAG_FIN | TCP_FLAG_ACK);
      }
      seq_host_ += 1;
    }
  }

  if (debug_) {
    MV_LOG("TCP fd=%d shutdown %s %x:%u -> %x:%u", fd_, how == SHUT_WR ? "WRITE" : "READ",
      sip_, sport_, dip_, dport_);
  }
}

void RedirectTcpSocket::InitializeRedirect() {
  fd_ = socket(AF_INET, SOCK_STREAM, 0);
  MV_ASSERT(fd_ >= 0);

  // Set non-blocking
  MV_ASSERT(fcntl(fd_, F_SETFL, fcntl(fd_, F_GETFL, 0) | O_NONBLOCK) != -1);

  sockaddr_in daddr = {
    .sin_family = AF_INET,
    .sin_port = htons(dport_),
    .sin_addr = {
      .s_addr = htonl(dip_)
    }
  };
  
  auto ret = connect(fd_, (sockaddr*)&daddr, sizeof(daddr));
  MV_ASSERT(ret < 0 && errno == EINPROGRESS);
  io_->StartPolling(fd_, EPOLLOUT | EPOLLIN | EPOLLET, [this](auto events) {
    can_read_ = events & EPOLLIN;
    can_write_ = events & EPOLLOUT;
  
    if (!connected_ && can_write_) {
      OnRemoteConnected();
    }
    if (can_write()) {
      StartWriting();
    }
    if (can_read()) {
      StartReading();
    }
  });

  if (debug_) {
    MV_LOG("TCP fd=%d %x:%u -> %x:%u", fd_, sip_, sport_, dip_, dport_);
  }
}

void RedirectTcpSocket::OnRemoteConnected() {
  MV_ASSERT(!connected_);
  connected_ = true;
  auto packet = AllocatePacket(true);
  if (packet) {
    OnDataFromHost(packet, TCP_FLAG_SYN | TCP_FLAG_ACK);
  }
  seq_host_ += 1;
}

bool RedirectTcpSocket::UpdateGuestAck(tcphdr* tcp) {
  if (TcpSocket::UpdateGuestAck(tcp)) {
    if (can_read()) {
      StartReading();
    }
    return true;
  }
  return false;
}

/* If receive operation is controlled, retry when a guest ACK comes */
void RedirectTcpSocket::StartReading() {
  while (can_read()) {
    /* Check if controlled by TCP window */
    int available = (int)(window_size_ - (seq_host_ - guest_acked_));
    if (available > UIP_MAX_TCP_PAYLOAD) {
      available = UIP_MAX_TCP_PAYLOAD;
    }
    if (available <= 0) {
      return;
    }

    /* Check if virtio buffer is full */
    auto packet = AllocatePacket(false);
    if (packet == nullptr) {
      if (debug_) {
        MV_LOG("TCP fd=%d failed to allocate packet", fd_);
      }
      return;
    }

    int ret = recv(fd_, packet->data, available, 0);
    if (ret <= 0) {
      packet->Release();
      if (ret < 0 && errno == EAGAIN) {
        can_read_ = false;
        return;
      }
      Shutdown(SHUT_RD);
    } else {
      packet->data_length = ret;
      OnDataFromHost(packet, TCP_FLAG_ACK);
      seq_host_ += packet->data_length;
      active_time_ = time(nullptr);
    }
  }
}

void RedirectTcpSocket::StartWriting() {
  while (can_write()) {
    /* Check if no data to send */
    if (send_queue_.empty()) {
      return;
    }

    auto packet = send_queue_.front();
    auto length = packet->data_length - packet->data_offset;
    int ret = send(fd_, (uint8_t*)packet->data + packet->data_offset, length, MSG_NOSIGNAL);
    if (ret < 0) {
      if (errno == EAGAIN) {
        can_write_ = false;
        return;
      }
      if (debug_) {
        MV_LOG("ERROR TCP %d %x:%u -> %x:%u is already closed. length=%d ret=%d",
          fd_, sip_, sport_, dip_, dport_, length, ret);
      }
      Shutdown(SHUT_RD);
      return;
    }
    packet->data_offset += ret;
    if (packet->data_offset == packet->data_length) {
      packet->Release();
      send_queue_.pop_front();
    }
    active_time_ = time(nullptr);
  }
}

void RedirectTcpSocket::OnPacketFromGuest(Ipv4Packet* packet) {
  if (fd_ == -1 || write_done_) {
    packet->Release();
    return;
  }
  send_queue_.push_back(packet);

  ack_host_ += packet->data_length;
  auto ack_packet = AllocatePacket(true);
  if (ack_packet) {
    OnDataFromHost(ack_packet, TCP_FLAG_ACK);
  }

  if (can_write()) {
    StartWriting();
  }
}
