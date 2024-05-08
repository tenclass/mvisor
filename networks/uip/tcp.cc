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

#include "tcp.h"

#include <arpa/inet.h>

#include "logger.h"


TcpSocket::TcpSocket(Uip* backend, uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport) :
  Ipv4Socket(backend, sip, dip), sport_(sport), dport_(dport) {
  mss_ = 1460;
  // setup initial sequence and acknowledge
  seq_host_ = 0x66666666;
  window_scale_ = 6;
  window_size_ = 65535 << window_scale_;
}

void TcpSocket::SynchronizeTcp(tcphdr* tcp) {
  window_size_ = ntohs(tcp->window) << window_scale_;
  ack_host_ = ntohl(tcp->seq) + 1;

  if (tcp->ack) {
    guest_acked_ = ntohl(tcp->ack_seq);
  }

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

/* TCP options set for packets from host to guest */
void TcpSocket::FillTcpOptions(tcphdr* tcp) {
  uint8_t* options = (uint8_t*)tcp + sizeof(*tcp);
  uint8_t* tcp_data = (uint8_t*)tcp + (tcp->doff * 4);
  bzero(options, tcp_data - options);
  uint8_t* p = options;
  // Window scale = 6, max window size = 64K * (1 << 6) = 4M
  *p++ = 3;
  *p++ = 3;
  *p++ = 6;
  // MSS
  *p++ = 2;
  *p++ = 4;
  *(uint16_t*)p = htons(mss_);
  p += 2;
  // 5 NOPs
  for (int i = 0; i < 5; i++) {
    *p++ = 1;
  }
}

bool TcpSocket::UpdateGuestAck(tcphdr* tcp) {
  if (ntohl(tcp->seq) == ack_host_) {
    window_size_ = ntohs(tcp->window) << window_scale_;
    guest_acked_ = ntohl(tcp->ack_seq);
    return true;
  }

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
  MV_ASSERT(tcp_len <= IPV4_MAX_TCP_PAYLOAD(packet) + 20);

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
  if (flags & TCP_FLAG_PSH) {
    tcp->psh = 1;
  }
  if (flags & TCP_FLAG_FIN) {
    tcp->fin = 1;
  }
  if (flags & TCP_FLAG_RST) {
    tcp->rst = 1;
  }
  tcp->doff = 5;
  if (flags & TCP_FLAG_SYN) {
    tcp->syn = 1;
    // Add 4 * 3 = 12 bytes for options
    tcp->doff += 3;
    FillTcpOptions(tcp);
  }

  // FIXME: fixed window size
  if (tcp->rst) {
    tcp->window = 0;
  } else {
    tcp->window = htons(window_size_ >> window_scale_);
  }
  tcp->check = 0;
  tcp->urg_ptr = 0;

  auto ip = packet->ip;
  ip->version = 4;
  ip->ihl = 5;
  ip->tos = 0;
  ip->tot_len = htons(tcp->doff * 4 + sizeof(iphdr) + packet->data_length);
  ip->id = 0;
  ip->frag_off = htons(0x4000);
  ip->ttl = 64;
  ip->protocol = 0x06;
  ip->check = 0;
  ip->saddr = htonl(dip_);
  ip->daddr = htonl(sip_);

  // checksum
  ip->check = CalculateChecksum((uint8_t*)ip, ip->ihl * 4);
  if (!packet->offload_checksum) {
    tcp->check = CalculateTcpChecksum(packet);
  }

  if (packet->offload_segmentation) {
    packet->vnet->gso_type = VIRTIO_NET_HDR_GSO_TCPV4;
    packet->vnet->hdr_len = (uint8_t*)packet->data - packet->buffer;
    packet->vnet->gso_size = mss_;
  }

  backend_->OnPacketFromHost(packet);
}

