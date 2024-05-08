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

#include "udp.h"
#include <arpa/inet.h>


UdpSocket::UdpSocket(Uip* backend, uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport) :
  Ipv4Socket(backend, sip, dip), sport_(sport), dport_(dport)
{
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
  ip->ttl = 64;
  ip->protocol = 0x11;
  ip->check = 0;
  ip->saddr = htonl(dip_);
  ip->daddr = htonl(sip_);

  // checksum
  ip->check = CalculateChecksum((uint8_t*)ip, ip->ihl * 4);
  udp->check = CalculateUdpChecksum(packet);

  backend_->OnPacketFromHost(packet);
}
