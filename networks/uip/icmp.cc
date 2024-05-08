/* 
 * MVisor ICMP
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

#include "icmp.h"
#include <arpa/inet.h>


IcmpSocket::IcmpSocket(Uip* backend, uint32_t sip, uint32_t dip, uint16_t echo_id):
  Ipv4Socket(backend, sip, dip) {
  echo_id_ = echo_id;
}

Ipv4Packet* IcmpSocket::AllocatePacket(bool urgent) {
  Ipv4Packet* packet = Ipv4Socket::AllocatePacket(urgent);
  if (packet) {
    packet->icmp = (icmphdr*)packet->data;
  }
  return packet;
}

uint16_t IcmpSocket::CalculateIcmpChecksum(Ipv4Packet* packet) {
  auto icmp = packet->icmp;
  int icmp_len = packet->data_length;
  uint16_t* ptr = (uint16_t*)icmp;
  uint32_t sum = 0;

  while(icmp_len > 1)
  {
    sum += *ptr++;
    icmp_len -= 2; 
  }

  if(icmp_len == 1)
  {  
    uint16_t word = *(uint8_t*)ptr;
    word = (word << 8) & 0xFF;
    sum += word;                
  }
  sum = (sum >> 16) + (sum & 0xFFFF); 
  sum += sum >> 16;  
  return (uint16_t)(~sum);
}

void IcmpSocket::OnDataFromHost(Ipv4Packet* packet) {
  auto icmp = packet->icmp;
  if (icmp->type == ICMP_ECHOREPLY) {
    icmp->un.echo.id = htons(echo_id_);
  }

  auto ip = packet->ip;
  ip->version = 4;
  ip->ihl = 5;
  ip->tos = 0;
  ip->tot_len = htons(sizeof(iphdr) + packet->data_length);
  ip->id = 0;
  ip->frag_off = htons(0x4000);
  ip->ttl = 64;
  ip->protocol = 0x1;
  ip->check = 0;
  ip->saddr = htonl(dip_);
  ip->daddr = htonl(sip_);

  // checksum
  ip->check = CalculateChecksum((uint8_t*)ip, ip->ihl * 4);
  if (!packet->offload_checksum) {
    icmp->checksum = 0;
    icmp->checksum = CalculateIcmpChecksum(packet);
  }

  backend_->OnPacketFromHost(packet);
}
