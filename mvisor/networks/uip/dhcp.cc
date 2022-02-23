/* 
 * MVisor DHCP Server
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
#include "logger.h"
#include <arpa/inet.h>

struct DhcpMessage {
  uint8_t message_type;
  uint8_t hardware_type;
  uint8_t hardware_address_length;
  uint8_t hops;
  uint32_t transaction_id;
  uint16_t seconds_elapsed;
  uint16_t bootp_flags;
  uint32_t client_ip;
  uint32_t your_ip;
  uint32_t next_server_ip;
  uint32_t relay_agent_ip;
  uint8_t client_mac[6];
  uint8_t client_pad[10];
  uint8_t server_host_name[64];
  uint8_t boot_filename[128];
  uint32_t magic_cookie;
  uint8_t option[0];
} __attribute__((packed));


void DhcpServiceUdpSocket::InitializeService(MacAddress router_mac, uint32_t router_ip, uint32_t subnet_mask, uint32_t guest_ip) {
  router_mac_ = router_mac;
  router_ip_ = router_ip;
  subnet_mask_ = subnet_mask;
  guest_ip_ = guest_ip;

  // Load DNS nameservers
  FILE* fp = fopen("/etc/resolv.conf", "r");
  if (fp) {
    while (!feof(fp)) {
      char key[256], val[256];
      if (fscanf(fp, "%s %s\n", key, val) != 2)
        continue;
      if (strncmp("nameserver", key, 10) == 0) {
        struct in_addr addr;
        if (inet_aton(val, &addr)) {
          nameservers_.push_back(ntohl(addr.s_addr));
        }
      }
    }
    fclose(fp);
  } else {
    MV_LOG("warning: /etc/resolv.conf not found");
  }
}

bool DhcpServiceUdpSocket::IsActive() {
  // Kill timedout
  if (time(nullptr) - active_time_ >= REDIRECT_TIMEOUT_SECONDS) {
    return false;
  }
  return true;
}

void DhcpServiceUdpSocket::OnPacketFromGuest(Ipv4Packet* packet) {
  DhcpMessage* dhcp = (DhcpMessage*)packet->data;

  std::string reply;
  if (dhcp->option[2] == 1 && dhcp->option[0] == 53) { // Discover
    reply = CreateDhcpResponse(dhcp, 2);
  } else if (dhcp->option[2] == 3 && dhcp->option[0] == 53) { // Request
    reply = CreateDhcpResponse(dhcp, 5);
  } else {
    DumpHex(dhcp, packet->data_length);
    MV_PANIC("unknown dhcp packet");
    return;
  }

  // Build UDP reply message
  auto reply_packet = AllocatePacket(false);
  if (reply_packet == nullptr) {
    return;
  }
  reply_packet->data_length = reply.size();
  memcpy(reply_packet->data, reply.data(), reply_packet->data_length);

  // DHCP message uses special IPs
  uint32_t sip = sip_, dip = dip_;
  sip_ = 0xFFFFFFFF;
  dip_ = router_ip_;
  OnDataFromHost(reply_packet);
  sip_ = sip;
  dip_ = dip;

  active_time_ = time(nullptr);
}

size_t DhcpServiceUdpSocket::FillDhcpOptions(uint8_t* option, int dhcp_type) {
  uint8_t* p = option;
  // dhcp message type
  *p++ = 53;
  *p++ = 1;
  *p++ = dhcp_type;

  // dhcp server id
  *p++ = 54;
  *p++ = 4;
  *(uint32_t*)p = htonl(router_ip_);
  p += 4;

  // subnet mask
  *p++ = 1;
  *p++ = 4;
  *(uint32_t*)p = htonl(subnet_mask_);
  p += 4;

  // router
  *p++ = 3;
  *p++ = 4;
  *(uint32_t*)p = htonl(router_ip_);
  p += 4;

  // nameserver
  *p++ = 6;
  *p++ = nameservers_.size() * 4;
  for (auto ip : nameservers_) {
    *(uint32_t*)p = htonl(ip);
    p += 4;
  }

  *p++ = 0xFF;
  return p - option;
}

std::string DhcpServiceUdpSocket::CreateDhcpResponse(DhcpMessage* request, int dhcp_type) {
  std::string buffer(512, '\0');
  DhcpMessage* response = (DhcpMessage*)buffer.data();
  size_t body_length = sizeof(DhcpMessage);
  // Copy values before option
  memcpy(response, request, body_length);

  // Set message type to Boot Reply
  response->message_type = 2;
  response->client_ip = 0;

  response->your_ip = htonl(guest_ip_);
  response->next_server_ip = 0;
  response->relay_agent_ip = 0;

  size_t option_length = FillDhcpOptions(response->option, dhcp_type);
  size_t padding = (option_length - 2) % 4;
  if (padding != 0) {
    option_length += (4 - padding);
  }
  buffer.resize(body_length + option_length);
  return std::move(buffer);
}

