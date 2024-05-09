/* 
 * MVisor User Mode TCP/IP Network
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

#include <deque>

#include <netdb.h>
#include <linux/if_arp.h>
#include <arpa/inet.h>
#include <linux/ip.h>
#include <linux/icmp.h>

#include "ipv4.h"
#include "redirect_icmp.h"
#include "redirect_tcp.h"
#include "redirect_udp.h"
#include "dhcp.h"
#include "map_tcp.h"
#include "object.h"
#include "device.h"
#include "device_interface.h"
#include "device_manager.h"
#include "logger.h"


#define MAX_OPEN_TCP_SOCKETS    1024
#define MAX_OPEN_UDP_SOCKETS    1024
#define MAX_OPEN_ICMP_SOCKETS   128


static std::deque<std::string> Split(std::string input, std::string token) {
  std::deque<std::string> result;
  size_t start = 0;
  while (true) {
    auto token_pos = input.find_first_of(token, start);
    result.emplace_back(input.substr(start, token_pos - start));
    if (token_pos == std::string::npos) {
      break;
    } else {
      start = token_pos + token.length();
    }
  }
  return result;
}


Uip::Uip() {
}

Uip::~Uip() {
  if (timer_) {
    real_device_->RemoveTimer(timer_);
  }
  for (auto fd : map_fds_) {
    real_device_->StopPolling(fd);
    safe_close(&fd);
  }
  /* Release all resources */
  Reset();
}

/* UIP Router Configuration / Router MAC: 5255C0A80001 */
void Uip::Initialize(NetworkDeviceInterface* device, MacAddress& mac) {
  device_ = device;
  guest_mac_ = mac;
  memcpy(router_mac_.data, "\x52\x55\xC0\xA8\x00\x01", ETH_ALEN);
  mtu_ = 1500;

  // Default configuration 192.168.128.1/24
  router_subnet_mask_ = 0xFFFFFF00;
  router_ip_ = 0xC0A88001;

  // This function could only be called once
  MV_ASSERT(real_device_ == nullptr);
  real_device_ = dynamic_cast<PciDevice*>(device_);
  timer_ = real_device_->AddTimer(NS_PER_SECOND * 10, true, [this](){
    OnTimer();
  });

  if (real_device_->has_key("restrict")) {
    restrict_ = std::get<bool>((*real_device_)["restrict"]);
  }

  if (real_device_->has_key("router")) {
    auto router = std::get<std::string>((*real_device_)["router"]);
    // Parse 192.168.2.2/24
    auto slash_pos = router.find('/');
    if (slash_pos != std::string::npos) {
      in_addr ip;
      if (inet_aton(router.substr(0, slash_pos).c_str(), &ip)) {
        router_ip_ = ntohl(ip.s_addr);
        auto mask_bits = atoi(router.substr(slash_pos + 1).c_str());
        router_subnet_mask_ = 0xFFFFFFFF << (32 - mask_bits);
      }
    }
  }

  // Generate guest ip after router ip is determined
  for (uint x = 0x64; x <= 0xF0; x++) {
    guest_ip_ = (router_ip_ & router_subnet_mask_) | x;
    if (guest_ip_ != router_ip_)
      break;
  }

  if (real_device_->has_key("redirect")) {
    auto redirect = std::get<std::string>((*real_device_)["redirect"]);
    // Parse tcp:192.168.2.2:7070-127.0.0.1:7070;udp:192.168.2.2:8000-www.test.com:8000
    for (auto rule : Split(redirect, ";")) {
      ParseRedirectRule(rule);
    }
  }

  if (real_device_->has_key("map")) {
    auto map = std::get<std::string>((*real_device_)["map"]);
    for (auto rule : Split(map, ";")) {
      ParseMapRule(rule);
    }
    StartMapServices();
  }
}

void Uip::SetMtu(int mtu) {
  mtu_ = mtu;
  MV_ASSERT(mtu_ <= 4096);
}

void Uip::Reset() {
  for (auto p : queued_packets_) {
    p->Release();
  }
  queued_packets_.clear();
  
  for (auto it = udp_sockets_.begin(); it != udp_sockets_.end(); it++) {
    delete *it;
  }
  udp_sockets_.clear();
  for (auto it = tcp_sockets_.begin(); it != tcp_sockets_.end(); it++) {
    delete *it;
  }
  tcp_sockets_.clear();
  for (auto it = icmp_sockets_.begin(); it != icmp_sockets_.end(); it++) {
    delete *it;
  }
  icmp_sockets_.clear();
}

// Parse tcp:192.168.2.2:7070
bool Uip::ParseEndpoint(std::string endpoint, uint8_t* protocol, std::string* address, uint16_t* port) {
  auto parts = Split(endpoint, ":");
  if (parts.size() < 2 || parts.size() > 3) {
    MV_PANIC("Invalid endpoint %s", endpoint.c_str());
    return false;
  }

  *protocol = 0;
  if (parts.size() == 3) {
    if (parts[0] == "tcp") {
      *protocol = 0x06;
    } else if (parts[0] == "udp") {
      *protocol = 0x11;
    } else {
      MV_ERROR("unknown protocol %s", parts[0].c_str());
    }
    parts.pop_front();
  }

  *address = parts[0];
  *port = atoi(parts[1].c_str());
  return true;
}

// Parse tcp:192.168.2.2:7070-127.0.0.1:7070
void Uip::ParseRedirectRule(std::string input) {
  auto endpoints = Split(input, "-");
  MV_ASSERT(endpoints.size() == 2);
  
  RedirectRule rule;
  std::string address;
  uint8_t protocol;
  uint16_t port;
  if (!ParseEndpoint(endpoints[0], &protocol, &address, &port)) {
    return;
  }

  rule.protocol = protocol;
  rule.match_ip = ntohl(inet_addr(address.c_str()));
  rule.match_port = port;
  if (!ParseEndpoint(endpoints[1], &protocol, &address, &port)) {
    return;
  }
  auto entry = gethostbyname2(address.c_str(), AF_INET);
  if (!entry) {
    MV_LOG("failed to resolve name %s", address.c_str());
    return;
  }
  rule.target_ip = ntohl(*(in_addr_t*)entry->h_addr_list[0]);
  rule.target_port = port;
  redirect_rules_.push_back(rule);
}

// Parse tcp:0.0.0.0:8080-:7070
void Uip::ParseMapRule(std::string input) {
  auto endpoints = Split(input, "-");
  MV_ASSERT(endpoints.size() == 2);

  MapRule rule;
  std::string address;
  uint8_t protocol;
  uint16_t port;
  if (!ParseEndpoint(endpoints[0], &protocol, &address, &port)) {
    return;
  }

  /* Currently only TCP service is allowed */
  MV_ASSERT(protocol == 0x06);
  rule.protocol = protocol;
  if (!address.empty()) {
    rule.listen_ip = ntohl(inet_addr(address.c_str()));
  } else {
    rule.listen_ip = INADDR_ANY;
  }
  rule.listen_port = port;
  if (!ParseEndpoint(endpoints[1], &protocol, &address, &port)) {
    return;
  }
  if (!address.empty()) {
    rule.target_ip = ntohl(inet_addr(address.c_str()));
  } else {
    rule.target_ip = guest_ip_;
  }
  rule.target_port = port;
  map_rules_.push_back(rule);
}

void Uip::StartMapServices() {
  for (auto& rule : map_rules_) {
    sockaddr_in addr = {
      .sin_family = AF_INET,
      .sin_port = htons(rule.listen_port),
      .sin_addr = {
        .s_addr = htonl(rule.listen_ip)
      },
      .sin_zero = {0}
    };

    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    MV_ASSERT(fd >= 0);

    // Set socket reuse flag
    int flag = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag));

    if (bind(fd, (sockaddr*)&addr, sizeof(addr)) != 0) {
      MV_ERROR("failed to bind address %s:%d", inet_ntoa(addr.sin_addr), rule.listen_port);
      safe_close(&fd);
      continue;
    }
    MV_ASSERT(listen(fd, 128) == 0);
    map_fds_.push_back(fd);
    if (real_device_->debug()) {
      MV_LOG("listen ip=0x%x port=%d fd=%d", rule.listen_ip, rule.listen_port, fd);
    }

    real_device_->StartPolling(fd, EPOLLIN, [this, fd, rule](auto events) {
      if (events & EPOLLIN) {
        sockaddr_in addr;
        socklen_t addr_len = sizeof(addr);
        int conn_fd = accept4(fd, (sockaddr*)&addr, &addr_len, SOCK_NONBLOCK);
        if (conn_fd != -1) {
          if (addr.sin_addr.s_addr == htonl(INADDR_LOOPBACK)) {
            addr.sin_addr.s_addr = htonl(router_ip_);
          }
          auto sock = new MapTcpSocket(this, rule.target_ip, ntohl(addr.sin_addr.s_addr),
            rule.target_port, ntohs(addr.sin_port), conn_fd);
          tcp_sockets_.push_back(sock);
        }
      } else {
        MV_LOG("invalid events=0x%x", events);
      }
    });
  }
}

void Uip::OnTimer() {
  for (auto it = tcp_sockets_.begin(); it != tcp_sockets_.end();) {
    if (!(*it)->active()) {
      delete *it;
      it = tcp_sockets_.erase(it);
    } else {
      it++;
    }
  }
  for (auto it = udp_sockets_.begin(); it != udp_sockets_.end();) {
    if (!(*it)->active()) {
      delete *it;
      it = udp_sockets_.erase(it);
    } else {
      it++;
    }
  }
  for (auto it = icmp_sockets_.begin(); it != icmp_sockets_.end();) {
    if (!(*it)->active()) {
      delete *it;
      it = icmp_sockets_.erase(it);
    } else {
      it++;
    }
  }
  if (real_device_->debug()) {
    MV_LOG("sockets tcp=%lu udp=%lu icmp=%lu", tcp_sockets_.size(), udp_sockets_.size(), icmp_sockets_.size());
  }
}

void Uip::OnReceiveAvailable() {
  while (!queued_packets_.empty()) {
    auto packet = queued_packets_.back();
    queued_packets_.pop_back();
    if (!OnPacketFromHost(packet)) {
      return;
    }
  }

  for (auto it = tcp_sockets_.begin(); it != tcp_sockets_.end() && queued_packets_.empty(); it++) {
    if ((*it)->active()) {
      (*it)->OnGuestBufferAvaialble();
    }
  }

  for (auto it = udp_sockets_.begin(); it != udp_sockets_.end() && queued_packets_.empty(); it++) {
    if ((*it)->active()) {
      (*it)->OnGuestBufferAvaialble();
    }
  }

  for (auto it = icmp_sockets_.begin(); it != icmp_sockets_.end() && queued_packets_.empty(); it++) {
    if ((*it)->active()) {
      (*it)->OnGuestBufferAvaialble();
    }
  }
}

void Uip::OnFrameFromGuest(std::deque<iovec>& vector) {
  size_t buffer_size = 0;
  for (auto &v : vector) {
    buffer_size += v.iov_len;
  }

  auto packet = new Ipv4Packet;
  packet->buffer = new uint8_t[buffer_size];
  packet->Release = [packet]() {
    delete[] packet->buffer;
    delete packet;
  };
  size_t copied = 0;
  for (auto &v : vector) {
    memcpy(packet->buffer + copied, v.iov_base, v.iov_len);
    copied += v.iov_len;
  }

  packet->vnet = (virtio_net_hdr_v1*)packet->buffer;
  packet->eth = (ethhdr*)&packet->vnet[1];
  ParseEthPacket(packet);
}

bool Uip::OnPacketFromHost(Ipv4Packet* packet) {
  if (packet->offload_checksum) {
    packet->vnet->flags |= VIRTIO_NET_HDR_F_DATA_VALID;
  }
  memcpy(packet->eth->h_dest, guest_mac_.data, sizeof(packet->eth->h_dest));
  memcpy(packet->eth->h_source, router_mac_.data, sizeof(packet->eth->h_source));
  size_t buffer_length = sizeof(*packet->vnet) + sizeof(*packet->eth);
  if (packet->eth->h_proto == htons(ETH_P_IP)) {
    buffer_length += ntohs(packet->ip->tot_len);
  } else if (packet->eth->h_proto == htons(ETH_P_ARP)) {
    buffer_length += sizeof(ArpMessage);
  } else {
    MV_WARN("Unknown ethernet packet protocol=%x", ntohs(packet->eth->h_proto));
  }
  
  if(device_->WriteBuffer(packet->buffer, buffer_length)) {
    packet->Release();
    return true;
  } else {
    queued_packets_.push_back(packet);
    return false;
  }
}

/* when return nullptr, socket should retry later */
Ipv4Packet* Uip::AllocatePacket(bool urgent) {
  if (!urgent && !queued_packets_.empty()) {
    return nullptr;
  }
  Ipv4Packet* packet = new Ipv4Packet;
  MV_ASSERT(packet);
  packet->offload_checksum = device_->offload_checksum();
  packet->offload_segmentation = device_->offload_segmentation();
  packet->mtu = mtu_;
  packet->buffer_size = packet->offload_segmentation ? 65536 : packet->mtu;
  packet->buffer = new uint8_t[packet->buffer_size + 30];
  packet->vnet = (virtio_net_hdr_v1*)packet->buffer;
  bzero(packet->vnet, sizeof(*packet->vnet));
  packet->eth = (ethhdr*)&packet->vnet[1];
  packet->eth->h_proto = htons(ETH_P_IP);
  packet->ip = (iphdr*)&packet->eth[1];
  packet->data = (void*)&packet->ip[1];
  packet->tcp = nullptr;
  packet->udp = nullptr;
  packet->icmp = nullptr;
  packet->data_length = 0;
  packet->Release = [packet]() {
    delete[] packet->buffer;
    delete packet;
  };
  return packet;
}

void Uip::ParseEthPacket(Ipv4Packet* packet) {
  auto eth = packet->eth;
  if (memcmp(eth->h_dest, router_mac_.data, ETH_ALEN) != 0 &&
    memcmp(eth->h_dest, "\xFF\xFF\xFF\xFF\xFF\xFF", ETH_ALEN) != 0) {
    // ignore packets to other ethernet addresses
    return;
  }

  uint16_t protocol = ntohs(eth->h_proto);
  switch (protocol)
  {
  case ETH_P_IP:
    packet->ip = (struct iphdr*)&eth[1];
    ParseIpPacket(packet);
    break;
  case ETH_P_ARP:
    ParseArpPacket(packet);
    break;
  case ETH_P_IPV6:
    if (real_device_->debug()) {
      MV_WARN("ignore IPv6");
    }
    packet->Release();
    break;
  case ETH_P_LLDP:
    if (real_device_->debug()) {
      MV_WARN("ignore LLDP");
    }
    packet->Release();
    break;
  case ETH_P_PPP_DISC:
    if (real_device_->debug()) {
      MV_LOG("ignore PPP_DISC");
    }
    packet->Release();
    break;
  default:
    MV_WARN("Unknown ethernet packet protocol=%x", protocol);
    packet->Release();
    break;
  }
}

void Uip::ParseArpPacket(Ipv4Packet* packet) {
  auto arp = (ArpMessage*)&packet->eth[1];
  if (ntohs(arp->ar_op) == 1) { // ARP request
    uint32_t dip = ntohl(arp->ar_tip);
    if (dip == router_ip_) {
      ArpMessage request = *arp;
      // ARP reply
      arp->ar_op = htons(2);
      memcpy(arp->ar_tha, &request.ar_sha, ETH_ALEN);
      arp->ar_tip = request.ar_sip;
      memcpy(arp->ar_sha, router_mac_.data, ETH_ALEN);
      arp->ar_sip = htonl(router_ip_);
      OnPacketFromHost(packet);
      return;
    }
  } else {
    MV_ERROR("Invalid Arp op=0x%x", arp->ar_op);
  }
  packet->Release();
}

void Uip::ParseIpPacket(Ipv4Packet* packet) {
  auto ip = packet->ip;
  switch (ip->protocol)
  {
  case 0x01:  // ICMP
    packet->icmp = (struct icmphdr*)((uint8_t*)ip + ip->ihl * 4);
    if (ParseIcmpPacket(packet)) {
      packet->Release();
    }
    break;
  case 0x06:  // TCP
    packet->tcp = (struct tcphdr*)((uint8_t*)ip + ip->ihl * 4);
    if (ParseTcpPacket(packet)) {
      packet->Release();
    }
    break;
  case 0x11:  // UDP
    packet->udp = (struct udphdr*)((uint8_t*)ip + ip->ihl * 4);
    if (ParseUdpPacket(packet)) {
      packet->Release();
    }
    break;
  default:
    if (real_device_->debug()) {
      MV_LOG("Not UDP or TCP, protocol=%d", ip->protocol);
      MV_HEXDUMP("ip packet", ip, ntohs(ip->tot_len));
    }
    packet->Release();
    break;
  }
}

IcmpSocket* Uip::LookupIcmpSocket(uint32_t sip, uint32_t dip, uint16_t echo_id) {
  for (auto socket : icmp_sockets_) {
    if (socket->Equals(sip, dip, echo_id)) {
      return socket;
    }
  }
  return nullptr;
}

bool Uip::ParseIcmpPacket(Ipv4Packet* packet) {
  auto ip = packet->ip;
  auto icmp = packet->icmp;
  auto sip = ntohl(ip->saddr);
  auto dip = ntohl(ip->daddr);

  /* For network security, only PING is supported */
  if (icmp->type != ICMP_ECHO) {
    return true;
  }

  auto echo_id = ntohs(icmp->un.echo.id);
  auto socket = dynamic_cast<RedirectIcmpSocket*>(LookupIcmpSocket(sip, dip, echo_id));
  if (socket == nullptr) {
    /* If restricted and not local network, don't redirect packets */
    if ((dip & router_subnet_mask_) != (router_ip_ & router_subnet_mask_) && restrict_) {
      return true;
    }
    if (icmp_sockets_.size() >= MAX_OPEN_ICMP_SOCKETS) {
      return true;
    }

    auto icmp_socket = new RedirectIcmpSocket(this, sip, dip, echo_id);
    icmp_socket->InitializeRedirect();
    socket = icmp_socket;
    icmp_sockets_.push_back(socket);
  }

  packet->data = icmp;
  packet->data_offset = 0;
  packet->data_length = ntohs(ip->tot_len) - ip->ihl * 4;
  socket->OnPacketFromGuest(packet);
  return false;
}

TcpSocket* Uip::LookupTcpSocket(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport) {
  for (auto socket : tcp_sockets_) {
    if (socket->Equals(sip, dip, sport, dport)) {
      return socket;
    }
  }
  return nullptr;
}

bool Uip::ParseTcpPacket(Ipv4Packet* packet) {
  auto ip = packet->ip;
  auto tcp = packet->tcp;
  auto sip = ntohl(ip->saddr);
  auto dip = ntohl(ip->daddr);
  auto sport = ntohs(tcp->source);
  auto dport = ntohs(tcp->dest);
  
  auto socket = dynamic_cast<RedirectTcpSocket*>(LookupTcpSocket(sip, dip, sport, dport));
  if (socket == nullptr) {
    /* If restricted and not local network, don't redirect packets */
    if ((dip & router_subnet_mask_) != (router_ip_ & router_subnet_mask_) && restrict_) {
      return true;
    }
    if (tcp_sockets_.size() >= MAX_OPEN_TCP_SOCKETS) {
      return true;
    }

    socket = new RedirectTcpSocket(this, sip, dip, sport, dport);
    tcp_sockets_.push_back(socket);
  }

  // Guest is trying to start a TCP session
  if (tcp->syn) {
    socket->InitializeRedirect(packet);
    return true;
  }

  // If not connected, other packets will reset the connection
  if (!socket->connected()) {
    if (real_device_->debug()) {
      MV_LOG("Reset TCP %x:%u -> %x:%u syn:%d ack:%d rst:%d fin:%d", sip, sport, dip, dport,
        tcp->syn, tcp->ack, tcp->rst, tcp->fin);
    }
    socket->ReplyReset(packet);
    return true;
  }

  if (tcp->rst) {
    socket->Reset();
    return true;
  }

  // ACK is always set if not SYN or FIN or RST
  if (!tcp->ack) {
    return true;
  }

  // If send window buffer is full, try again when new guest ack comes
  if (!socket->UpdateGuestAck(tcp)) {
    return true;
  }

  // Send out TCP data to remote host
  packet->data = (uint8_t*)tcp + tcp->doff * 4;
  packet->data_offset = 0;
  packet->data_length = ntohs(ip->tot_len) - ip->ihl * 4 - tcp->doff * 4;
  socket->OnPacketFromGuest(packet);
  return false;
}

UdpSocket* Uip::LookupUdpSocket(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport) {
  for (auto socket : udp_sockets_) {
    if (socket->Equals(sip, dip, sport, dport)) {
      return socket;
    }
  }
  return nullptr;
}

bool Uip::ParseUdpPacket(Ipv4Packet* packet) {
  auto ip = packet->ip;
  auto udp = packet->udp;
  auto sip = ntohl(ip->saddr);
  auto dip = ntohl(ip->daddr);
  auto sport = ntohs(udp->source);
  auto dport = ntohs(udp->dest);

  auto socket = LookupUdpSocket(sip, dip, sport, dport);
  if (socket == nullptr) {
    if (udp_sockets_.size() >= MAX_OPEN_UDP_SOCKETS) {
      return true;
    }

    // Check if it's UDP broadcast
    if (dport == 67) {
      socket = new DhcpServiceUdpSocket(this, sip, dip, sport, dport);
    } else {
      if ((dip & router_subnet_mask_) != (router_ip_ & router_subnet_mask_) && restrict_) {
        /* If restricted and not local network, don't redirect UDP packets */
        return true;
      }
      auto redirect_udp = new RedirectUdpSocket(this, sip, dip, sport, dport);
      redirect_udp->InitializeRedirect();
      socket = redirect_udp;
    }
    udp_sockets_.push_back(socket);
  }

  packet->data = &udp[1];
  packet->data_offset = 0;
  packet->data_length = ntohs(udp->len) - sizeof(*udp);
  socket->OnPacketFromGuest(packet);
  return false;
}

DECLARE_NETWORK(Uip);
