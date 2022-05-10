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

#include <list>

#include <netdb.h>
#include <arpa/inet.h>
#include <linux/if_arp.h>

#include "object.h"
#include "device.h"
#include "device_interface.h"
#include "device_manager.h"
#include "logger.h"

struct ArpMessage {
  uint16_t    ar_hrd;    /* format of hardware address  */
  uint16_t    ar_pro;    /* format of protocol address  */
  uint8_t     ar_hln;    /* length of hardware address  */
  uint8_t     ar_pln;    /* length of protocol address  */
  uint16_t    ar_op;     /* ARP opcode (command)    */

  /*
  * Ethernet looks like this : This bit is variable sized however...
  */
  uint8_t     ar_sha[ETH_ALEN];  /* sender hardware address  */
  uint32_t    ar_sip;            /* sender IP address    */
  uint8_t     ar_tha[ETH_ALEN];  /* target hardware address  */
  uint32_t    ar_tip;            /* target IP address    */
} __attribute__((packed));

class Uip : public Object, public NetworkBackendInterface {
 private:
  std::list<TcpSocket*> tcp_sockets_;
  std::list<UdpSocket*> udp_sockets_;
  IoTimer*              timer_ = nullptr;
  Device*               real_device_ = nullptr;
  std::vector<Ipv4Packet*> queued_packets_;
  uint                  mtu_;
  bool                  restrict_ = false;

 public:
  Uip() {
  }

  ~Uip() {
    if (timer_) {
      real_device_->manager()->io()->RemoveTimer(timer_);
    }
    /* Release all resources */
    Reset();
  }

  /* UIP Router Configuration
   * Router MAC: 5255C0A80001
   * Router IP: 192.168.128.1
   */
  virtual void Initialize(NetworkDeviceInterface* device, MacAddress& mac, int mtu) {
    device_ = device;
    guest_mac_ = mac;
    memcpy(router_mac_.data, "\x52\x55\xC0\xA8\x00\x01", ETH_ALEN);
    mtu_ = mtu;
    MV_ASSERT(mtu_ + 16 <= 4096);

    // Default configuration 192.168.128.1/24
    router_subnet_mask_ = 0xFFFFFF00;
    router_ip_ = 0xC0A88001;

    // This function could only be called once
    MV_ASSERT(real_device_ == nullptr);
    real_device_ = dynamic_cast<Device*>(device_);
    timer_ = real_device_->manager()->io()->AddTimer(10 * 1000, true, [this](){
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

    if (real_device_->has_key("redirect")) {
      auto redirect = std::get<std::string>((*real_device_)["redirect"]);
      ParseRedirectRules(redirect);
    }
  }

  virtual void Reset() {
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
  }

  // Parse tcp:192.168.2.2:7070
  bool ParseEndpoint(std::string endpoint, uint8_t* protocol, std::string* address, uint16_t* port) {
    auto first_pos = endpoint.find_first_of(':');
    auto last_pos = endpoint.find_last_of(':');
    if (first_pos == std::string::npos || last_pos == std::string::npos) {
      return false;
    }

    *protocol = 0;
    if (first_pos != last_pos) {
      auto protocol_string = endpoint.substr(0, first_pos);
      if (protocol_string == "tcp") {
        *protocol = 0x06;
      } else if (protocol_string == "udp") {
        *protocol = 0x11;
      } else {
        MV_LOG("unknown protocol %s", protocol_string.c_str());
      }
      first_pos += 1;
    } else {
      first_pos = 0;
    }

    *address = endpoint.substr(first_pos, last_pos - first_pos);
    std::string port_string = endpoint.substr(last_pos + 1);
    *port = atoi(port_string.c_str());
    return true;
  }

  // Parse tcp:192.168.2.2:7070-127.0.0.1:7070
  void ParseRedirectRule(std::string input) {
    auto slash_pos = input.find('-');
    if (slash_pos == std::string::npos)
      return;
    
    RedirectRule rule;
    std::string address;
    uint8_t protocol;
    uint16_t port;
    if (!ParseEndpoint(input.substr(0, slash_pos), &protocol, &address, &port)) {
      return;
    }

    rule.protocol = protocol;
    rule.match_ip = ntohl(inet_addr(address.c_str()));
    rule.match_port = port;
    if (!ParseEndpoint(input.substr(slash_pos + 1), &protocol, &address, &port)) {
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

  // Parse tcp:192.168.2.2:7070-127.0.0.1:7070;udp:192.168.2.2:8000-www.test.com:8000
  void ParseRedirectRules(std::string rules) {
    size_t start = 0;
    while (true) {
      auto colon_pos = rules.find_first_of(';', start);
      if (colon_pos > start) {
        ParseRedirectRule(rules.substr(start, colon_pos));
      }
      if (colon_pos == std::string::npos) {
        break;
      } else {
        start = colon_pos + 1;
      }
    }
  }

  void OnTimer() {
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
    if (real_device_->debug()) {
      MV_LOG("tcp_sockets.size=%lu udp_sockets.size=%lu", tcp_sockets_.size(), udp_sockets_.size());
    }
  }

  virtual void OnReceiveAvailable() {
    while (!queued_packets_.empty()) {
      auto packet = queued_packets_.back();
      queued_packets_.pop_back();
      if (!OnPacketFromHost(packet))
        break;
    }
  }

  virtual void OnFrameFromGuest(std::deque<iovec>& vector) {
    auto packet = new Ipv4Packet;
    packet->Release = [packet]() {
      delete packet;
    };
    size_t copied = 0;
    for (auto &v : vector) {
      memcpy(packet->buffer + copied, v.iov_base, v.iov_len);
      copied += v.iov_len;
    }

    packet->eth = (ethhdr*)packet->buffer;
    ParseEthPacket(packet);
  }

  bool OnFrameFromHost(uint16_t protocol, void* buffer, size_t size) {
    /* Ethernet header is filled here */
    ethhdr* eth = (ethhdr*)buffer;
    eth->h_proto = htons(protocol);
    memcpy(eth->h_dest, guest_mac_.data, sizeof(eth->h_dest));
    memcpy(eth->h_source, router_mac_.data, sizeof(eth->h_source));

    return device_->WriteBuffer(buffer, size);
  }

  virtual bool OnPacketFromHost(Ipv4Packet* packet) {
    size_t packet_length = sizeof(ethhdr) + ntohs(packet->ip->tot_len);
    if (OnFrameFromHost(ETH_P_IP, packet->buffer, packet_length)) {
      packet->Release();
      return true;
    } else {
      queued_packets_.push_back(packet);
      return false;
    }
  }

  /* when return nullptr, socket should retry later */
  virtual Ipv4Packet* AllocatePacket(bool urgent) {
    if (!urgent && !queued_packets_.empty()) {
      return nullptr;
    }
    Ipv4Packet* packet = new Ipv4Packet;
    packet->mtu = mtu_;
    packet->eth = (ethhdr*)packet->buffer;
    packet->ip = (iphdr*)&packet->eth[1];
    packet->data = (void*)&packet->ip[1];
    packet->tcp = nullptr;
    packet->udp = nullptr;
    packet->data_length = 0;
    packet->Release = [packet]() {
      delete packet;
    };
    return packet;
  }

  void ParseEthPacket(Ipv4Packet* packet) {
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
      ParseArpPacket(eth, (ArpMessage*)&eth[1]);
      break;
    case ETH_P_IPV6:
      // ignore IPv6
      MV_LOG("ignore IPv6");
      break;
    case ETH_P_LLDP:
      // ignore LLDP
      MV_LOG("ignore LLDP");
      break;
    default:
      MV_PANIC("Unknown ethernet packet protocol=%x", protocol);
      break;
    }
  }

  void ParseArpPacket(ethhdr* eth, ArpMessage* arp) {
    if (ntohs(arp->ar_op) == 1) { // ARP request
      uint32_t dip = ntohl(arp->ar_tip);
      if (dip == router_ip_) {
        // ARP reply
        uint8_t buffer[42];
        ArpMessage* reply = (ArpMessage*)&buffer[14];
        reply->ar_hrd = arp->ar_hrd;
        reply->ar_pro = arp->ar_pro;
        reply->ar_hln = arp->ar_hln;
        reply->ar_pln = arp->ar_pln;
        reply->ar_op = htons(2);
        memcpy(reply->ar_tha, arp->ar_sha, ETH_ALEN);
        reply->ar_tip = arp->ar_sip;
        memcpy(reply->ar_sha, router_mac_.data, ETH_ALEN);
        reply->ar_sip = htonl(router_ip_);
        OnFrameFromHost(ETH_P_ARP, buffer, sizeof(buffer));
      }
    } else {
      MV_PANIC("Invalid Arp op=0x%x", arp->ar_op);
    }
  }

  void ParseIpPacket(Ipv4Packet* packet) {
    auto ip = packet->ip;
    switch (ip->protocol)
    {
    case 0x06: { // TCP
      packet->tcp = (struct tcphdr*)((uint8_t*)ip + ip->ihl * 4);
      ParseTcpPacket(packet);
      break;
    }
    case 0x11: { // UDP
      packet->udp = (struct udphdr*)((uint8_t*)ip + ip->ihl * 4);
      ParseUdpPacket(packet);
      break;
    }
    default:
      if (real_device_->debug()) {
        MV_LOG("==================Not UDP or TCP===================");
        MV_LOG("ip packet protocol=%d", ip->protocol);
        DumpHex(ip, ntohs(ip->tot_len));
      }
      break;
    }
  }

  TcpSocket* LookupTcpSocket(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport) {
    for (auto socket : tcp_sockets_) {
      if (socket->Equals(sip, dip, sport, dport)) {
        return socket;
      }
    }
    return nullptr;
  }

  void ParseTcpPacket(Ipv4Packet* packet) {
    auto ip = packet->ip;
    auto tcp = packet->tcp;
    uint32_t sip = ntohl(ip->saddr);
    uint32_t dip = ntohl(ip->daddr);
    uint16_t sport = ntohs(tcp->source);
    uint16_t dport = ntohs(tcp->dest);
    
    auto socket = dynamic_cast<RedirectTcpSocket*>(LookupTcpSocket(sip, dip, sport, dport));
    if (socket == nullptr) {
      /* If restricted and not local network, don't redirect packets */
      if ((dip & router_subnet_mask_) != (router_ip_ & router_subnet_mask_) && restrict_) {
        return;
      }
      socket = new RedirectTcpSocket(this, packet);
      tcp_sockets_.push_back(socket);
    }

    // Guest is trying to start a TCP session
    if (tcp->syn) {
      socket->InitializeRedirect(packet);
      return;
    }

    // If not connected, other packets will reset the connection
    if (!socket->connected()) {
      if (real_device_->debug()) {
        MV_LOG("Reset TCP %x:%u -> %x:%u syn:%d ack:%d rst:%d fin:%d", sip, sport, dip, dport,
          tcp->syn, tcp->ack, tcp->rst, tcp->fin);
      }
      socket->Reset(packet);
      return;
    }
  
    // Guest is closing the TCP
    if (tcp->fin) {
      socket->Shutdown(SHUT_WR);
      return;
    }

    // ACK is always set if not SYN or FIN or RST
    if (!tcp->ack) {
      return;
    }

    // If send window buffer is full, try again when new guest ack comes
    if (!socket->UpdateGuestAck(tcp)) {
      return;
    }

    // Send out TCP data to remote host
    packet->data = (uint8_t*)tcp + tcp->doff * 4;
    packet->data_offset = 0;
    packet->data_length = ntohs(ip->tot_len) - ip->ihl * 4 - tcp->doff * 4;
    if (packet->data_length == 0) {
      return;
    }
    socket->OnPacketFromGuest(packet);
  }

  UdpSocket* LookupUdpSocket(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport) {
    for (auto socket : udp_sockets_) {
      if (socket->Equals(sip, dip, sport, dport)) {
        return socket;
      }
    }
    return nullptr;
  }

  void ParseUdpPacket(Ipv4Packet* packet) {
    auto ip = packet->ip;
    auto udp = packet->udp;
    uint32_t sip = ntohl(ip->saddr);
    uint32_t dip = ntohl(ip->daddr);
    uint16_t sport = ntohs(udp->source);
    uint16_t dport = ntohs(udp->dest);

    auto socket = LookupUdpSocket(sip, dip, sport, dport);
    if (socket == nullptr) {
      // Check if it's UDP broadcast
      if (dport == 67) {
        auto dhcp = new DhcpServiceUdpSocket(this, packet);
        dhcp->InitializeService(router_mac_, router_ip_, router_subnet_mask_);
        socket = dhcp;
      } else {
        if ((dip & router_subnet_mask_) != (router_ip_ & router_subnet_mask_) && restrict_) {
          /* If restricted and not local network, don't redirect UDP packets */
          return;
        }
        auto redirect_udp = new RedirectUdpSocket(this, packet);
        redirect_udp->InitializeRedirect();
        socket = redirect_udp;
      }
      udp_sockets_.push_back(socket);
    }

    MV_ASSERT(socket);
    packet->data = &udp[1];
    packet->data_offset = 0;
    packet->data_length = ntohs(udp->len) - sizeof(*udp);
    socket->OnPacketFromGuest(packet);
  }

};

DECLARE_NETWORK(Uip);
