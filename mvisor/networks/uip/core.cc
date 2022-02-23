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
#include <mutex>
#include <list>
#include <thread>
#include <arpa/inet.h>
#include <linux/if_arp.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <unistd.h>
#include "utilities.h"
#include "object.h"
#include "device_interface.h"
#include "device.h"
#include "pci_device.h"
#include "device_manager.h"

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
  MacAddress router_mac_;
  uint32_t router_ip_;
  uint32_t router_subnet_mask_;
  uint32_t guest_ip_;
  std::list<TcpSocket*> tcp_sockets_;
  std::list<UdpSocket*> udp_sockets_;
  IoTimer* timer_ = nullptr;
  Device* real_device_ = nullptr;
  std::vector<Ipv4Packet*> queued_packets_;

 public:
  Uip() {
  }

  ~Uip() {
    if (timer_) {
      real_device_->manager()->io()->RemoveTimer(timer_);
    }
  }

  /* UIP Router Configuration
   * Router MAC: 5255C0A80001
   * Router IP: 192.168.0.1
   */
  virtual void Initialize(NetworkDeviceInterface* device, MacAddress& mac) {
    device_ = device;
    guest_mac_ = mac;
    memcpy(router_mac_.data, "\x52\x55\xC0\xA8\x00\x01", ETH_ALEN);

    // Assign IP 192.168.1.1 to machine
    // FIXME: should be configurable
    router_subnet_mask_ = 0xFFFF0000;
    router_ip_ = 0xC0A80001;
    guest_ip_ = 0xC0A80101;

    // This function could only be called once
    MV_ASSERT(real_device_ == nullptr);
    real_device_ = dynamic_cast<Device*>(device_);
    timer_ = real_device_->manager()->io()->AddTimer(10 * 1000, true, [this](){
      OnTimer();
    });
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

  void OnTimer() {
    for (auto it = tcp_sockets_.begin(); it != tcp_sockets_.end();) {
      if (!(*it)->IsActive()) {
        delete *it;
        it = tcp_sockets_.erase(it);
      } else {
        it++;
      }
    }
    for (auto it = udp_sockets_.begin(); it != udp_sockets_.end();) {
      if (!(*it)->IsActive()) {
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
    // fill eth headers
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
    
    RedirectTcpSocket* socket = nullptr;
    
    socket = dynamic_cast<RedirectTcpSocket*>(LookupTcpSocket(sip, dip, sport, dport));
  
    // Guest is trying to start a TCP session
    if (tcp->syn) {
      if (socket == nullptr) {
        socket = new RedirectTcpSocket(this, packet);
        tcp_sockets_.push_back(socket);
      }
      return;
    }
  
    if (socket == nullptr) {
      if (real_device_->debug()) {
        MV_LOG("failed to lookup TCP %x:%u -> %x:%u syn:%d ack:%d rst:%d fin:%d", sip, sport, dip, dport,
          tcp->syn, tcp->ack, tcp->rst, tcp->fin);
      }
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

    UdpSocket* socket = LookupUdpSocket(sip, dip, sport, dport);
    if (socket == nullptr) {
      // Check if it's UDP broadcast
      if (dip == 0xFFFFFFFF || (dip & router_subnet_mask_) == (router_ip_ & router_subnet_mask_)) {
        if (dport == 67) {
          auto dhcp = new DhcpServiceUdpSocket(this, packet);
          dhcp->InitializeService(router_mac_, router_ip_, router_subnet_mask_, guest_ip_);
          socket = dhcp;
        } else {
          if (real_device_->debug()) {
            MV_LOG("unhandled UDP to %x:%d", dip, dport);
          }
          return;
        }
      } else {
        socket = new RedirectUdpSocket(this, packet);
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
