/* 
 * MVisor User Mode Network
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

#ifndef _MVISOR_UIP_H
#define _MVISOR_UIP_H

#include <cstdint>
#include <string>
#include <vector>
#include <list>

#include <linux/if_ether.h>

#include "device_interface.h"
#include "pci_device.h"

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

struct RedirectRule {
  uint        protocol;
  uint32_t    match_ip;
  uint16_t    match_port;
  uint32_t    target_ip;
  uint16_t    target_port;
};

struct MapRule {
  uint        protocol;
  uint32_t    listen_ip;
  uint16_t    listen_port;
  uint32_t    target_ip;
  uint16_t    target_port;
};

struct Ipv4Packet;
class TcpSocket;
class UdpSocket;
class IcmpSocket;
class Uip : public Object, public NetworkBackendInterface {
 private:
  std::list<TcpSocket*>     tcp_sockets_;
  std::list<UdpSocket*>     udp_sockets_;
  std::list<IcmpSocket*>    icmp_sockets_;
  IoTimer*                  timer_ = nullptr;
  PciDevice*                real_device_ = nullptr;
  std::vector<Ipv4Packet*>  queued_packets_;
  uint                      mtu_;
  bool                      restrict_ = false;
  std::vector<int>          map_fds_;
  MacAddress                guest_mac_;
  MacAddress                router_mac_;
  uint32_t                  router_ip_;
  uint32_t                  router_subnet_mask_;
  uint32_t                  guest_ip_;
  std::vector<RedirectRule> redirect_rules_;
  std::vector<MapRule>      map_rules_;

 public:
  Uip();
  virtual ~Uip();
  virtual void Initialize(NetworkDeviceInterface* device, MacAddress& mac) override;
  virtual void SetMtu(int mtu) override;
  virtual void Reset() override;
  virtual void OnReceiveAvailable() override;
  virtual void OnFrameFromGuest(std::deque<iovec>& vector) override;
  virtual Ipv4Packet* AllocatePacket(bool urgent);
  virtual bool OnPacketFromHost(Ipv4Packet* packet);

  inline MacAddress& router_mac() { return router_mac_; }
  inline uint32_t router_ip() { return router_ip_; }
  inline uint32_t router_subnet_mask() { return router_subnet_mask_; }
  inline uint32_t guest_ip() { return guest_ip_; }
  inline const std::vector<RedirectRule>& redirect_rules() const { return redirect_rules_; }
  inline const std::vector<MapRule>& map_rules() const { return map_rules_; }

 private:
  bool ParseEndpoint(std::string endpoint, uint8_t* protocol, std::string* address, uint16_t* port);
  void ParseRedirectRule(std::string input);
  void ParseMapRule(std::string input);
  void StartMapServices();
  void OnTimer();
  void ParseEthPacket(Ipv4Packet* packet);
  void ParseArpPacket(Ipv4Packet* packet);
  void ParseIpPacket(Ipv4Packet* packet);
  IcmpSocket* LookupIcmpSocket(uint32_t sip, uint32_t dip, uint16_t echo_id);
  bool ParseIcmpPacket(Ipv4Packet* packet);
  TcpSocket* LookupTcpSocket(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport);
  bool ParseTcpPacket(Ipv4Packet* packet);
  UdpSocket* LookupUdpSocket(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport);
  bool ParseUdpPacket(Ipv4Packet* packet);
};


#endif // _MVISOR_UIP_H
