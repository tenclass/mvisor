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

#ifndef _MVISOR_NETWORKS_USER_H
#define _MVISOR_NETWORKS_USER_H

#include "device_interface.h"
#include <string>
#include <vector>
#include <linux/if_ether.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/tcp.h>
#include <ctime>

#define UIP_MAX_BUFFER_SIZE (64*1024 + 16)
#define UIP_MAX_UDP_PAYLOAD (64*1024 - 20 - 8)
#define UIP_MAX_TCP_PAYLOAD (64*1024 - 144)

#define REDIRECT_TIMEOUT_SECONDS (120)

struct PseudoHeader {
  uint32_t sip;
  uint32_t dip;
  uint8_t zero;
  uint8_t protocol;
  uint16_t length;
} __attribute__((packed));


struct Ipv4Packet {
  uint8_t*  buffer;
  ethhdr*   eth;
  iphdr*    ip;
  udphdr*   udp;
  tcphdr*   tcp;
  void*     data;
  size_t    data_length;
};

class Ipv4Socket {
 public:
  Ipv4Socket(NetworkBackendInterface* backend, ethhdr* eth, iphdr* ip);
  virtual ~Ipv4Socket() {}
  virtual bool IsActive();
  
 protected:
  virtual Ipv4Packet* AllocatePacket();
  virtual void FreePacket(Ipv4Packet* packet);
  uint16_t CalculateChecksum(uint8_t* addr, uint16_t count);

  NetworkBackendInterface* backend_;
  uint32_t sip_;
  uint32_t dip_;
  bool closed_;
  time_t active_time_;
  bool debug_;
};


class TcpSocket : public Ipv4Socket {
 public:
  TcpSocket(NetworkBackendInterface* backend, ethhdr* eth, iphdr* ip, tcphdr* tcp);
  bool UpdateGuestAck(tcphdr* tcp);
   
  inline bool Equals(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport) {
    return sip_ == sip && dip_ == dip && sport_ == sport && dport_ == dport;
  }
  virtual void OnDataFromGuest(void* data, size_t length) = 0;

 protected:
  virtual Ipv4Packet* AllocatePacket();
  uint16_t CalculateTcpChecksum(Ipv4Packet* packet);
  void OnDataFromHost(Ipv4Packet* packet, uint32_t tcp_flags);
  void ParseTcpOptions(tcphdr* tcp);
  void FillTcpOptions(tcphdr* tcp);

  uint16_t sport_;
  uint16_t dport_;
  uint32_t window_size_;
  uint32_t guest_acked_;
  /* Initial sequence number */
  uint32_t isn_host_;
  uint32_t isn_guest_;
  uint32_t ack_host_;
  uint32_t seq_host_;
  uint16_t mss_;
  uint8_t  window_scale_;
  bool     sack_permitted_;
};

class UdpSocket : public Ipv4Socket {
 public:
  UdpSocket(NetworkBackendInterface* backend, ethhdr* eth, iphdr* ip, udphdr* udp);

  inline bool Equals(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport) {
    return sip_ == sip && dip_ == dip && sport_ == sport && dport_ == dport;
  }
  virtual void OnDataFromGuest(void* data, size_t length) = 0;

 protected:
  virtual Ipv4Packet* AllocatePacket();
  uint16_t CalculateUdpChecksum(Ipv4Packet* packet);
  void OnDataFromHost(Ipv4Packet* packet);

  uint16_t sport_;
  uint16_t dport_;
};

class RedirectTcpSocket : public TcpSocket {
 public:
  RedirectTcpSocket(NetworkBackendInterface* backend, ethhdr* eth, iphdr* ip, tcphdr* tcp);
  virtual ~RedirectTcpSocket();
  void Shutdown(int how);
  void OnDataFromGuest(void* data, size_t length);
  void OnRemoteDataAvailable();
  bool IsGuestOverflow() { return guest_overflow_; }
  bool IsConnected() { return connected_; }
  virtual bool IsActive();

 protected:
  void InitializeRedirect();
  void OnRemoteConnected();

  bool write_done_;
  bool read_done_;
  int fd_;
  bool connected_;
  bool guest_overflow_;
};

class Device;
class DeviceManager;
class RedirectUdpSocket : public UdpSocket {
 public:
  RedirectUdpSocket(NetworkBackendInterface* backend, ethhdr* eth, iphdr* ip, udphdr* udp) :
    UdpSocket(backend, eth, ip, udp) {
    InitializeRedirect();
  }
  virtual ~RedirectUdpSocket();
  void InitializeRedirect();
  void OnDataFromGuest(void* data, size_t length);
  void OnRemoteDataAvailable();
  virtual bool IsActive();

 protected:
  int fd_;
};

struct DhcpMessage;
class DhcpServiceUdpSocket : public UdpSocket {
 public:
  DhcpServiceUdpSocket(NetworkBackendInterface* backend, ethhdr* eth, iphdr* ip, udphdr* udp) :
    UdpSocket(backend, eth, ip, udp) {
  }
  void InitializeService(MacAddress router_mac, uint32_t router_ip, uint32_t subnet_mask, uint32_t guest_ip);
  void OnDataFromGuest(void* data, size_t length);
  std::string CreateDhcpResponse(DhcpMessage* request, int dhcp_type);
  size_t FillDhcpOptions(uint8_t* option, int dhcp_type);

 private:
  std::vector<uint32_t> nameservers_;
  MacAddress router_mac_;
  uint32_t subnet_mask_;
  uint32_t router_ip_;
  uint32_t guest_ip_;
};

#endif // _MVISOR_NETWORKS_USER_H
