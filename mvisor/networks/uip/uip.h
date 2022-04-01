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
#include "io_thread.h"

#define UIP_MAX_BUFFER_SIZE         (4096 - 16)
#define UIP_MAX_UDP_PAYLOAD(packet) (packet->mtu - 20 - 8)
#define UIP_MAX_TCP_PAYLOAD(packet) (packet->mtu - 144)

#define REDIRECT_TIMEOUT_SECONDS    (120)

struct PseudoHeader {
  uint32_t sip;
  uint32_t dip;
  uint8_t zero;
  uint8_t protocol;
  uint16_t length;
} __attribute__((packed));


class Ipv4Socket;
struct Ipv4Packet {
  Ipv4Socket*   socket;
  uint8_t       buffer[UIP_MAX_BUFFER_SIZE];
  int           mtu;
  ethhdr*       eth;
  iphdr*        ip;
  udphdr*       udp;
  tcphdr*       tcp;
  void*         data;
  size_t        data_length;
  size_t        data_offset;
  VoidCallback  Release;
};

class Ipv4Socket {
 public:
  Ipv4Socket(NetworkBackendInterface* backend, Ipv4Packet* packet);
  virtual ~Ipv4Socket() {}
  virtual void OnPacketFromGuest(Ipv4Packet* packet) = 0;
  
  virtual bool active() = 0;

 protected:
  virtual Ipv4Packet* AllocatePacket(bool urgent);
  uint16_t CalculateChecksum(uint8_t* addr, uint16_t count);

  NetworkBackendInterface* backend_;
  uint32_t sip_;
  uint32_t dip_;
  bool closed_;
  time_t active_time_;
  bool debug_;

  IoThread*   io_ = nullptr;
};


class TcpSocket : public Ipv4Socket {
 public:
  TcpSocket(NetworkBackendInterface* backend, Ipv4Packet* packet);
   
  inline bool Equals(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport) {
    return sip_ == sip && dip_ == dip && sport_ == sport && dport_ == dport;
  }
  virtual bool UpdateGuestAck(tcphdr* tcp);

 protected:
  virtual Ipv4Packet* AllocatePacket(bool urgent);
  uint16_t CalculateTcpChecksum(Ipv4Packet* packet);
  void OnDataFromHost(Ipv4Packet* packet, uint32_t tcp_flags);
  void ParseTcpOptions(tcphdr* tcp);
  void FillTcpOptions(tcphdr* tcp);
  void SynchronizeTcp(tcphdr* tcp);

  uint16_t sport_;
  uint16_t dport_;
  uint32_t window_size_;
  uint32_t guest_acked_;
  uint32_t ack_host_;
  uint32_t seq_host_;
  uint16_t mss_;
  uint8_t  window_scale_;
  bool     sack_permitted_;
};

class UdpSocket : public Ipv4Socket {
 public:
  UdpSocket(NetworkBackendInterface* backend, Ipv4Packet* packet);

  inline bool Equals(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport) {
    return sip_ == sip && dip_ == dip && sport_ == sport && dport_ == dport;
  }

 protected:
  virtual Ipv4Packet* AllocatePacket(bool urgent);
  uint16_t CalculateUdpChecksum(Ipv4Packet* packet);
  void OnDataFromHost(Ipv4Packet* packet);

  uint16_t sport_;
  uint16_t dport_;
};

class RedirectTcpSocket : public TcpSocket {
 public:
  RedirectTcpSocket(NetworkBackendInterface* backend, Ipv4Packet* packet);
  virtual ~RedirectTcpSocket();
  void Shutdown(int how);
  void InitializeRedirect(Ipv4Packet* packet);
  void OnPacketFromGuest(Ipv4Packet* packet);
  bool UpdateGuestAck(tcphdr* tcp);
  void Reset(Ipv4Packet* packet);

  bool active();
  bool connected() { return connected_; }

 protected:
  void StartReading();
  void StartWriting();
  void OnRemoteConnected();

  bool can_read() { return fd_ != -1 && connected_ && !read_done_ && can_read_; }
  bool can_write() { return fd_ != -1 && connected_ && !write_done_ && can_write_; }

  bool write_done_ = false;
  bool read_done_ = false;
  int  fd_ = -1;
  bool can_read_ = false;
  bool can_write_ = false;
  std::deque<Ipv4Packet*> send_queue_;
  bool connected_ = false;
};

class Device;
class DeviceManager;
class RedirectUdpSocket : public UdpSocket {
 public:
  RedirectUdpSocket(NetworkBackendInterface* backend, Ipv4Packet* packet) :
    UdpSocket(backend, packet) {
  }
  virtual ~RedirectUdpSocket();
  void InitializeRedirect();
  void OnPacketFromGuest(Ipv4Packet* packet);
  bool active();

 protected:
  void StartReading();

  int fd_;
  IoTimer*  wait_timer_ = nullptr;
};

struct DhcpMessage;
class DhcpServiceUdpSocket : public UdpSocket {
 public:
  DhcpServiceUdpSocket(NetworkBackendInterface* backend, Ipv4Packet* packet) :
    UdpSocket(backend, packet) {
  }
  void InitializeService(MacAddress router_mac, uint32_t router_ip, uint32_t subnet_mask, uint32_t guest_ip);
  void OnPacketFromGuest(Ipv4Packet* packet);
  bool active();

 private:
  std::string CreateDhcpResponse(DhcpMessage* request, int dhcp_type);
  size_t FillDhcpOptions(uint8_t* option, int dhcp_type);

  std::vector<uint32_t> nameservers_;
  MacAddress router_mac_;
  uint32_t subnet_mask_;
  uint32_t router_ip_;
  uint32_t guest_ip_;
};

#endif // _MVISOR_NETWORKS_USER_H
