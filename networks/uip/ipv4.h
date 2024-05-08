#ifndef _MVISOR_UIP_IPV4_H
#define _MVISOR_UIP_IPV4_H

#include "uip.h"
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/tcp.h>
#include <linux/icmp.h>
#include <linuz/virtio_net.h>

#define IPV4_MAX_BUFFER_SIZE           (4096)
#define IPV4_MAX_UDP_PAYLOAD(packet)   (packet->mtu - 20)
#define IPV4_MAX_TCP_PAYLOAD(packet)   (packet->mtu - 20 - 20)

#define REDIRECT_TIMEOUT_SECONDS      (120)

/* Used to calculate checksum */
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
  uint8_t       buffer[IPV4_MAX_BUFFER_SIZE];
  int           mtu;
  virtio_net_hdr_v1* vnet;
  ethhdr*       eth;
  iphdr*        ip;
  udphdr*       udp;
  tcphdr*       tcp;
  icmphdr*      icmp;
  void*         data;
  size_t        data_length;
  size_t        data_offset;
  VoidCallback  Release;
};


class Ipv4Socket {
 public:
  Ipv4Socket(Uip* backend, uint32_t sip, uint32_t dip);

  virtual ~Ipv4Socket() {}
  virtual void OnPacketFromGuest(Ipv4Packet* packet) = 0;
  virtual void OnGuestBufferAvaialble();
  
  virtual bool active() = 0;

 protected:
  virtual Ipv4Packet* AllocatePacket(bool urgent);
  uint16_t CalculateChecksum(uint8_t* addr, uint16_t count);

  Uip*                      backend_;
  uint32_t                  sip_;
  uint32_t                  dip_;
  bool                      closed_;
  time_t                    active_time_;
  bool                      debug_;
  PciDevice*                device_ = nullptr;
};

#endif  // _MVISOR_UIP_IPV4_H
