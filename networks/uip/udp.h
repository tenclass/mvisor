#ifndef _MVISOR_UIP_UDP_H
#define _MVISOR_UIP_UDP_H

#include "ipv4.h"

class UdpSocket : public Ipv4Socket {
 public:
  UdpSocket(Uip* backend, uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport);
  bool Equals(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport) {
    return sip_ == sip && dip_ == dip && sport_ == sport && dport_ == dport;
  }

 protected:
  virtual Ipv4Packet* AllocatePacket(bool urgent);
  uint16_t CalculateUdpChecksum(Ipv4Packet* packet);
  void OnDataFromHost(Ipv4Packet* packet);

  uint16_t sport_;
  uint16_t dport_;
};

#endif // _MVISOR_UIP_UDP_H
