#ifndef _MVISOR_UIP_ICMP_H
#define _MVISOR_UIP_ICMP_H

#include "ipv4.h"

class IcmpSocket : public Ipv4Socket {
 public:
  IcmpSocket(Uip* backend, uint32_t sip, uint32_t dip, uint16_t echo_id);
  bool Equals(uint32_t sip, uint32_t dip, uint16_t echo_id) {
    return sip_ == sip && dip_ == dip && echo_id_ == echo_id;
  }

 protected:
  virtual Ipv4Packet* AllocatePacket(bool urgent);
  uint16_t CalculateIcmpChecksum(Ipv4Packet* packet);
  void OnDataFromHost(Ipv4Packet* packet);

  uint16_t  echo_id_;
};


#endif // _MVISOR_UIP_ICMP_H
