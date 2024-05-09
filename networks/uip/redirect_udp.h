#ifndef _MVISOR_UIP_REDIRECT_UDP_H
#define _MVISOR_UIP_REDIRECT_UDP_H

#include "udp.h"

class RedirectUdpSocket : public UdpSocket {
 public:
  RedirectUdpSocket(Uip* backend, uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport) :
    UdpSocket(backend, sip, dip, sport, dport) {
  }
  virtual ~RedirectUdpSocket();
  virtual void InitializeRedirect();
  virtual void OnGuestBufferAvaialble();
  virtual void OnPacketFromGuest(Ipv4Packet* packet);
  bool active();

 protected:
  void StartReading();

  inline bool can_read() { return fd_ != -1 && can_read_; }

  int       fd_;
  bool      can_read_ = false;
};

#endif // _MVISOR_UIP_REDIRECT_UDP_H
