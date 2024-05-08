#ifndef _MVISOR_UIP_REDIRECT_ICMP_H
#define _MVISOR_UIP_REDIRECT_ICMP_H

#include "icmp.h"

class RedirectIcmpSocket : public IcmpSocket {
 public:
  RedirectIcmpSocket(Uip* backend, uint32_t sip, uint32_t dip, uint16_t echo_id) :
    IcmpSocket(backend, sip, dip, echo_id) {
  }
  virtual ~RedirectIcmpSocket();
  virtual void InitializeRedirect();
  virtual void OnPacketFromGuest(Ipv4Packet* packet);
  bool active();

 protected:
  void StartReading();

  inline bool can_read() { return fd_ != -1 && can_read_; }

  int       fd_;
  bool      can_read_ = false;
  bool      raw_mode_ = false;
};

#endif // _MVISOR_UIP_REDIRECT_ICMP_H
