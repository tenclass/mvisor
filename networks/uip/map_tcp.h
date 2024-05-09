#ifndef _MVISOR_UIP_MAP_TCP_H
#define _MVISOR_UIP_MAP_TCP_H

#include "redirect_tcp.h"

class MapTcpSocket : public RedirectTcpSocket {
 public:
  MapTcpSocket(Uip* backend, uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport, int fd);
  virtual void InitializeRedirect(Ipv4Packet* packet);
};

#endif // _MVISOR_UIP_MAP_TCP_H
