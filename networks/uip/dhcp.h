#ifndef _MVISOR_UIP_DHCP_H
#define _MVISOR_UIP_DHCP_H

#include "udp.h"

struct DhcpMessage;
class DhcpServiceUdpSocket : public UdpSocket {
 public:
  DhcpServiceUdpSocket(Uip* backend, uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport);
  void OnPacketFromGuest(Ipv4Packet* packet);
  bool active();

 private:
  std::string CreateDhcpResponse(DhcpMessage* request, int dhcp_type);
  size_t FillDhcpOptions(uint8_t* option, int dhcp_type);

  std::vector<uint32_t> nameservers_;
};

#endif // _MVISOR_UIP_DHCP_H
