#ifndef _MVISOR_UIP_TCP_H
#define _MVISOR_UIP_TCP_H

#include "ipv4.h"

class TcpSocket : public Ipv4Socket {
 public:
  TcpSocket(Uip* backend, uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport);
  bool Equals(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport) {
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
  uint32_t window_size_ = 0;
  uint32_t guest_acked_ = 0;
  uint32_t ack_host_ = 0;
  uint32_t seq_host_ = 0;
  uint16_t mss_ = 0;
  uint8_t  window_scale_ = 0;
  bool     sack_permitted_ = false;
};

#endif // _MVISOR_UIP_TCP_H
