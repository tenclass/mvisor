#ifndef _MVISOR_NETWORKS_UIP_REDIRECT_TCP_H
#define _MVISOR_NETWORKS_UIP_REDIRECT_TCP_H

#include "tcp.h"

class RedirectTcpSocket : public TcpSocket {
 public:
  RedirectTcpSocket(Uip* backend, uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport);
  virtual ~RedirectTcpSocket();
  virtual void InitializeRedirect(Ipv4Packet* packet);
  virtual void OnGuestBufferAvaialble();
  virtual void OnPacketFromGuest(Ipv4Packet* packet);
  void Shutdown(int how);
  bool UpdateGuestAck(tcphdr* tcp);
  void ReplyReset(Ipv4Packet* packet);
  void Reset();

  bool active();
  bool connected() { return connected_; }

 protected:
  void StartReading();
  void StartWriting();
  void OnRemoteConnected();

  inline bool can_read() { return fd_ != -1 && connected_ && !read_done_ && can_read_; }
  inline bool can_write() { return fd_ != -1 && connected_ && !write_done_ && can_write_; }

  bool write_done_ = false;
  bool read_done_ = false;
  int  fd_ = -1;
  bool can_read_ = false;
  bool can_write_ = false;
  std::deque<Ipv4Packet*> send_queue_;
  bool connected_ = false;
};

#endif // _MVISOR_NETWORKS_UIP_REDIRECT_TCP_H
