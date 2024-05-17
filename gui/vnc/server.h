#ifndef _MVISOR_VNC_SERVER_H
#define _MVISOR_VNC_SERVER_H

#include <mutex>
#include <list>

#include "device_interface.h"
#include "machine.h"

enum VncSecurityTypes {
  kVncSecurityInvalid = 0,
  kVncSecruityNone = 1,
  kVncSecurityVncAuth = 2,
};

class VncConnection;
class VncServer {
 private:
  Machine*    machine_;
  int         server_fd_ = -1;
  int         event_fd_ = -1;
  uint16_t    port_ = 0;
  std::mutex  mutex_;
  std::list<VoidCallback>   tasks_;
  std::list<VncConnection*> connections_;
  int         security_type_ = kVncSecruityNone;
  std::string password_;

  VncConnection* GetConnectionByFd(int fd);
  void RemoveConnection(VncConnection* conn);
  void OnEvent();
  void OnAccept();

 public:
  VncServer(Machine* machine, uint16_t port);
  ~VncServer();
  void MainLoop();
  void Close();
  void SetPassword(const std::string& password);
  void Schedule(VoidCallback callback);

  inline Machine* machine() { return machine_; }
  inline std::mutex& mutex() { return mutex_; }
  inline int security_type() { return security_type_; }
  inline const std::string& password() { return password_; }
};

#endif // _MVISOR_VNC_SERVER_H
