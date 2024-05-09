#ifndef _MVISOR_TAP_H
#define _MVISOR_TAP_H

#include <string>

#include "object.h"
#include "device_interface.h"
#include "pci_device.h"

class Tap : public Object, public NetworkBackendInterface {
 private:
  PciDevice*    real_device_ = nullptr;
  int           tap_fd_ = -1;
  std::string   ifname_;
  size_t        mtu_ = 1500;
  bool          can_read_ = false;
  bool          can_write_ = false;

  bool          can_read() { return tap_fd_ != -1 && can_read_; }
  bool          can_write() { return tap_fd_ != -1 && can_write_; }

  void          StartReading();
  void          StartWriting();

 public:
  virtual ~Tap();
  virtual void Initialize(NetworkDeviceInterface* device, MacAddress& mac) override;
  virtual void SetMtu(int mtu) override;
  virtual void Reset() override;
  virtual void OnFrameFromGuest(std::deque<iovec>& vector) override;
  virtual void OnReceiveAvailable() override;
};

#endif // _MVISOR_TAP_H
