/* 
 * MVisor USB Device
 * Copyright (C) 2022 Terrence <terrence@tenclass.com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef _MVISOR_DEVICES_USB_USB_DEVICE_H
#define _MVISOR_DEVICES_USB_USB_DEVICE_H

#include "device.h"
#include "usb_descriptor.h"
#include <sys/uio.h>

#define USB_MAX_ENDPOINTS  15
#define USB_MAX_INTERFACES 16

enum UsbSpeed {
  kUsbSpeedLow = 0,
  kUsbSpeedFull,
  kUsbSpeedHigh,
  kUsbSpeedSuper
};

struct UsbEndpoint {

};

struct UsbPacket {
  uint      direction;
  uint      stream_id;
  uint64_t  id;
  int       status;
  size_t    content_length;
  size_t    size;
  std::vector<struct iovec>  iov;
  /* control transfer */
  uint64_t  control_parameter;
  /* destructor */
  VoidCallback Release;
  VoidCallback OnComplete;
};

class UsbDevice : public Device {
 public:
  int speed() { return speed_; }
  UsbPacket* CreatePacket(uint direction, uint stream_id, uint64_t id, VoidCallback on_complete);
  void HandlePacket(UsbPacket* packet);
  void CancelPacket(UsbPacket* packet);

 protected:
  int speed_ = kUsbSpeedHigh;
  UsbEndpoint control_endpoint_;
  UsbEndpoint in_endpoints_[USB_MAX_ENDPOINTS];
  UsbEndpoint out_endpoints_[USB_MAX_ENDPOINTS];
  const UsbDeviceDescriptor* device_descriptor_ = nullptr;
  const UsbStringsDescriptor* strings_descriptor_ = nullptr;
  const UsbConfigurationDescriptor* config_ = nullptr;
  uint8_t configuration_ = 0;
  bool remote_wakeup_ = false;
  int alternate_settings_[16] = { 0 };

  void SetupDescriptor(const UsbDeviceDescriptor*, const UsbStringsDescriptor*);

  /* Low level interfaces */
  virtual void OnControlPacket(UsbPacket* packet);
  virtual void OnDataPacket(UsbPacket* packet);

  virtual int OnControl(uint request, uint value, uint index, uint8_t* data, int length);
  virtual int OnInputData(uint8_t* data, int length);
  virtual int OnOutputData(uint8_t* data, int length);

 private:
  void CopyPacketData(UsbPacket* packet, uint8_t* data, int length);
  int CopyConfigurationDescriptor(uint index, uint8_t* data, int length);
  int CopyStringsDescriptor(uint index, uint8_t* data, int length);
  int GetDescriptor(uint value, uint8_t* data, int length);
  int GetStatus(uint8_t* data, int length);
  int SetConfiguration(uint value);
  int SetInterface(uint index, uint value);
};

#endif // _MVISOR_DEVICES_USB_USB_DEVICE_H
