/* 
 * MVisor
 * Copyright (C) 2021 Terrence <terrence@tenclass.com>
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

#ifndef _MVISOR_DEVICES_VIRTIO_CONSOLE_H
#define _MVISOR_DEVICES_VIRTIO_CONSOLE_H

#include "device.h"
#include <linux/virtio_console.h>

class VirtioConsolePort;
class VirtioConsoleInterface {
 public:
  virtual void SendPortMessage(VirtioConsolePort* port, uint8_t* data, size_t size) = 0;
};

class VirtioConsolePort : public Device {
 public:
  virtual void OnMessage(uint8_t* data, size_t size) = 0;
  virtual void OnGuestWritable() = 0;

  virtual void SetGuestConnected(bool connected) {
    guest_connected_ = connected;
  }

  void Initialize(VirtioConsoleInterface* console, uint32_t id) {
    console_ = console;
    port_id_ = id;
  }
  uint32_t port_id() { return port_id_; }
  const char* port_name() { return port_name_; }

 protected:
  VirtioConsoleInterface* console_;
  uint32_t port_id_;
  char port_name_[100];
  bool    guest_connected_ = false;
  bool    guest_writable_ = false;
};


#endif // _MVISOR_DEVICES_VIRTIO_CONSOLE_H
