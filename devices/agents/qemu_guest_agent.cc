/* 
 * MVisor Qemu Guest Agent
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

/*
 * Reference: https://qemu.readthedocs.io/en/latest/interop/qemu-ga-ref.html
 */

#include <string>

#include "device.h"
#include "device_manager.h"
#include "device_interface.h"
#include "utilities.h"
#include "logger.h"

class QemuGuestAgent : public Device, public SerialPortInterface {
 public:
  QemuGuestAgent() {
    set_default_parent_class("VirtioConsole");

    strcpy(port_name_, "org.qemu.guest_agent.0");
  }

  // virtual void set_ready(bool ready) {
  //   SerialPortInterface::set_ready(ready);
  //   if (ready) {
  //     std::string cmd = "{\"execute\":\"guest-info\"}\n";
  //     device_->SendMessage(this, (uint8_t*)cmd.data(), cmd.size());
  //   }
  // }

  /* This interface function is called by UI thread */
  virtual void SendMessage(uint8_t* data, size_t size) {
    Schedule([this, copied = std::string((char*)data, size)]() {
      device_->SendMessage(this, (uint8_t*)copied.data(), copied.size());
    });
  }
};

DECLARE_DEVICE(QemuGuestAgent)
