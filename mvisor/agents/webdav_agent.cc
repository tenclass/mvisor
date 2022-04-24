/* 
 * MVisor WebDAV Agent
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

#include "object.h"
#include "device_interface.h"
#include "utilities.h"
#include "logger.h"

class WebdavAgent : public Object, public SerialPortInterface {
 private:
  std::string buffer_;

 public:
  WebdavAgent() {
    set_parent_name("virtio-console");
    strcpy(port_name_, "org.spice-space.webdav.0");
  }

  virtual void OnMessage(uint8_t* data, size_t size) {
    if (debug_) {
      MV_LOG("message");
      DumpHex(data, size);
    }
  }
};

DECLARE_AGENT(WebdavAgent)
