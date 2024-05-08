/* 
 * MVisor Tap Network
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

#include "tap.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <linux/if_tun.h>

#include "utilities.h"
#include "logger.h"


void Tap::Initialize(NetworkDeviceInterface* device, MacAddress& mac) {
  MV_UNUSED(mac);
  device_ = device;
  real_device_ = dynamic_cast<PciDevice*>(device_);

  if (real_device_->has_key("ifname")) {
    ifname_ = std::get<std::string>((*real_device_)["ifname"]);
  } else {
    MV_PANIC("No ifname specified for Tap device");
  }

  /* Create the tap device */
  tap_fd_ = open("/dev/net/tun", O_RDWR);
  if (tap_fd_ < 0) {
    MV_PANIC("Failed to open /dev/net/tun");
  }

  /* Set the tap device name */
  struct ifreq ifr;
  bzero(&ifr, sizeof(ifr));
  ifr.ifr_flags = IFF_TAP | IFF_NO_PI | IFF_VNET_HDR;
  strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ);
  if (ioctl(tap_fd_, TUNSETIFF, (void*)&ifr) < 0) {
    MV_PANIC("Failed to create tap device");
  }

  /* Set virtio-net header size */
  int vnet_hdr_sz = 12;
  if (ioctl(tap_fd_, TUNSETVNETHDRSZ, &vnet_hdr_sz) < 0) {
    MV_PANIC("Failed to set vnet header size for tap device");
  }

  // Set checksum offloading */
  int offload = TUN_F_CSUM | TUN_F_TSO4 | TUN_F_UFO;
  if (ioctl(tap_fd_, TUNSETOFFLOAD, offload) < 0) {
    MV_PANIC("Failed to set offload for tap device");
  }

  // Set non-blocking
  MV_ASSERT(fcntl(tap_fd_, F_SETFL, fcntl(tap_fd_, F_GETFL, 0) | O_NONBLOCK) != -1);

  real_device_->StartPolling(tap_fd_, EPOLLIN | EPOLLOUT | EPOLLET, [this](auto events) {
    can_read_ = events & EPOLLIN;
    can_write_ = events & EPOLLOUT;
  
    if (can_read()) {
      StartReading();
    }
    if (can_write()) {
      StartWriting();
    }
  });
}

Tap::~Tap() {
  safe_close(&tap_fd_);
}

void Tap::SetMtu(int mtu) {
  mtu_ = mtu;
}

void Tap::Reset() {
}

void Tap::StartReading() {
  size_t buffer_size = (device_->offload_segmentation() ? 65536 : mtu_) + 30;
  uint8_t buffer[buffer_size];

  while (can_read()) {
    ssize_t len = read(tap_fd_, buffer, buffer_size);
    if (len < 0) {
      if (errno == EAGAIN) {
        can_read_ = false;
        return;
      }
      MV_PANIC("Failed to read from tap device");
    }

    if (len == 0) {
      return;
    }

    if (!device_->WriteBuffer(buffer, len)) {
      return;
    }
  }
}

void Tap::StartWriting() {
}

void Tap::OnFrameFromGuest(std::deque<iovec>& vector) {
  if(can_write()) {
    size_t buffer_size = 0;
    for (auto &v : vector) {
      buffer_size += v.iov_len;
    }

    uint8_t buffer[buffer_size];
    size_t copied = 0;
    for (auto &v : vector) {
      memcpy(buffer + copied, v.iov_base, v.iov_len);
      copied += v.iov_len;
    }

    // MV_HEXDUMP("out", buffer, copied);

    ssize_t len = write(tap_fd_, buffer, copied);
    if (len < 0) {
      if (errno == EAGAIN) {
        can_write_ = false;
        return;
      }
      MV_PANIC("Failed to write to tap device");
    }
  }
}

void Tap::OnReceiveAvailable() {
  if (can_read()) {
    StartReading();
  }
}

DECLARE_NETWORK(Tap);
