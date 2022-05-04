/* 
 * MVisor Ipv4 Socket
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

#include "uip.h"

#include <arpa/inet.h>

#include "device_manager.h"
#include "logger.h"


Ipv4Socket::Ipv4Socket(NetworkBackendInterface* backend, Ipv4Packet* packet) :
  backend_(backend) {
  auto ip = packet->ip;
  sip_ = ntohl(ip->saddr);
  dip_ = ntohl(ip->daddr);
  debug_ = false;
  active_time_ = time(nullptr);

  auto device = dynamic_cast<Device*>(backend_->device());
  io_ = device->manager()->io();
  debug_ = device->debug();
  MV_ASSERT(io_);
}

Ipv4Packet* Ipv4Socket::AllocatePacket(bool urgent) {
  return backend_->AllocatePacket(urgent);
}

uint16_t Ipv4Socket::CalculateChecksum(uint8_t* addr, uint16_t count) {
  long sum = 0;

  while (count > 1) {
    sum += *(uint16_t *)addr;
    addr += 2;
    count -= 2;
  }

  if (count > 0)
    sum += *(uint8_t*)addr;

  while (sum >> 16)
    sum = (sum & 0xffff) + (sum >> 16);

  return ~sum;
}
