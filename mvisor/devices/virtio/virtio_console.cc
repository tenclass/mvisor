/* 
 * MVisor VirtIO Serial PCI class
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

#include "virtio_console.h"
#include <cstring>
#include <array>
#include <functional>
#include "logger.h"
#include "device_manager.h"
#include "virtio_pci.h"

class VirtioConsole : public VirtioPci, public VirtioConsoleInterface {
 private:
  virtio_console_config           console_config_;
  std::vector<VirtioConsolePort*> console_ports_;

 public:
  VirtioConsole() {
    /* Device specific features */
    device_features_ |= (1UL << VIRTIO_CONSOLE_F_EMERG_WRITE) | (1UL << VIRTIO_CONSOLE_F_MULTIPORT);

    bzero(&console_config_, sizeof(console_config_));
    console_config_.max_nr_ports = common_config_.num_queues / 2 - 1;
  
    CreateQueuesForPorts();
  }

  void Connect() {
    VirtioPci::Connect();

    for (auto device : children_) {
      VirtioConsolePort* port = dynamic_cast<VirtioConsolePort*>(device);
      if (port) {
        port->Initialize(this, console_ports_.size() + 1);
        console_ports_.push_back(port);
      }
    }
  }

  void CreateQueuesForPorts() {
    AddQueue(128, std::bind(&VirtioConsole::OnPortInput, this, 0));
    AddQueue(128, std::bind(&VirtioConsole::OnPortOutput, this, 0));
    
    AddQueue(32, std::bind(&VirtioConsole::OnControlInput, this));
    AddQueue(32, std::bind(&VirtioConsole::OnControlOutput, this));

    for (uint i = 1; i < console_config_.max_nr_ports; i++) {
      AddQueue(128, std::bind(&VirtioConsole::OnPortInput, this, i));
      AddQueue(128, std::bind(&VirtioConsole::OnPortOutput, this, i));
    }
  }

  void ReadDeviceConfig(uint64_t offset, uint8_t* data, uint32_t size) {
    MV_ASSERT(offset + size <= sizeof(console_config_));
    memcpy(data, (uint8_t*)&console_config_ + offset, size);
  }

  void OnPortInput(uint32_t id) {
    auto port = FindPortById(id);
    port->OnGuestWritable();
  }

  void OnPortOutput(uint32_t id) {
    auto port = FindPortById(id);
    auto &vq = queues_[3 + id * 2];
    VirtElement element;
  
    while (PopQueue(vq, element)) {
      for (auto iov : element.vector) {
        port->OnMessage((uint8_t*)iov.iov_base, iov.iov_len);
      }
      PushQueue(vq, element);
    }
    MV_ASSERT(vq.available_ring->flags == 0);
    NotifyQueue(vq);
  }

  void SendPortMessage(VirtioConsolePort* port, uint8_t* data, size_t size) {
    auto &vq = queues_[2 + port->port_id() * 2];
    WriteBuffer(vq, data, size);
  }

  void OnControlInput() {
    /* Guest confirm control input */
  }

  void OnControlOutput() {
    auto &vq = queues_[3];
    VirtElement element;
  
    while (PopQueue(vq, element)) {
      for (auto iov : element.vector) {
        virtio_console_control* vcc = (virtio_console_control*)iov.iov_base;
        HandleConsoleControl(vcc);
      }
      PushQueue(vq, element);
    }
    MV_ASSERT(vq.available_ring->flags == 0);
    NotifyQueue(vq);
  }

  void WriteBuffer(VirtQueue& vq, void* buffer, size_t size) {
    size_t offset = 0;
    // MV_LOG("Send size=%lu", size);
    // DumpHex(buffer, size);

    while (offset < size) {
      VirtElement element;
      if (!PopQueue(vq, element)) {
        break;
      }

      element.length = 0;
      size_t remain_bytes = size - offset;
      for (auto iov : element.vector) {
        size_t bytes = iov.iov_len < remain_bytes ? iov.iov_len : remain_bytes;
        memcpy(iov.iov_base, (uint8_t*)buffer + offset, bytes);
        offset += bytes;
        remain_bytes -= bytes;
        element.length += bytes;
      }

      PushQueue(vq, element);
    }
    MV_ASSERT(vq.used_ring->flags == 0);
    NotifyQueue(vq);
  }

  void SendControlEvent(VirtioConsolePort* port, uint16_t event, uint16_t value) {
    virtio_console_control vcc = {
      .id = port->port_id(),
      .event = event,
      .value = value
    };
    WriteBuffer(queues_[2], &vcc, sizeof(vcc));
  }

  VirtioConsolePort* FindPortById(uint32_t id) {
    for (auto &port: console_ports_) {
      if (port->port_id() == id) {
        return port;
      }
    }
    return nullptr;
  }

  void SendPortName(VirtioConsolePort* port) {
    virtio_console_control vcc = {
      .id = port->port_id(),
      .event = VIRTIO_CONSOLE_PORT_NAME,
      .value = 1
    };
    char buffer[200] = { 0 };
    memcpy(buffer, &vcc, sizeof(vcc));
    memcpy(buffer + sizeof(vcc), port->port_name(), strlen(port->port_name()));
    WriteBuffer(queues_[2], buffer, sizeof(vcc) + strlen(port->port_name()) + 1);
  }

  void HandleConsoleControl(virtio_console_control* vcc) {
    if (vcc->event == VIRTIO_CONSOLE_DEVICE_READY) {
      MV_ASSERT(vcc->value == 1); /* 1 means success */
      
      for (auto &port : console_ports_) {
        SendControlEvent(port, VIRTIO_CONSOLE_PORT_ADD, 1);
      }
      return;
    }

    MV_ASSERT(vcc->id >= 1 && vcc->id <= console_ports_.size());
    auto port = FindPortById(vcc->id);
    switch (vcc->event)
    {
    case VIRTIO_CONSOLE_PORT_READY:
      MV_ASSERT(vcc->value == 1); /* 1 means success */
      SendPortName(port);
      SendControlEvent(port, VIRTIO_CONSOLE_PORT_OPEN, 1);
      break;
    
    case VIRTIO_CONSOLE_PORT_OPEN:
      port->SetGuestConnected(vcc->value);
      break;
    default:
      MV_PANIC("port=%d event=%d", vcc->id, vcc->event);
      break;
    }
  }
};

DECLARE_DEVICE(VirtioConsole)
