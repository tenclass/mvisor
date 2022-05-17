/* 
 * MVisor VirtIO Console Device
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

#include <cstring>
#include <array>
#include <functional>
#include <linux/virtio_console.h>

#include "device_interface.h"
#include "logger.h"
#include "device_manager.h"
#include "virtio_pci.h"
#include "pb/virtio_console.pb.h"

class VirtioConsole : public VirtioPci, public SerialDeviceInterface {
 private:
  virtio_console_config             console_config_;
  std::vector<SerialPortInterface*> console_ports_;

 public:
  VirtioConsole() {
    pci_header_.class_code = 0x078000;
    pci_header_.device_id = 0x1003;
    pci_header_.subsys_id = 0x0003;
    
    AddPciBar(1, 0x1000, kIoResourceTypeMmio);
    AddMsiXCapability(1, 2, 0, 0x1000);

    /* Device specific features */
    device_features_ |= (1UL << VIRTIO_CONSOLE_F_MULTIPORT);

    bzero(&console_config_, sizeof(console_config_));
    console_config_.max_nr_ports = 1;
  }

  void Connect() {
    VirtioPci::Connect();

    for (auto object : children_) {
      SerialPortInterface* port = dynamic_cast<SerialPortInterface*>(object);
      if (port) {
        port->Initialize(this, console_ports_.size() + 1);
        console_ports_.push_back(port);
        ++console_config_.max_nr_ports;
      } else {
        MV_PANIC("%s is not a port object", object->name());
      }
    }
  }

  void Reset() {
    /* stop console port activities */
    for (auto &port: console_ports_) {
      port->set_ready(false);
    }
  
    /* Reset all queues */
    VirtioPci::Reset();
  
    CreateQueuesForPorts();
  }

  bool SaveState(MigrationWriter* writer) {
    VirtioConsoleState state;
    for (auto console_port : console_ports_) {
      auto port = state.add_ports();
      port->set_id(console_port->port_id());
      port->set_ready(console_port->ready());
    }
    writer->WriteProtobuf("VIRTIO_CONSOLE", state);
    return VirtioPci::SaveState(writer);
  }

  bool LoadState(MigrationReader* reader) {
    if (!VirtioPci::LoadState(reader)) {
      return false;
    }
    VirtioConsoleState state;
    if (!reader->ReadProtobuf("VIRTIO_CONSOLE", state)) {
      return false;
    }
    for (int i = 0; i < state.ports_size(); i++) {
      auto& port = state.ports(i);
      for (auto console_port : console_ports_) {
        if (console_port->port_id() == port.id()) {
          console_port->set_ready(port.ready());
        }
      }
    }
    return true;
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
    port->OnWritable();
  }

  void OnPortOutput(uint32_t id) {
    auto port = FindPortById(id);
    auto &vq = queues_[3 + id * 2];
  
    while (auto element = PopQueue(vq)) {
      for (auto &iov : element->vector) {
        port->OnMessage((uint8_t*)iov.iov_base, iov.iov_len);
      }
      PushQueue(vq, element);
      NotifyQueue(vq);
    }
  }

  void SendMessage(SerialPortInterface* port, uint8_t* data, size_t size) {
    auto &vq = queues_[2 + port->port_id() * 2];
    WriteBuffer(vq, data, size);
  }

  void OnControlInput() {
    /* Guest confirm control input */
  }

  void OnControlOutput() {
    auto &vq = queues_[3];
  
    while (auto element = PopQueue(vq)) {
      for (auto &iov : element->vector) {
        virtio_console_control* vcc = (virtio_console_control*)iov.iov_base;
        HandleConsoleControl(vcc);
      }
      PushQueue(vq, element);
    }
    NotifyQueue(vq);
  }

  void WriteBuffer(VirtQueue& vq, void* buffer, size_t size) {
    size_t offset = 0;
    while (offset < size) {
      auto element = PopQueue(vq);
      if (!element) {
        break;
      }
      size_t remain_bytes = size - offset;
      for (auto &iov : element->vector) {
        size_t bytes = iov.iov_len < remain_bytes ? iov.iov_len : remain_bytes;
        memcpy(iov.iov_base, (uint8_t*)buffer + offset, bytes);
        offset += bytes;
        remain_bytes -= bytes;
        element->length += bytes;
      }

      PushQueue(vq, element);
      NotifyQueue(vq);
    }
  }

  void SendControlEvent(SerialPortInterface* port, uint16_t event, uint16_t value) {
    virtio_console_control vcc = {
      .id = port->port_id(),
      .event = event,
      .value = value
    };
    WriteBuffer(queues_[2], &vcc, sizeof(vcc));
  }

  SerialPortInterface* FindPortById(uint32_t id) {
    for (auto &port: console_ports_) {
      if (port->port_id() == id) {
        return port;
      }
    }
    return nullptr;
  }

  void SendPortName(SerialPortInterface* port) {
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
      if (vcc->value != 1) { /* 1 means success */
        MV_LOG("failed to initialize virtio console ret=0x%x", vcc->value);
        return;
      }
      
      for (auto &port : console_ports_) {
        SendControlEvent(port, VIRTIO_CONSOLE_PORT_ADD, 1);
      }
      return;
    }

    if (vcc->id < 1 || vcc->id > console_ports_.size()) {
      MV_LOG("invalid vcc id=%d", vcc->id);
      return;
    }
    auto port = FindPortById(vcc->id);
    switch (vcc->event)
    {
    case VIRTIO_CONSOLE_PORT_READY:
      MV_ASSERT(vcc->value == 1); /* 1 means success */
      SendPortName(port);
      SendControlEvent(port, VIRTIO_CONSOLE_PORT_OPEN, 1);
      break;
    
    case VIRTIO_CONSOLE_PORT_OPEN:
      port->set_ready(vcc->value);
      break;
    default:
      MV_PANIC("port=%d event=%d", vcc->id, vcc->event);
      break;
    }
  }
};

DECLARE_DEVICE(VirtioConsole)
