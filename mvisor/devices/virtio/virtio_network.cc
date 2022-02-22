/* 
 * MVisor VirtIO Network Device
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

#include "virtio_pci.h"
#include <cstring>
#include <cstdlib>
#include <set>
#include "linux/virtio_net.h"
#include "device_interface.h"
#include "logger.h"

#define DEFAULT_QUEUE_SIZE 256

struct RxMode {
  bool  promisc;
  bool  all_multicast;
  bool  all_unicast;
  bool  no_multicast;
  bool  no_unicast;
  bool  no_broadcast;
};

class VirtioNetwork : public VirtioPci, public NetworkDeviceInterface {
 private:
  virtio_net_config net_config_;
  RxMode            rx_mode_;
  std::set<MacAddress> mac_table_;
  NetworkBackendInterface* backend_ = nullptr;

 public:
  VirtioNetwork() {
    devfn_ = PCI_MAKE_DEVFN(1, 0);
    pci_header_.class_code = 0x020000;
    pci_header_.device_id = 0x1000;
    pci_header_.subsys_id = 0x0001;
    
    // FIXME: IRQ interrupts sometimes not work on Windows 10
    AddPciBar(1, 0x1000, kIoResourceTypeMmio);
    AddMsiXCapability(1, 4, 0, 0x1000);
    
    device_features_ |=
      (1UL << VIRTIO_NET_F_MAC) |
      (1UL << VIRTIO_NET_F_MRG_RXBUF) |
      (1UL << VIRTIO_NET_F_STATUS) |
      (1UL << VIRTIO_NET_F_CTRL_VQ) |
      (1UL << VIRTIO_NET_F_CTRL_RX) |
      (1UL << VIRTIO_NET_F_CTRL_VLAN) |
      (1UL << VIRTIO_NET_F_CTRL_RX_EXTRA) |
      (1UL << VIRTIO_NET_F_GUEST_ANNOUNCE);

    bzero(&net_config_, sizeof(net_config_));
    GenerateRandomMac(net_config_.mac);
    net_config_.status = VIRTIO_NET_S_LINK_UP;
  }

  void GenerateRandomMac(uint8_t mac[6]) {
    mac[0] = 0x00;
    mac[1] = 0x50;
    mac[2] = 0x00;
    for (int i = 3; i < 6; i++) {
      mac[i] = rand() & 0xFF;
    }
  }

  virtual void Disconnect() {
    VirtioPci::Disconnect();
    if (backend_) {
      delete dynamic_cast<Object*>(backend_);
      backend_ = nullptr;
    }
  }

  virtual void Connect() {
    VirtioPci::Connect();

    /* Configurable MAC address */
    if (has_key("mac")) {
      uint32_t mac[6];
      std::string mac_string = std::get<std::string>(key_values_["mac"]);
      sscanf(mac_string.c_str(), "%02x:%02x:%02x:%02x:%02x:%02x", &mac[0], &mac[1], &mac[2],
        &mac[3], &mac[4], &mac[5]);
      for (int i = 0; i < 6; i++)
        net_config_.mac[i] = mac[i];
    }
    /* Check user network or tap network */
    if (has_key("backend")) {
      std::string network_type = std::get<std::string>(key_values_["backend"]);
      backend_ = dynamic_cast<NetworkBackendInterface*>(Object::Create(network_type.c_str()));
      MV_ASSERT(backend_);
      MacAddress mac;
      memcpy(mac.data, net_config_.mac, sizeof(mac.data));
      backend_->Initialize(this, mac);
    } else {
      MV_PANIC("network backend is not set");
    }
  }

  void Reset() {
    /* Reset all queues */
    VirtioPci::Reset();
  
    /* MQ is not supported yet */
    AddQueue(DEFAULT_QUEUE_SIZE, std::bind(&VirtioNetwork::OnReceive, this, 0));
    AddQueue(DEFAULT_QUEUE_SIZE, std::bind(&VirtioNetwork::OnTransmit, this, 1));
    AddQueue(DEFAULT_QUEUE_SIZE, std::bind(&VirtioNetwork::OnControl, this, 2));

    backend_->Reset();
  }

  void ReadDeviceConfig(uint64_t offset, uint8_t* data, uint32_t size) {
    MV_ASSERT(offset + size <= sizeof(net_config_));
    memcpy(data, (uint8_t*)&net_config_ + offset, size);
  }

  void WriteDeviceConfig(uint64_t offset, uint8_t* data, uint32_t size) {
    MV_ASSERT(offset + size <= sizeof(net_config_));
    memcpy((uint8_t*)&net_config_ + offset, data, size);
  }

  void OnReceive(int queue_index) {
    if (debug_) {
      MV_LOG("OnReceive %d", queue_index);
    }
    if (backend_) {
      backend_->OnReceiveAvailable();
    }
  }

  void OnTransmit(int queue_index) {
    auto &vq = queues_[queue_index];
  
    while (auto element = PopQueue(vq)) {
      HandleTransmit(vq, element);
      PushQueue(vq, element);
      NotifyQueue(vq);
    }
  }

  void OnControl(int queue_index) {
    auto &vq = queues_[queue_index];
  
    while (auto element = PopQueue(vq)) {
      HandleControl(vq, element);
      PushQueue(vq, element);
      NotifyQueue(vq);
    }
  }

  virtual bool WriteBuffer(void* buffer, size_t size) {
    VirtQueue& vq = queues_[0];
    size_t offset = 0;
    while (offset < size) {
      auto element = PopQueue(vq);
      if (!element) {
        if (debug_) {
          MV_LOG("network queue is full, queue size=%d", vq.size);
        }
        return false;
      }

      if (offset == 0) {
        /* Prepend virtio net header to the buffer vector, the first buffer length = 0xC */
        virtio_net_hdr_v1 header = { .gso_type = VIRTIO_NET_HDR_GSO_NONE };
        auto &iov = element->vector[0];
        MV_ASSERT(iov.iov_len == sizeof(header));
        memcpy(iov.iov_base, &header, sizeof(header));
        element->length += sizeof(header);
        element->vector.pop_front();
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
      MV_ASSERT(offset == size);
    }
    NotifyQueue(vq);
    return true;
  }

  void HandleTransmit(VirtQueue& vq, VirtElement* element) {
    auto &vector = element->vector;
    MV_ASSERT(vector.size() >= 1);
    auto &front = vector.front();
    virtio_net_hdr_v1* header = (virtio_net_hdr_v1*)front.iov_base;
    MV_ASSERT(header->gso_type == VIRTIO_NET_HDR_GSO_NONE);

    if (vector.size() == 1) {
      backend_->OnFrameFromGuest(&header[1], element->size - sizeof(*header));
    } else {
      /* merge buffers into one piece */
      auto frame = new uint8_t[element->size];
      size_t copied = 0;
      for (auto &v : vector) {
        memcpy(frame + copied, v.iov_base, v.iov_len);
        copied += v.iov_len;
      }
      MV_ASSERT(copied == element->size);
      backend_->OnFrameFromGuest(frame + sizeof(*header), element->size - sizeof(*header));
      delete frame;
    }
  }

  void HandleControl(VirtQueue& vq, VirtElement* element) {
    auto &vector = element->vector;
    MV_ASSERT(vector.size() >= 3);

    virtio_net_ctrl_hdr* control = (virtio_net_ctrl_hdr*)vector.front().iov_base;
    vector.pop_front();
    // MV_LOG("control cls=0x%x cmd=0x%x vector size=%d", control->cls, control->cmd, vector.size());

    uint8_t* status = (uint8_t*)vector.back().iov_base;
    MV_ASSERT(vector.back().iov_len == 1);
    vector.pop_back();

    element->length = sizeof(*status);
    auto &iov = vector.front();

    switch (control->cls)
    {
    case VIRTIO_NET_CTRL_RX:
      MV_ASSERT(iov.iov_len >= 1);
      *status = ControlRxMode(control->cmd, *(uint8_t*)iov.iov_base);
      break;
    case VIRTIO_NET_CTRL_MAC:
      *status = ControlMacTable(control->cmd, (virtio_net_ctrl_mac*)iov.iov_base);
      break;
    case VIRTIO_NET_CTRL_VLAN:
      if (iov.iov_len == 2) {
        *status = ControlVlanTable(control->cmd, *(uint16_t*)iov.iov_base);
      } else {
        *status = VIRTIO_NET_ERR;
      }
      break;
    default:
      MV_PANIC("unhandled control class=0x%x command=0x%x", control->cls, control->cmd);
      break;
    }
  }

  uint8_t ControlMacTable(uint8_t command, virtio_net_ctrl_mac* table) {
    if (command == VIRTIO_NET_CTRL_MAC_TABLE_SET) {
      for (uint32_t i = 0; i < table->entries; i++) {
        MacAddress mac;
        memcpy(mac.data, table->macs[i], sizeof(table->macs[i]));
        mac_table_.insert(mac);
      }
      return VIRTIO_NET_OK;
    } else {
      MV_PANIC("unhandled command=0x%x", command);
    }
    return VIRTIO_NET_ERR;
  }

  uint8_t ControlVlanTable(uint8_t command, uint16_t vlan_id) {
    /* FIXME: Not documented */
    return VIRTIO_NET_ERR;
  }

  uint8_t ControlRxMode(uint8_t command, uint8_t on) {
    switch (command)
    {
    case VIRTIO_NET_CTRL_RX_PROMISC:
      rx_mode_.promisc = on;
      break;
    case VIRTIO_NET_CTRL_RX_ALLMULTI:
      rx_mode_.all_multicast = on;
      break;
    case VIRTIO_NET_CTRL_RX_ALLUNI:
      rx_mode_.all_unicast = on;
      break;
    case VIRTIO_NET_CTRL_RX_NOMULTI:
      rx_mode_.no_multicast = on;
      break;
    case VIRTIO_NET_CTRL_RX_NOUNI:
      rx_mode_.no_unicast = on;
      break;
    case VIRTIO_NET_CTRL_RX_NOBCAST:
      rx_mode_.no_broadcast = on;
      break;
    default:
      return VIRTIO_NET_ERR;
    }
    return VIRTIO_NET_OK;
  }
};

DECLARE_DEVICE(VirtioNetwork);
