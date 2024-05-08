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
#include "linuz/virtio_net.h"
#include "device_interface.h"
#include "logger.h"

#define DEFAULT_MTU 1500

class VirtioNetwork : public VirtioPci, public NetworkDeviceInterface {
 private:
  virtio_net_config         net_config_;
  std::set<MacAddress>      mac_table_;
  NetworkBackendInterface*  backend_ = nullptr;

 public:
  VirtioNetwork() {
    pci_header_.class_code = 0x020000;
    pci_header_.device_id = 0x1000;
    pci_header_.subsys_id = 0x0001;
    
    SetupPciBar(1, 0x1000, kIoResourceTypeMmio);
    AddMsiXCapability(1, 4, 0, 0x1000);
    
    device_features_ |=
      (1UL << VIRTIO_NET_F_MTU) |
      (1UL << VIRTIO_NET_F_MAC) |
      (1UL << VIRTIO_NET_F_STATUS) |
      (1UL << VIRTIO_NET_F_CTRL_VQ) |
      // (1UL << VIRTIO_F_ANY_LAYOUT) |
      (1UL << VIRTIO_NET_F_SPEED_DUPLEX) |
      (1UL << VIRTIO_NET_F_GUEST_ANNOUNCE);

    bzero(&net_config_, sizeof(net_config_));
    GenerateRandomMac(net_config_.mac);
    net_config_.status = VIRTIO_NET_S_LINK_UP;
    net_config_.duplex = 1;
    net_config_.speed = 10000;
    net_config_.mtu = DEFAULT_MTU;
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
    if (backend_) {
      delete dynamic_cast<Object*>(backend_);
      backend_ = nullptr;
    }
    VirtioPci::Disconnect();
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
    std::string network_type = "uip";
    if (has_key("backend")) {
      network_type = std::get<std::string>(key_values_["backend"]);
    }
    backend_ = dynamic_cast<NetworkBackendInterface*>(Object::Create(network_type.c_str()));
    MV_ASSERT(backend_);
    MacAddress mac;
    memcpy(mac.data, net_config_.mac, sizeof(mac.data));
    backend_->Initialize(this, mac);

    if (has_key("mtu")) {
      net_config_.mtu = std::get<uint64_t>(key_values_["mtu"]);
      backend_->SetMtu(net_config_.mtu);
    } else {
      backend_->SetMtu(DEFAULT_MTU);
    }
  }

  void Reset() {
    /* Reset all queues */
    VirtioPci::Reset();
  
    /* MQ is not supported yet */
    AddQueue(1024, std::bind(&VirtioNetwork::OnReceive, this, 0));
    AddQueue(1024, std::bind(&VirtioNetwork::OnTransmit, this, 1));
    AddQueue(64, std::bind(&VirtioNetwork::OnControl, this, 2));

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

  void EnableQueue(uint16_t queue_index) {
    VirtioPci::EnableQueue(queue_index);

    /* Some low verison driver doesn't support MTU configuration, set it to normal value */
    if (!(driver_features_ & (1UL << VIRTIO_NET_F_MTU))) {
      net_config_.mtu = DEFAULT_MTU;
      backend_->SetMtu(net_config_.mtu);
    }
  }

  void OnReceive(int queue_index) {
    MV_UNUSED(queue_index);
    if (backend_) {
      backend_->OnReceiveAvailable();
    }
  }

  void OnTransmit(int queue_index) {
    auto &vq = queues_[queue_index];
  
    while (auto element = PopQueue(vq)) {
      HandleTransmit(vq, element);
      PushQueue(vq, element);
    }
    NotifyQueue(vq);
  }

  void OnControl(int queue_index) {
    auto &vq = queues_[queue_index];
  
    while (auto element = PopQueue(vq)) {
      HandleControl(vq, element);
      PushQueue(vq, element);
    }
    NotifyQueue(vq);
  }


  virtual bool WriteBuffer(void* buffer, size_t size) {
    VirtQueue& vq = queues_[0];
    if (!vq.enabled) {
      /* Drop all packets if device is not ready */
      return true;
    }

    std::vector<VirtElement*> elements;

    virtio_net_hdr_v1* net_header = nullptr;
    size_t net_header_length = sizeof(*net_header);
  
    size_t offset = 0;
    while (offset < size) {
      auto element = PopQueue(vq);
      if (!element) {
        if (debug_) {
          MV_ERROR("network queue is full, queue size=%d", vq.size);
        }
        return false;
      }
      element->length = 0;

      if (offset == 0) {
        /* Prepend virtio net header to the buffer vector*/
        auto &iov = element->vector.front();
        net_header = (virtio_net_hdr_v1*)iov.iov_base;
        bzero(net_header, net_header_length);
        element->length += net_header_length;

        /* Big packet mode has standalone buffer for virtio net header */ 
        if (iov.iov_len == net_header_length) {
          element->vector.pop_front();
        } else {
          iov.iov_base = (uint8_t*)iov.iov_base + net_header_length;
          iov.iov_len -= net_header_length;
        }
      }

      size_t remain_bytes = size - offset;
      for (auto &iov : element->vector) {
        size_t bytes = iov.iov_len < remain_bytes ? iov.iov_len : remain_bytes;
        memcpy(iov.iov_base, (uint8_t*)buffer + offset, bytes);
        offset += bytes;
        remain_bytes -= bytes;
        element->length += bytes;
      }

      elements.push_back(element);
      if (offset < size) {
        MV_ERROR("mergeable rxbuf mode is not supported yet. offset=%lu size=%lu mtu=%u",
          offset, size, net_config_.mtu);
      }
    }

    net_header->num_buffers = elements.size();
    PushQueueMultiple(vq, elements);
    NotifyQueue(vq);
    return true;
  }

  void HandleTransmit(VirtQueue& vq, VirtElement* element) {
    MV_UNUSED(vq);

    auto &vector = element->vector;
    MV_ASSERT(vector.size() >= 1);
    auto &front = vector.front();
    virtio_net_hdr_v1* header = (virtio_net_hdr_v1*)front.iov_base;
    MV_ASSERT(header->gso_type == VIRTIO_NET_HDR_GSO_NONE);

    if (front.iov_len == sizeof(*header)) {
      vector.pop_front();
    } else {
      front.iov_base = &header[1];
      front.iov_len -= sizeof(*header);
    }
    backend_->OnFrameFromGuest(vector);
  }

  void HandleControl(VirtQueue& vq, VirtElement* element) {
    MV_UNUSED(vq);

    auto &vector = element->vector;
    MV_ASSERT(vector.size() >= 3);

    virtio_net_ctrl_hdr* control = (virtio_net_ctrl_hdr*)vector.front().iov_base;
    vector.pop_front();

    if (debug_) {
      MV_LOG("control cls=0x%x cmd=0x%x vector size=%d", control->cls, control->cmd, vector.size());
    }

    uint8_t* status = (uint8_t*)vector.back().iov_base;
    MV_ASSERT(vector.back().iov_len == 1);
    vector.pop_back();

    element->length = sizeof(*status);
    auto &iov = vector.front();

    switch (control->cls)
    {
    case VIRTIO_NET_CTRL_RX_NOBCAST:
      if (debug_) {
        MV_LOG("no broadcast");
      }
      *status = VIRTIO_NET_OK;
      break;
    default:
      *status = VIRTIO_NET_ERR;
      MV_HEXDUMP("control packet", iov.iov_base, iov.iov_len);
      MV_PANIC("unhandled control class=0x%x command=0x%x", control->cls, control->cmd);
      break;
    }
  }
};

DECLARE_DEVICE(VirtioNetwork);
