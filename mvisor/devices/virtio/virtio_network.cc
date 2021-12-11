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

struct MacAddress {
  uint8_t data[6];
  bool operator < (const MacAddress& a) const {
    return memcmp(data, a.data, 6) < 0;
  }
};

class VirtioNetwork : public VirtioPci {
 private:
  virtio_net_config net_config_;
  RxMode            rx_mode_;
  std::set<MacAddress> mac_table_;

 public:
  VirtioNetwork() {
    devfn_ = PCI_MAKE_DEVFN(1, 0);
    pci_header_.class_code = 0x020000;
    pci_header_.device_id = 0x1000;
    pci_header_.subsys_id = 0x0001;
    
    device_features_ |= (1UL << VIRTIO_NET_F_MAC) |
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

  void Reset() {
    /* Reset all queues */
    VirtioPci::Reset();
  
    /* MQ is not supported yet */
    AddQueue(DEFAULT_QUEUE_SIZE, std::bind(&VirtioNetwork::OnReceive, this, 0));
    AddQueue(DEFAULT_QUEUE_SIZE, std::bind(&VirtioNetwork::OnTransmit, this, 1));
    AddQueue(DEFAULT_QUEUE_SIZE, std::bind(&VirtioNetwork::OnControl, this, 2));
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
    MV_LOG("OnReceive %d", queue_index);
  }

  void OnTransmit(int queue_index) {
    auto &vq = queues_[queue_index];
    VirtElement element;
  
    while (PopQueue(vq, element)) {
      HandleTransmit(vq, element);
      PushQueue(vq, element);
      NotifyQueue(vq);
    }
  }

  void OnControl(int queue_index) {
    auto &vq = queues_[queue_index];
    VirtElement element;
  
    while (PopQueue(vq, element)) {
      HandleControl(vq, element);
      PushQueue(vq, element);
      NotifyQueue(vq);
    }
  }

  void WriteBuffer(VirtQueue& vq, void* buffer, size_t size) {
    size_t offset = 0;
    while (offset < size) {
      VirtElement element;
      if (!PopQueue(vq, element)) {
        break;
      }

      element.length = 0;
      size_t remain_bytes = size - offset;
      for (auto iov : element.write_vector) {
        size_t bytes = iov.iov_len < remain_bytes ? iov.iov_len : remain_bytes;
        memcpy(iov.iov_base, (uint8_t*)buffer + offset, bytes);
        offset += bytes;
        remain_bytes -= bytes;
        element.length += bytes;
      }

      PushQueue(vq, element);
      NotifyQueue(vq);
    }
  }

  void HandleTransmit(VirtQueue& vq, VirtElement& element) {
    // virtio_net_hdr_v1* header = (virtio_net_hdr_v1*)element.read_vector[0].iov_base;
    for (auto vec : element.read_vector) {
      DumpHex(vec.iov_base, vec.iov_len);
    }
    MV_LOG("vectors read=%lu write=%lu rs=%lu ws=%lu", element.read_vector.size(), element.write_vector.size(),
      element.read_size, element.write_size);
  }

  void HandleControl(VirtQueue& vq, VirtElement& element) {
    virtio_net_ctrl_hdr* control = (virtio_net_ctrl_hdr*)element.read_vector[0].iov_base;
    uint8_t* status = (uint8_t*)element.write_vector[0].iov_base;
    MV_ASSERT(element.write_vector[0].iov_len == 1);
    element.length = sizeof(*status);

    switch (control->cls)
    {
    case VIRTIO_NET_CTRL_RX:
      MV_ASSERT(element.read_vector[1].iov_len == 1);
      *status = ControlRxMode(control->cmd, *(uint8_t*)element.read_vector[1].iov_base);
      break;
    case VIRTIO_NET_CTRL_MAC:
      *status = ControlMacTable(control->cmd, (virtio_net_ctrl_mac*)element.read_vector[1].iov_base);
      break;
    case VIRTIO_NET_CTRL_VLAN:
      if (element.read_vector[1].iov_len == 2) {
        *status = ControlVlanTable(control->cmd, *(uint16_t*)element.read_vector[1].iov_base);
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
