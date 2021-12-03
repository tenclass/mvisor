/* 
 * MVisor VirtIO PCI class
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
#include <linux/virtio_config.h>
#include "logger.h"
#include "device_manager.h"

VirtioPci::VirtioPci() {
    devfn_ = PCI_MAKE_DEVFN(4, 0);
    
    pci_header_.vendor_id = 0x1AF4;
    pci_header_.device_id = 0x1003;
    pci_header_.class_code = 0x078000;
    pci_header_.revision_id = 0;
    pci_header_.subsys_vendor_id = 0x1AF4;
    pci_header_.subsys_id = 0x0003;
    pci_header_.irq_pin = 1;
    pci_header_.command = PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;

    AddPciBar(0, 0x40, kIoResourceTypePio);
    AddPciBar(4, 0x4000, kIoResourceTypeMmio);

    uint8_t cap_common_config[] = {
      0x10, 0x01, 0x04, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00
    };
    uint8_t cap_isr_status[] = {
      0x10, 0x03, 0x04, 0x00, 0x00, 0x00,
      0x00, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00
    };
    uint8_t cap_device_config[] = {
      0x10, 0x04, 0x04, 0x00, 0x00, 0x00,
      0x00, 0x20, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00
    };
    uint8_t cap_notification[] = {
      0x14, 0x02, 0x04, 0x00, 0x00, 0x00,
      0x00, 0x30, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00,
      0x04, 0x00, 0x00, 0x00
    };
    uint8_t cap_pci_config_access[] = {
      0x14, 0x05, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00
    };
    AddCapability(0x09, cap_common_config, sizeof(cap_common_config));
    AddCapability(0x09, cap_isr_status, sizeof(cap_isr_status));
    AddCapability(0x09, cap_device_config, sizeof(cap_device_config));
    AddCapability(0x09, cap_notification, sizeof(cap_notification));
    AddCapability(0x09, cap_pci_config_access, sizeof(cap_pci_config_access));
    AddMsiXCapability(1, 2);

    bzero(&common_config_, sizeof(common_config_));
    bzero(driver_features_, sizeof(driver_features_));
    for (uint index = 0; index < queues_.size(); index++) {
      queues_[index].index = index;
      queues_[index].size = 0;
    }
    /* Device common features */
    device_features_ = (1UL << VIRTIO_RING_F_INDIRECT_DESC) | (1UL << VIRTIO_RING_F_EVENT_IDX) | \
      (1UL << VIRTIO_F_VERSION_1);

    common_config_.num_queues = queues_.size();
    isr_status_ = 0;
}

VirtioPci::~VirtioPci() {

}


void VirtioPci::Reset() {
}

void VirtioPci::PrintQueue(VirtQueue& vq) {
  MV_LOG("queue index=%d size=%d descriptors: ", vq.index, vq.size);
  for (int i = 0; i < vq.size; i++) {
    auto descriptor = &vq.descriptor_table[i];
    MV_LOG("descriptor address=0x%lx length=%x flags=%x next=%x", descriptor->address,
      descriptor->length, descriptor->flags, descriptor->next);
  }
  MV_LOG("avalable flags=%x index=%x: ", vq.available_ring->flags, vq.available_ring->index);
  for (int i = 0; i < vq.size; i++) {
    auto element = vq.available_ring->items[i];
    MV_LOG("avalable ring[%d]=%u", i, element);
  }
  MV_LOG("used flags=%x index=%x: ", vq.used_ring->flags, vq.used_ring->index);
  for (int i = 0; i < vq.size; i++) {
    auto &element = vq.used_ring->items[i];
    MV_LOG("used ring[%d] id=%u length=%u", i, element.id, element.length);
  }
}

bool VirtioPci::PopQueue(VirtQueue& vq, VirtElement& element) {
  if (vq.available_ring->index == vq.last_available_index) {
    return false;
  }
  // asm volatile ("lfence": : :"memory");
  
  auto item = vq.available_ring->items[vq.last_available_index++ % vq.size];
  VRingDescriptor* descriptor = &vq.descriptor_table[item];
  while (true) {
    /* FIXME: indirect is not implemented yet */
    MV_ASSERT((descriptor->flags & VRING_DESC_F_INDIRECT) == 0);

    void* host = manager_->TranslateGuestMemory(descriptor->address);
    element.vector.push_back(iovec {
      .iov_base = host,
      .iov_len = descriptor->length
    });
    if ((descriptor->flags & VRING_DESC_F_NEXT) == 0) {
      break;
    }
    descriptor = &vq.descriptor_table[descriptor->next];
  }

  element.id = item;
  element.length = 0;
  return true;
}

void VirtioPci::PushQueue(VirtQueue& vq, const VirtElement& element) {
  auto &item = vq.used_ring->items[vq.used_ring->index % vq.size];
  item.id = element.id;
  item.length = element.length;
  // asm volatile ("sfence": : :"memory");
  ++vq.used_ring->index;
}

void VirtioPci::NotifyQueue(VirtQueue& vq) {
  /* Set queue interrupt bit */
  isr_status_ = 1;
  /* Make sure MSI X Enabled */
  if (vq.msix_vector == VIRTIO_MSI_NO_VECTOR) {
    MV_PANIC("MSI X is not enabled");
  }
  SignalMsi(vq.msix_vector);
}

void VirtioPci::AddQueue(uint16_t queue_size, VoidCallback callback) {
  for (auto &vq : queues_) {
    if (vq.size != 0)
      continue;
    vq.size = queue_size;
    vq.notification_callback = callback;
    vq.descriptor_table = nullptr;
    vq.available_ring = nullptr;
    vq.used_ring = nullptr;
    vq.enabled = false;
    vq.last_available_index = 0;
    return;
  }
  MV_PANIC("exceeded queue size");
}

void VirtioPci::EnableQueue(uint16_t queue_index, uint64_t desc_gpa, uint64_t avail_gpa, uint64_t used_gpa) {
  auto &vq = queues_[queue_index];
  vq.descriptor_table = (VRingDescriptor*)manager_->TranslateGuestMemory(desc_gpa);
  vq.available_ring = (VRingAvailable*)manager_->TranslateGuestMemory(avail_gpa);
  vq.used_ring = (VRingUsed*)manager_->TranslateGuestMemory(used_gpa);
  vq.enabled = true;
  MV_ASSERT(vq.descriptor_table && vq.available_ring && vq.used_ring);
}

void VirtioPci::WriteCommonConfig(uint64_t offset, uint8_t* data, uint32_t size) {
  MV_ASSERT(offset + size <= sizeof(common_config_));
  memcpy((uint8_t*)&common_config_ + offset, data, size);
  switch (offset)
  {
  case VIRTIO_PCI_COMMON_STATUS:
    if (!common_config_.device_status) {
      Reset();
    }
    break;
  case VIRTIO_PCI_COMMON_GF:
    driver_features_[common_config_.guest_feature_select] = *(uint32_t*)data;
    if (common_config_.guest_feature_select == 0 && driver_features_[0] & VIRTIO_RING_F_EVENT_IDX) {
      MV_PANIC("FIXME: event idx is not implemented yet");
    }
    break;
  case VIRTIO_PCI_COMMON_Q_ENABLE:
    if (common_config_.queue_enable == 1) {
      EnableQueue(common_config_.queue_select,
        ((uint64_t)common_config_.queue_desc_hi << 32) | common_config_.queue_desc_lo,
        ((uint64_t)common_config_.queue_avail_hi << 32) | common_config_.queue_avail_lo,
        ((uint64_t)common_config_.queue_used_hi << 32) | common_config_.queue_used_lo
      );
    }
    break;
  case VIRTIO_PCI_COMMON_Q_MSIX: {
    auto &vq = queues_[common_config_.queue_select];
    vq.msix_vector = common_config_.queue_msix_vector;
    break;
  }
  }
}

void VirtioPci::ReadCommonConfig(uint64_t offset, uint8_t* data, uint32_t size) {
  uint64_t value = 0;
  switch (offset)
  {
  case VIRTIO_PCI_COMMON_DF:
    if (common_config_.device_feature_select == 0) {
      value = (uint32_t)device_features_;
    } else {
      value = (uint32_t)(device_features_ >> 32);
    }
    break;
  case VIRTIO_PCI_COMMON_STATUS:
    value = common_config_.device_status;
    break;
  case VIRTIO_PCI_COMMON_NUMQ:
    value = common_config_.num_queues;
    break;
  case VIRTIO_PCI_COMMON_Q_SIZE: {
    auto &vq = queues_[common_config_.queue_select];
    value = vq.size;
    break;
  }
  case VIRTIO_PCI_COMMON_Q_NOFF:
    value = common_config_.queue_select;
    break;
  case VIRTIO_PCI_COMMON_Q_MSIX: {
    auto &q = queues_[common_config_.queue_select];
    value = q.msix_vector;
    break;
  }
  default:
    MV_PANIC("unhandled read offset=0x%lx size=%x", offset, size);
    break;
  }
  memcpy(data, &value, size);
}

void VirtioPci::ReadDeviceConfig(uint64_t offset, uint8_t* data, uint32_t size) {
  MV_PANIC("not implemented");
}

void VirtioPci::WriteNotification(uint64_t offset, uint8_t* data, uint32_t size) {
  uint16_t queue = offset / 4;
  MV_ASSERT(size == 2 && queue == *(uint16_t*)data && queue < queues_.size());
  auto vq = queues_[queue];
  vq.notification_callback();
}

void VirtioPci::Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  if (ir.base == pci_bars_[4].address) {
    if (offset < 0x1000) { /* Common config */
      WriteCommonConfig(offset, data, size);
    } else if (offset < 0x2000) { /* ISR Status */
    } else if (offset < 0x3000) { /* Device config */        
      MV_PANIC("%s write %s base=0x%lx offset=0x%lx data=0x%lx size=%d",
        ir.name, "Device", ir.base, offset, *(uint64_t*)data, size);
    } else if (offset < 0x4000) { /* Notification */
      WriteNotification(offset - 0x3000, data, size);
    }
  } else {
    PciDevice::Write(ir, offset, data, size);
  }
}

void VirtioPci::Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  if (ir.base == pci_bars_[4].address) {
    if (offset < 0x1000) { /* Common config */
      ReadCommonConfig(offset, data, size);
    } else if (offset < 0x2000) { /* ISR Status */
      *data = isr_status_;
      isr_status_ = 0;
    } else if (offset < 0x3000) { /* Device config */        
      ReadDeviceConfig(offset - 0x2000, data, size);
    } else if (offset < 0x4000) { /* Notification */
    }
  } else {
    PciDevice::Read(ir, offset, data, size);
  }
}

