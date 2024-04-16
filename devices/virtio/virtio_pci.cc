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
#include "virtio_pci.pb.h"

VirtioPci::VirtioPci() {
    pci_header_.vendor_id = 0x1AF4;
    pci_header_.subsys_vendor_id = 0x1AF4;
    pci_header_.command = PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;
    pci_header_.irq_pin = 1;

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

    bzero(&common_config_, sizeof(common_config_));
    /* Device common features */
    device_features_ = (1UL << VIRTIO_RING_F_INDIRECT_DESC) | (1UL << VIRTIO_RING_F_EVENT_IDX) | \
      (1UL << VIRTIO_F_VERSION_1);
    driver_features_ = 0;

    common_config_.num_queues = queues_.size();
    use_ioevent_ = true;
}

void VirtioPci::Disconnect() {
  if (use_ioevent_) {
    for (uint index = 0; index < queues_.size(); index++) {
      if (queues_[index].enabled) {
        uint64_t notify_address = pci_bars_[4].address + 0x3000 + index * 4;
        manager_->UnregisterIoEvent(this, kIoResourceTypeMmio, notify_address);
      }
    }
  }
  PciDevice::Disconnect();
}

void VirtioPci::Reset() {
  PciDevice::Reset();
  isr_status_ = 0;
  for (uint index = 0; index < queues_.size(); index++) {
    queues_[index].index = index;
    if (queues_[index].enabled && use_ioevent_) {
      uint64_t notify_address = pci_bars_[4].address + 0x3000 + index * 4;
      manager_->UnregisterIoEvent(this, kIoResourceTypeMmio, notify_address);
    }
    queues_[index].enabled = false;
    queues_[index].size = 0;
  }
}

bool VirtioPci::SaveState(MigrationWriter* writer) {
  VirtioPciState state;
  auto common = state.mutable_common_config();
  common->set_guest_feature(driver_features_);
  common->set_msix_config(common_config_.msix_config);
  common->set_device_status(common_config_.device_status);
  common->set_queue_select(common_config_.queue_select);

  for (uint index = 0; index < queues_.size(); index++) {
    auto q = state.add_queues();
    q->set_enabled(queues_[index].enabled);
    q->set_msix_vector(queues_[index].msix_vector);
    q->set_size(queues_[index].size);
    q->set_last_available_index(queues_[index].last_available_index);
    q->set_descriptor_table_address(queues_[index].descriptor_table_address);
    q->set_available_ring_address(queues_[index].available_ring_address);
    q->set_used_ring_address(queues_[index].used_ring_address);

    if (dynamic_cast<MigrationNetworkWriter*>(writer)) {
      manager_->AddDirtyMemory(queues_[index].used_ring_address, sizeof(VRingUsed) + queues_[index].size * sizeof(VRingUsedElement));
    }
  }
  state.set_isr_status(isr_status_);
  writer->WriteProtobuf("VIRTIO_PCI", state);
  return PciDevice::SaveState(writer);
}

bool VirtioPci::LoadState(MigrationReader* reader) {
  if (!PciDevice::LoadState(reader)) {
    return false;
  }
  VirtioPciState state;
  if (!reader->ReadProtobuf("VIRTIO_PCI", state)) {
    return false;
  }
  auto& common = state.common_config();
  driver_features_ = common.guest_feature();
  common_config_.msix_config = common.msix_config();
  common_config_.device_status = common.device_status();
  common_config_.queue_select = common.queue_select();
  
  for (uint index = 0; index < queues_.size(); index++) {
    auto& q = state.queues(index);
    queues_[index].msix_vector = q.msix_vector();
    queues_[index].size = q.size();
    queues_[index].last_available_index = q.last_available_index();
    queues_[index].descriptor_table_address = q.descriptor_table_address();
    queues_[index].available_ring_address = q.available_ring_address();
    queues_[index].used_ring_address = q.used_ring_address();
    if (q.enabled()) {
      EnableQueue(index);
    }
  }
  isr_status_ = state.isr_status();
  return true;
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

void VirtioPci::AddDescriptorToElement(VirtElement& element,  VRingDescriptor* descriptor) {
  void* host = manager_->TranslateGuestMemory(descriptor->address);
  element.vector.push_back(iovec {
    .iov_base = host,
    .iov_len = descriptor->length
  });
  element.size += descriptor->length;

  if (descriptor->flags & VRING_DESC_F_WRITE) {
    manager_->AddDirtyMemory(descriptor->address, descriptor->length);
  }
}

void VirtioPci::ReadIndirectDescriptorTable(VirtElement& element, VRingDescriptor* table) {
  VRingDescriptor* descriptor = &table[0];
  while (true) {
    AddDescriptorToElement(element, descriptor);
    if ((descriptor->flags & VRING_DESC_F_NEXT) == 0) {
      break;
    }
    descriptor = &table[descriptor->next];
  }
}

VirtElement* VirtioPci::PopQueue(VirtQueue& vq) {
  asm volatile ("mfence": : :"memory");

  if (vq.available_ring->index == vq.last_available_index) {
    return nullptr;
  }

  asm volatile ("lfence": : :"memory");

  auto element = new VirtElement;
  element->Initialize();

  auto item = vq.available_ring->items[vq.last_available_index++ % vq.size];
  if (driver_features_ & (1 << VIRTIO_RING_F_EVENT_IDX)) {
    void* end = &vq.used_ring->items[vq.size];
    *(uint16_t*)end = vq.last_available_index;
  }

  VRingDescriptor* descriptor = &vq.descriptor_table[item];
  while (true) {
    if (descriptor->flags & VRING_DESC_F_INDIRECT) {
      VRingDescriptor* table = (VRingDescriptor*)manager_->TranslateGuestMemory(descriptor->address);
      ReadIndirectDescriptorTable(*element, table);
    } else {
      AddDescriptorToElement(*element, descriptor);
    }
    if ((descriptor->flags & VRING_DESC_F_NEXT) == 0) {
      break;
    }
    descriptor = &vq.descriptor_table[descriptor->next];
  }

  element->id = item;
  element->length = 0;
  return element;
}

void VirtioPci::PushQueue(VirtQueue& vq, VirtElement* element) {
  asm volatile ("mfence": : :"memory");

  auto &item = vq.used_ring->items[vq.used_ring->index % vq.size];
  item.id = element->id;
  item.length = element->length;
  delete element;

  /* Make sure other vCPU could see the buffer before we update index. */
  asm volatile ("sfence": : :"memory");

  ++vq.used_ring->index;
}

void VirtioPci::PushQueueMultiple(VirtQueue& vq, std::vector<VirtElement*>& elements) {
  asm volatile ("mfence": : :"memory");

  auto index = vq.used_ring->index;
  for (auto element : elements) {
    auto &item = vq.used_ring->items[index++ % vq.size];
    item.id = element->id;
    item.length = element->length;
    delete element;
  }

  /* Make sure other vCPU could see the buffer before we update index. */
  asm volatile ("sfence": : :"memory");

  vq.used_ring->index = index;
}

void VirtioPci::NotifyQueue(VirtQueue& vq) {
  asm volatile ("mfence": : :"memory");

  if (driver_features_ & (1 << VIRTIO_RING_F_EVENT_IDX)) {
    /* Carefully handle the overflow */
    uint16_t compare = vq.used_ring->index - vq.available_ring->items[vq.size];
    if (compare != 1) {
      return;
    }
  } else if (vq.available_ring->flags & VRING_AVAIL_F_NO_INTERRUPT) {
    return;
  }

  /* Set queue interrupt bit */
  isr_status_ = 1;
  if (msi_config_.enabled) {
    if (vq.msix_vector == VIRTIO_MSI_NO_VECTOR) {
      MV_ERROR("msix_vector was not set correctly");
      return;
    }
    SignalMsi(vq.msix_vector);
  } else if (pci_header_.irq_pin) {
    SetIrq(isr_status_ & 1);
  }
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

void VirtioPci::EnableQueue(uint16_t queue_index) {
  auto &vq = queues_[queue_index];
  MV_ASSERT(!vq.enabled);
  vq.descriptor_table = (VRingDescriptor*)manager_->TranslateGuestMemory(vq.descriptor_table_address);
  vq.available_ring = (VRingAvailable*)manager_->TranslateGuestMemory(vq.available_ring_address);
  vq.used_ring = (VRingUsed*)manager_->TranslateGuestMemory(vq.used_ring_address);
  MV_ASSERT(vq.descriptor_table && vq.available_ring && vq.used_ring);

  if (use_ioevent_) {
    uint64_t notify_address = pci_bars_[4].address + 0x3000 + queue_index * 4;
    manager_->RegisterIoEvent(this, kIoResourceTypeMmio, notify_address);
  }

  vq.enabled = true;
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
    if (common_config_.guest_feature_select == 0) {
      uint32_t value = *(uint32_t*)data;
      driver_features_ = (driver_features_ & ~0xFFFFFFFFULL) | value;
    } else {
      uint32_t value = *(uint32_t*)data;
      driver_features_ = (driver_features_ & 0xFFFFFFFFULL) | (uint64_t(value) << 32);
    }
    break;
  }

  if (offset >= VIRTIO_PCI_COMMON_Q_SIZE) {
    auto& vq = queues_[common_config_.queue_select];
    switch (offset)
    {
    case VIRTIO_PCI_COMMON_Q_SIZE:
      vq.size = *(uint32_t*)data;
      break;
    case VIRTIO_PCI_COMMON_Q_AVAILHI:
    case VIRTIO_PCI_COMMON_Q_AVAILLO:
      vq.available_ring_address = ((uint64_t)common_config_.queue_avail_hi << 32) | common_config_.queue_avail_lo;
      break;
    case VIRTIO_PCI_COMMON_Q_USEDHI:
    case VIRTIO_PCI_COMMON_Q_USEDLO:
      vq.used_ring_address = ((uint64_t)common_config_.queue_used_hi << 32) | common_config_.queue_used_lo;
      break;
    case VIRTIO_PCI_COMMON_Q_DESCHI:
    case VIRTIO_PCI_COMMON_Q_DESCLO:
      vq.descriptor_table_address = ((uint64_t)common_config_.queue_desc_hi << 32) | common_config_.queue_desc_lo;
      break;
    case VIRTIO_PCI_COMMON_Q_MSIX: 
      vq.msix_vector = common_config_.queue_msix_vector;
      break;
    case VIRTIO_PCI_COMMON_Q_ENABLE:
      if (common_config_.queue_enable == 1) {
        EnableQueue(common_config_.queue_select);
      } else {
        MV_PANIC("%s not implemented disable queue %d", name_, common_config_.queue_select);
      }
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
  case VIRTIO_PCI_COMMON_MSIX:
    value = common_config_.msix_config;
    break;
  case VIRTIO_PCI_COMMON_STATUS:
    value = common_config_.device_status;
    break;
  case VIRTIO_PCI_COMMON_CFGGENERATION:
    value = common_config_.config_generation;
    break;
  case VIRTIO_PCI_COMMON_NUMQ:
    value = common_config_.num_queues;
    break;
  case VIRTIO_PCI_COMMON_Q_NOFF:
    value = common_config_.queue_select;
    break;
  case VIRTIO_PCI_COMMON_Q_SIZE:
    value = queues_[common_config_.queue_select].size;
    break;
  case VIRTIO_PCI_COMMON_Q_MSIX:
    value = queues_[common_config_.queue_select].msix_vector;
    break;
  case VIRTIO_PCI_COMMON_Q_ENABLE:
    value = queues_[common_config_.queue_select].enabled;
    break;
  default:
    MV_PANIC("unhandled read offset=0x%lx size=%x", offset, size);
    break;
  }
  memcpy(data, &value, size);
}

void VirtioPci::ReadDeviceConfig(uint64_t offset, uint8_t* data, uint32_t size) {
  MV_UNUSED(data);
  MV_PANIC("%s not implemented read device offset=0x%lx size=%d",
    name_, offset, size);
}

void VirtioPci::WriteDeviceConfig(uint64_t offset, uint8_t* data, uint32_t size) {
  MV_PANIC("%s not implemented write device offset=0x%lx data=0x%lx size=%d",
    name_, offset, *(uint64_t*)data, size);
}

void VirtioPci::WriteNotification(uint64_t offset, uint8_t* data, uint32_t size) {
  MV_UNUSED(data);
  MV_UNUSED(size);

  uint16_t queue = offset / 4;
  MV_ASSERT(queue < queues_.size());
  auto &vq = queues_[queue];
  if (vq.enabled) {
    if (use_ioevent_) {
      vq.notification_callback();
    } else {
      Schedule(vq.notification_callback);
    }
  } else {
    MV_LOG("%s queue %u is not enabled", name_, queue);
  }
}

void VirtioPci::Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  if (resource->base == pci_bars_[4].address) {
    if (offset < 0x1000) { /* Common config */
      WriteCommonConfig(offset, data, size);
    } else if (offset < 0x2000) { /* ISR Status */
    } else if (offset < 0x3000) { /* Device config */
      WriteDeviceConfig(offset - 0x2000, data, size);
    } else if (offset < 0x4000) { /* Notification */
      WriteNotification(offset - 0x3000, data, size);
    }
  } else if (resource->base == pci_bars_[0].address) {
    /* ignore legacy driver writes */
  } else {
    PciDevice::Write(resource, offset, data, size);
  }
}

void VirtioPci::Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  if (resource->base == pci_bars_[4].address) {
    if (offset < 0x1000) { /* Common config */
      ReadCommonConfig(offset, data, size);
    } else if (offset < 0x2000) { /* ISR Status */
      *data = isr_status_;
      isr_status_ = 0;
      SetIrq(0);
    } else if (offset < 0x3000) { /* Device config */
      ReadDeviceConfig(offset - 0x2000, data, size);
    } else if (offset < 0x4000) { /* Notification */
    }
  } else if (resource->base == pci_bars_[0].address) {
    /* let legacy driver fail */
    bzero(data, size);
  } else {
    PciDevice::Read(resource, offset, data, size);
  }
}

