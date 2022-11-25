/* 
 * MVisor USB 1.0 UHCI
 * Reference: https://wiki.osdev.org/Universal_Host_Controller_Interface
 * Reference: http://www.ece.mcgill.ca/~zzilic/426/USB.ppt
 * Specification: ftp://ftp.netbsd.org/pub/NetBSD/misc/blymn/uhci11d.pdf
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

#include "uhci_host.h"
#include "device_manager.h"
#include "usb.h"
#include "uhci_host.pb.h"


#define UHCI_FRAME_TIMER_FREQUENCY    1000
#define UHCI_FRAME_TIMER_INTERVAL_NS  (NS_PER_SECOND / UHCI_FRAME_TIMER_FREQUENCY)


UhciHost::UhciHost() {
  pci_header_.class_code = 0x0C0300;
  pci_header_.subsys_vendor_id = 0x1AF4;
  pci_header_.subsys_id = 0x1100;
  pci_header_.irq_pin = 1;

  /* Specification Release 1.0 */
  pci_header_.data[0x60] = 0x10;

  AddPciBar(4, 32, kIoResourceTypePio);
}

void UhciHost::Connect() {
  PciDevice::Connect();

  /* USB ports support low speed and full speed */
  for (size_t i = 0; i < max_ports_; i++) {
    port_states_[i].id = i + 1;
    port_states_[i].speed_mask = 0b0011;
    port_states_[i].device = nullptr;
  }

  /* Connect USB devices to ports */
  for (auto device : children_) {
    UsbDevice* usb = dynamic_cast<UsbDevice*>(device);
    if (usb) {
      AttachUsbDevice(usb);
    }
  }
}

bool UhciHost::AttachUsbDevice(UsbDevice* device) {
  for (auto &port_state : port_states_) {
    if (port_state.device)
      continue;
    if ((port_state.speed_mask & (1 << device->speed())) == 0)
      continue;
    port_state.device = device;
    if (debug_) {
      MV_LOG("Attach USB device %s to port %d", device->name(), port_state.id);
    }
    return true;
  }
  MV_PANIC("failed to attach USB device %s", device->name());
  return false;
}

void UhciHost::Disconnect() {
  Halt();
  PciDevice::Disconnect();
}

bool UhciHost::SaveState(MigrationWriter* writer) {
  UhciHostState state;
  auto operational = state.mutable_operational();
  operational->set_usb_command(uhci_.usb_command);
  operational->set_usb_status(uhci_.usb_status);
  operational->set_usb_interrupt_enable(uhci_.usb_interrupt_enable);
  operational->set_frame_number(uhci_.frame_number);
  operational->set_frame_timing(uhci_.frame_timing);
  operational->set_frame_list_base(uhci_.frame_list_base);

  for (uint i = 0; i < max_ports_; i++) {
    auto port = state.add_ports();
    port->set_status_control(uhci_.port_sc[i]);
  }
  writer->WriteProtobuf("UHCI", state);
  return PciDevice::SaveState(writer);
}

bool UhciHost::LoadState(MigrationReader* reader) {
  if (!PciDevice::LoadState(reader)) {
    return false;
  }

  UhciHostState state;
  if (!reader->ReadProtobuf("UHCI", state)) {
    return false;
  }
  auto& operational = state.operational();
  uhci_.usb_command = operational.usb_command();
  uhci_.usb_status = operational.usb_status();
  uhci_.usb_interrupt_enable = operational.usb_interrupt_enable();
  uhci_.frame_number = operational.frame_number();
  uhci_.frame_timing = operational.frame_timing();
  uhci_.frame_list_base = operational.frame_list_base();

  if (uhci_.frame_list_base) {
    frame_list_ = (uint32_t*)manager_->TranslateGuestMemory(uhci_.frame_list_base);
  }

  for (uint i = 0; i < max_ports_; i++) {
    auto& port = state.ports(i);
    uhci_.port_sc[i] = port.status_control();
  }

  if (uhci_.usb_command & UHCI_CMD_RS) {
    Run();
  }
  return true;
}

void UhciHost::Reset() {
  PciDevice::Reset();
  
  bzero(&uhci_, sizeof(uhci_));
  uhci_.frame_timing = 64;
  Halt();

  for (size_t i = 0; i < max_ports_; i++) {
    uhci_.port_sc[i] = 0x0080;
    ResetPort(i);
  }
  UpdateIrqLevel();
}

void UhciHost::UpdateIrqLevel() {
  int level = 0;
  if (((uhci_.usb_interrupt_enable & 0b1100) && (uhci_.usb_status & UHCI_STS_USBINT)) ||
      ((uhci_.usb_interrupt_enable & 0b0001) && (uhci_.usb_status & UHCI_STS_USBERR)) ||
      ((uhci_.usb_interrupt_enable & 0b0010) && (uhci_.usb_status & UHCI_STS_RD)) ||
      ((uhci_.usb_status & UHCI_STS_HSERR)) ||
      ((uhci_.usb_status & UHCI_STS_HCPERR))
  ) {
    level = 1;
  }
  SetIrq(level);
}

void UhciHost::ResetPort(uint index) {
  /* set connect status */
  if (port_states_[index].device) {
    uhci_.port_sc[index] |= UHCI_PORT_CCS | UHCI_PORT_CSC;
  } else {
    if (uhci_.port_sc[index] & UHCI_PORT_CCS) {
      uhci_.port_sc[index] &= ~UHCI_PORT_CCS;
      uhci_.port_sc[index] |= UHCI_PORT_CSC;
    }
    if (uhci_.port_sc[index] & UHCI_PORT_EN) {
      uhci_.port_sc[index] &= ~UHCI_PORT_EN;
      uhci_.port_sc[index] |= UHCI_PORT_ENC;
    }
  }

  /* force global resume and set resume detect */
  if (uhci_.usb_command & UHCI_CMD_EGSM) {
    uhci_.usb_command |= UHCI_CMD_FGR;
    uhci_.usb_status |= UHCI_STS_RD;
    UpdateIrqLevel();
  }
}

void UhciHost::Run() {
  if (frame_timer_ == nullptr) {
    frame_timer_ = AddTimer(UHCI_FRAME_TIMER_INTERVAL_NS, true, [this]() {
      OnFrameTimer();
    });
  }
  uhci_.usb_status &= ~UHCI_STS_HCHALTED;
}

void UhciHost::Halt() {
  if (frame_timer_) {
    RemoveTimer(frame_timer_);
    frame_timer_ = nullptr;
  }

  /* Cleanup queues */
  for (auto it = queues_.begin(); it != queues_.end(); it++) {
    if (it->second->transfer) {
      FreeTransfer(it->second->transfer);
    }
    delete it->second;
  }
  queues_.clear();

  uhci_.usb_status |= UHCI_STS_HCHALTED;
}

void UhciHost::SetError(UhciTransferDescriptor* td, int status) {
  switch (status) {
  case USB_RET_NAK:
    td->nak = 1;
    break;
  case USB_RET_STALL:
    td->stalled = 1;
    break;
  case USB_RET_BABBLE:
    td->stalled = 1;
    td->babble = 1;
    break;
  case USB_RET_IOERROR:
  case USB_RET_NODEV:
  default:
    td->timeout = 1;
    td->error_count = 0;
    break;
  }

  td->active = 0;
  uhci_.usb_status |= UHCI_STS_USBERR;
}

UsbDevice* UhciHost::FindDevice(UhciTransferDescriptor* td) {
  for (auto &port_state : port_states_) {
    if (port_state.device && port_state.device->device_address() == td->device) {
      return port_state.device;
    }
  }
  return nullptr;
}

void UhciHost::NotifyEndpoint(UsbDevice* device, uint endpoint_address) {
  for (auto it = queues_.begin(); it != queues_.end(); it++) {
    if (it->second->device == device && it->second->endpoint_address == endpoint_address) {
      it->second->nak = false;
      break;
    }
  }
}

void UhciHost::CompleteTransfer(UhciTransfer* transfer, uint32_t* next_link) {
  MV_ASSERT(transfer->tds[0]->active);

  auto packet = transfer->packet;
  if (transfer->isochronous) {
    for (auto td: transfer->tds) {
      td->active = 0;
    }
  }

  transfer->completed = true;
  if (packet->status != USB_RET_SUCCESS) {
    transfer->stop = true;
    return SetError(transfer->tds[0], packet->status);
  }

  size_t left = transfer->packet->content_length;
  for (auto td : transfer->tds) {
    if (td->pid == USB_TOKEN_SETUP) {
      td->actual_length = 8 - 1;
    } else {
      size_t chunk = td->max_length + 1;
      if (chunk > left) {
        chunk = left;
      }
      left -= chunk;
      td->actual_length = chunk - 1;
    }
    td->timeout = 0;
    td->nak = 0;
    td->active = 0;

    if (td->pid == USB_TOKEN_IN && td->short_packet && td->actual_length < td->max_length) {
      transfer->interrupt_on_completion = true;
      transfer->stop = true;
      break;
    }
    
    if (next_link) {
      // advance element pointer
      *next_link = td->link;
    }
  }
}

void UhciHost::FreeTransfer(UhciTransfer* transfer) {
  delete transfer->packet;
  delete transfer;
}

/* Create a transfer object for a USB transaction */
UhciTransfer* UhciHost::CreateTransfer(UhciTransferDescriptor* td, uint32_t link) {
  /* Find device */
  auto device = FindDevice(td);
  if (!device) {
    MV_ERROR("failed to find usb device by id %d", td->device);
    SetError(td, USB_RET_NODEV);
    return nullptr;
  }

  /* A transaction may have multiple transfer descriptors */
  std::vector<UhciTransferDescriptor*> tds;
  tds.push_back(td);

  if (td->endpoint == 0) {
    /* Control endpoint */
    if (td->pid == USB_TOKEN_SETUP) {
      while (!(td->link & UHCI_LINK_TERM)) {
        td = (UhciTransferDescriptor*)manager_->TranslateGuestMemory(td->link & ~0xF);
        if (!td->active) {
          break;  // don't add inactive TDs
        }
        tds.push_back(td);

        if (tds.size() >= 3 && tds[1]->pid != tds.back()->pid) {
          break;  // ack TD is already pushed
        }
      }

      if (tds.size() < 2) {
        MV_ERROR("setup tds size=%lu", tds.size());
        return nullptr;
      }
    } else {
      MV_ERROR("invalid pid %x", td->pid);
    }
  } else {
    MV_ASSERT(td->pid == USB_TOKEN_IN || td->pid == USB_TOKEN_OUT);
  }

  uint endpoint_address;
  if (tds[0]->pid == USB_TOKEN_SETUP) {
    endpoint_address = (tds[1]->pid == USB_TOKEN_IN ? 0x80 : 0) | tds[1]->endpoint;
  } else {
    endpoint_address = (tds[0]->pid == USB_TOKEN_IN ? 0x80 : 0) | tds[0]->endpoint;
  }

  /* Build USB packet that we pass it to device later */
  auto packet = new UsbPacket;
  packet->endpoint_address = endpoint_address;
  packet->stream_id = 0;
  packet->id = link;
  packet->status = USB_RET_SUCCESS;
  packet->content_length = 0;
  packet->control_parameter = 0;
  packet->size = 0;

  if (tds[0]->pid == USB_TOKEN_SETUP) {
    auto setup = manager_->TranslateGuestMemory(tds[0]->buffer);
    memcpy(&packet->control_parameter, setup, tds[0]->max_length + 1);
    
    for (size_t i = 1; i < tds.size() - 1; i++) {
      packet->iov.push_back(iovec {
        .iov_base = manager_->TranslateGuestMemory(tds[i]->buffer),
        .iov_len = size_t(tds[i]->max_length + 1)
      });
      if (endpoint_address & 0x80) {
        manager_->AddDirtyMemory(tds[i]->buffer, packet->iov.back().iov_len);
      }
      packet->size += packet->iov.back().iov_len;
    }
  } else {
    for (auto td : tds) {
      packet->iov.push_back(iovec {
        .iov_base = manager_->TranslateGuestMemory(td->buffer),
        .iov_len = size_t(td->max_length + 1)
      });
      if (endpoint_address & 0x80) {
        manager_->AddDirtyMemory(td->buffer, packet->iov.back().iov_len);
      }
      packet->size += packet->iov.back().iov_len;
    }
  }

  /* Create a transfer to hold all the TDs and USB packet */
  auto transfer = new UhciTransfer;
  transfer->packet = packet;
  transfer->device = device;
  transfer->endpoint_address = endpoint_address;
  transfer->link = link;
  transfer->stop = false;
  transfer->completed = false;
  transfer->isochronous = tds[0]->isochronous;
  for (auto td: tds) {
    if (td->interrupt) {
      transfer->interrupt_on_completion = true;
    }
  }

  transfer->tds = std::move(tds);
  return transfer;
}

void UhciHost::OnFrameTimer() {
  uint32_t link = frame_list_[uhci_.frame_number & 0x3FF];
  std::unordered_set<uint32_t> visited_qh;
  bool interrupt = false;
  UhciQueue* current_queue = nullptr;
  UhciTransfer* transfer = nullptr;

  /* Cleanup invalid queues */
  for (auto it = queues_.begin(); it != queues_.end();) {
    if (--it->second->valid == 0) {
      auto q = it->second;
      if (q->transfer) {
        FreeTransfer(q->transfer);
      }
      delete q;
      it = queues_.erase(it);
    } else {
      it++;
    }
  }

  while (!(link & UHCI_LINK_TERM)) {
    /* link is Queue head */
    if (link & UHCI_LINK_QUEUE) {
      /* avoid infinite loop */
      if (visited_qh.find(link) != visited_qh.end()) {
        break;
      }
      visited_qh.insert(link);

      auto it = queues_.find(link & ~0xF);
      if (it == queues_.end()) {
        auto q = new UhciQueue;
        q->device = nullptr;
        q->endpoint_address = 0;
        q->qh = (UhciQueueHead*)manager_->TranslateGuestMemory(link & ~0xF);
        q->transfer = nullptr;
        q->nak = false;
        queues_[link & ~0xF] = q;
        current_queue = q;
      } else {
        current_queue = it->second;
      }
      
      current_queue->valid = 32;
      if (current_queue->qh->element_link & UHCI_LINK_TERM) {
        link = current_queue->qh->link;
        current_queue = nullptr;
      } else {
        link = current_queue->qh->element_link;
      }
      continue;
    }

    /* link is Transfer descriptor */
    auto gpa = link & ~0xF;
    auto td = (UhciTransferDescriptor*)manager_->TranslateGuestMemory(gpa);
    manager_->AddDirtyMemory(gpa, sizeof(UhciTransferDescriptor));
    if (!td->active) {
      if (td->interrupt) {
        interrupt = true;
      }
      goto next_qh;
    }

    /* TD without queue not supported yet */
    if (current_queue == nullptr) {
      MV_ERROR("TD without queue %x not supported yet", link);
      goto next_qh;
    }
    
    /* Ignore ACK packet of control endpoint */
    if (td->endpoint == 0 && td->pid == USB_TOKEN_OUT) {
      td->active = 0;
      current_queue->qh->element_link = td->link;
      if (td->interrupt) {
        interrupt = true;
      }
      goto next_qh;
    }

    /* If current transfer link differs from the queue, it becomes invalid */
    transfer = current_queue->transfer;
    if (transfer) {
      if (transfer->link != link) {
        FreeTransfer(transfer);
        current_queue->transfer = transfer = nullptr;
      } else if (current_queue->nak) {
        link = current_queue->qh->link;
        continue;
      }
    }

    if (!transfer) {
      current_queue->transfer = transfer = CreateTransfer(td, link);
    }

    if (transfer) {
      if (!current_queue->device) {
        current_queue->device = transfer->device;
        current_queue->endpoint_address = transfer->endpoint_address;
      }
      current_queue->device->HandlePacket(transfer->packet);
    
      if (transfer->packet->status == USB_RET_NAK) {
        current_queue->nak = true;
      } else {
        CompleteTransfer(transfer, current_queue ? &current_queue->qh->element_link : nullptr);
        bool stop = transfer->stop;
        if (transfer->interrupt_on_completion) {
          interrupt = true;
        }
        FreeTransfer(transfer);
        current_queue->transfer = nullptr;

        if (!stop && (current_queue->qh->element_link & UHCI_LINK_VF)) {
          link = current_queue->qh->element_link;
          continue; // not tested yet
        }
      }
    }

    /* Next QH */
next_qh:
    link = current_queue ? current_queue->qh->link : td->link;
  }

  uhci_.frame_number++;
  if (interrupt) {
    uhci_.usb_status |= UHCI_STS_USBINT;
    UpdateIrqLevel();
  }
}

void UhciHost::WriteUsbCommand(uint16_t value) {
  if ((value & UHCI_CMD_RS) && !(uhci_.usb_command & UHCI_CMD_RS)) {
    Run();
  } else if (!(value & UHCI_CMD_RS)) {
    Halt();
  }
  if ((value & UHCI_CMD_GRESET) | (value & UHCI_CMD_HCRESET)) {
    Reset();
    return;
  }

  uhci_.usb_command = value;
}

void UhciHost::WritePortStatusControl(uint index, uint16_t value) {
  if (value & UHCI_PORT_RESET && !(uhci_.port_sc[index] & UHCI_PORT_RESET)) {
    /* Reset the device */
    if (port_states_[index].device) {
      port_states_[index].device->Reset();
    }
  }

  uhci_.port_sc[index] &= UHCI_PORT_READ_ONLY;
  if (!(uhci_.port_sc[index] & UHCI_PORT_CCS)) {
    value &= ~UHCI_PORT_EN;
  }
  uhci_.port_sc[index] |= (value & ~UHCI_PORT_READ_ONLY);
  uhci_.port_sc[index] &= ~(value & UHCI_PORT_WRITE_CLEAR);
}

void UhciHost::Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  if (resource->base == pci_bars_[4].address) {
    switch (offset)
    {
    case 0x00:  // USB COMMAND
      WriteUsbCommand(*(uint16_t*)data);
      break;
    case 0x02:  // USB Status
      uhci_.usb_status &= ~*(uint16_t*)data;
      UpdateIrqLevel();
      break;
    case 0x04:  // USB Interrupt Enable
      uhci_.usb_interrupt_enable = *(uint16_t*)data;
      UpdateIrqLevel();
      break;
    case 0x06:  // Frame Number
      if (uhci_.usb_status & UHCI_STS_HCHALTED) {
        uhci_.frame_number = *(uint16_t*)data;
      }
      break;
    case 0x08:  // Frame List Base Address
      MV_ASSERT(size == 4);
      uhci_.frame_list_base = *(uint32_t*)data;
      frame_list_ = (uint32_t*)manager_->TranslateGuestMemory(uhci_.frame_list_base);
      break;
    case 0x0C:  // Frame Timing
      uhci_.frame_timing = data[0];
      break;
    case 0x10 ... 0x1E:  // Port status control
      WritePortStatusControl((offset - 0x10) / 2, *(uint16_t*)data);
      break;
    default:
      MV_PANIC("write 0x%lx data=0x%x size=%u", offset, data[0], size);
      break;
    }
  } else {
    PciDevice::Write(resource, offset, data, size);
  }
}

void UhciHost::Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
  if (resource->base == pci_bars_[4].address) {
    MV_ASSERT(offset + size <= sizeof(uhci_));
    memcpy(data, (uint8_t*)&uhci_ + offset, size);
  } else {
    PciDevice::Read(resource, offset, data, size);
  }
}
