/* 
 * MVisor USB Device
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

#include "usb_device.h"
#include <cstring>
#include "usb.h"

UsbPacket* UsbDevice::CreatePacket(uint direction, uint stream_id, uint64_t id, VoidCallback on_complete) {
  auto packet = new UsbPacket {
    .direction = direction,
    .stream_id = stream_id,
    .id = id,
    .status = USB_RET_SUCCESS,
    .size = 0,
    .OnComplete = on_complete
  };
  packet->Release = [packet]() {
    delete packet;
  };
  return packet;
}

void UsbDevice::HandlePacket(UsbPacket* packet) {
  packet->status = USB_RET_SUCCESS;

  if (packet->control_parameter) {
    OnControlPacket(packet);
  } else {
    OnDataPacket(packet);
  }

  if (packet->status == USB_RET_NAK) {
    // Handle the packet later
    if (debug_) {
      MV_LOG("NAK packet id=0x%lx", packet->id);
    }
    return;
  }
  packet->OnComplete();
  packet->Release();
}

void UsbDevice::OnControlPacket(UsbPacket* packet) {
  uint8_t* setup_buf = (uint8_t*)&packet->control_parameter;
  uint request  = (setup_buf[0] << 8) | setup_buf[1];
  uint value    = (setup_buf[3] << 8) | setup_buf[2];
  uint index    = (setup_buf[5] << 8) | setup_buf[4];

  uint setup_len = (setup_buf[7] << 8) | setup_buf[6];
  if (debug_) {
    MV_LOG("control dir=0x%x request=0x%x value=0x%x index=0x%x setup_len=0x%x",
      packet->direction, request, value, index, setup_len);
  }
  
  uint8_t buffer[setup_len];
  if (packet->direction == USB_TOKEN_OUT && setup_len) {
    CopyPacketData(packet, buffer, setup_len);
  }

  int ret = OnControl(request, value, index, buffer, setup_len);
  if (ret < 0) {
    packet->status = ret;
    return;
  }

  if (packet->direction == USB_TOKEN_IN) {
    CopyPacketData(packet, buffer, ret);
  }
}

void UsbDevice::OnDataPacket(UsbPacket* packet) {
  uint8_t buffer[packet->size];
  if (packet->direction == USB_TOKEN_OUT) {
    CopyPacketData(packet, buffer, packet->size);
    int ret = OnOutputData(buffer, packet->size);
    if (ret < 0) {
      packet->status = ret;
      return;
    }
  } else if (packet->direction == USB_TOKEN_IN) {
    int ret = OnInputData(buffer, packet->size);
    if (ret < 0) {
      packet->status = ret;
      return;
    }
    CopyPacketData(packet, buffer, ret);
  }
}

void UsbDevice::CopyPacketData(UsbPacket* packet, uint8_t* data, int length) {
  MV_ASSERT(packet->content_length + length <= packet->size);

  size_t left = length;
  uint8_t* ptr = data;
  for (auto &v : packet->iov) {
    size_t copy = left;
    if (copy > v.iov_len) {
      copy = v.iov_len;
    }
    if (packet->direction == USB_TOKEN_IN) {
      memcpy(v.iov_base, ptr, copy);
    } else {
      memcpy(ptr, v.iov_base, copy);
    }
    ptr += copy;
    left -= copy;
    if (left == 0)
      break;
  }
  packet->content_length += length;
}

int UsbDevice::OnInputData(uint8_t* data, int length) {
  MV_PANIC("not implemented");
  return USB_RET_STALL;
}

int UsbDevice::OnOutputData(uint8_t* data, int length) {
  MV_PANIC("not implemented");
  return USB_RET_STALL;
}

int UsbDevice::OnControl(uint request, uint value, uint index, uint8_t* data, int length) {
  switch (request)
  {
  case DeviceOutRequest | USB_REQ_SET_ADDRESS:
    MV_PANIC("set address to %d", value);
    return 0;

  case DeviceRequest | USB_REQ_GET_DESCRIPTOR:
    return GetDescriptor(value, data, length);
  
  case DeviceRequest | USB_REQ_GET_CONFIGURATION:
    data[0] = config_ ? config_->bConfigurationValue : 0;
    return 1;
  
  case DeviceOutRequest | USB_REQ_SET_CONFIGURATION:
    return SetConfiguration(value);
  
  case DeviceRequest | USB_REQ_GET_STATUS:
    return GetStatus(data, length);
  
  case DeviceOutRequest | USB_REQ_CLEAR_FEATURE:
    if (value == USB_DEVICE_REMOTE_WAKEUP) {
      remote_wakeup_ = false;
      return 0;
    }
    break;
  
  case DeviceOutRequest | USB_REQ_SET_FEATURE:
    if (value == USB_DEVICE_REMOTE_WAKEUP) {
      remote_wakeup_ = true;
      return 0;
    }
    break;

  case DeviceOutRequest | USB_REQ_SET_SEL:
  case DeviceOutRequest | USB_REQ_SET_ISOCH_DELAY:
    if (speed_ == USB_SPEED_SUPER) {
      return 0;
    }
    break;

  case InterfaceRequest | USB_REQ_GET_INTERFACE:
    if (config_ && index < config_->bNumInterfaces) {
      data[0] = alternate_settings_[index];
      return 1;
    }
    break;

  case InterfaceOutRequest | USB_REQ_SET_INTERFACE:
    return SetInterface(index, value);

  case VendorDeviceRequest | 'Q':
  case VendorInterfaceRequest | 'Q':
    MV_PANIC("not implemented");
    break;

  }
  return USB_RET_STALL;
}

void UsbDevice::SetupDescriptor(const UsbDeviceDescriptor* device_desc,
  const UsbStringsDescriptor* strings_desc) {
  device_descriptor_ = device_desc;
  strings_descriptor_ = strings_desc;
}

int UsbDevice::CopyStringsDescriptor(uint index, uint8_t* data, int length) {
  if (length < 4)
    return USB_RET_IOERROR;
  
  if (index == 0) {
    data[0] = 4;
    data[1] = USB_DT_STRING;
    data[2] = 9;
    data[3] = 4;
    return 4;
  }

  const char* str = (*strings_descriptor_)[index];
  if (str == nullptr) {
    return 0;
  }
  int bLength = strlen(str) * 2 + 2;
  data[0] = bLength;
  data[1] = USB_DT_STRING;
  int pos = 2;
  for (int i = 0; pos + 1 < bLength && pos + 1 < length;) {
    data[pos++] = str[i++];
    data[pos++] = 0;
  }
  return pos;
}

int UsbDevice::CopyConfigurationDescriptor(uint index, uint8_t* data, int length) {
  if (index >= device_descriptor_->bNumConfigurations) {
    return USB_RET_IOERROR;
  }

  uint8_t buffer[4096];
  uint16_t wTotalLength = 0;
  auto config = &device_descriptor_->configurations[index];
  
  // Copy configuration
  memcpy(&buffer[wTotalLength], config, config->bLength);
  wTotalLength += config->bLength;

  // Copy interfaces
  for (int i = 0; i < config->bNumInterfaces; i++) {
    // interface
    auto interface = &config->interfaces[i];
    memcpy(&buffer[wTotalLength], interface, interface->bLength);
    wTotalLength += interface->bLength;
    // other
    for (int j = 0; j < interface->ndesc; j++) {
      auto other = &interface->descriptors[j];
      uint length = other->data[0];
      memcpy(&buffer[wTotalLength], other->data, length);
      wTotalLength += length;
    }
    // endpoint
    for (int j = 0; j < interface->bNumEndpoints; j++) {
      auto endpoint = &interface->endpoints[j];
      uint length = endpoint->is_audio ? endpoint->bLength : endpoint->bLength - 2;
      memcpy(&buffer[wTotalLength], endpoint, length);
      buffer[wTotalLength] = length;
      wTotalLength += length;

      if (endpoint->extra) {
        length = endpoint->extra[0];
        memcpy(&buffer[wTotalLength], endpoint->extra, length);
        wTotalLength += length;
      }
    }
  }

  *(uint16_t*)&buffer[2] = wTotalLength;
  if (length > wTotalLength) {
    length = wTotalLength;
  }
  memcpy(data, buffer, length);
  return length;
}

int UsbDevice::GetDescriptor(uint value, uint8_t* data, int length) {
  uint8_t type = value >> 8;
  uint8_t index = value & 0xFF;

  switch (type)
  {
  case USB_DT_DEVICE:
    if (length > device_descriptor_->bLength) {
      length = device_descriptor_->bLength;
    }
    memcpy(data, device_descriptor_, length);
    return length;
  
  case USB_DT_CONFIG:
    return CopyConfigurationDescriptor(index, data, length);
  
  case USB_DT_STRING:
    return CopyStringsDescriptor(index, data, length);

  default:
    MV_PANIC("unknown type=%d", type);
    return USB_RET_STALL;
  }
}

int UsbDevice::GetStatus(uint8_t* data, int length) {
  MV_PANIC("not impl");
  return USB_RET_STALL;
}

int UsbDevice::SetConfiguration(uint value) {
  if (value == 0) {
    configuration_ = 0;
    config_ = nullptr;
  } else {
    for (int i = 0; i < device_descriptor_->bNumConfigurations; i++) {
      auto c = &device_descriptor_->configurations[i];
      if (c->bConfigurationValue == value) {
        config_ = c;
        configuration_ = value;
      }
    }
  }
  return 0;
}

int UsbDevice::SetInterface(uint index, uint value) {
  MV_PANIC("not impl");
  return USB_RET_STALL;
}
