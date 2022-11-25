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
#include "logger.h"
#include "device_manager.h"
#include "usb_device.pb.h"

UsbDevice::UsbDevice() {
  set_default_parent_class("XhciHost", "Piix3Uhci");

  speed_ = kUsbSpeedFull;
}

void UsbDevice::Connect() {
  Device::Connect();
  host_ = dynamic_cast<UsbHost*>(parent_);
  MV_ASSERT(host_);
}

void UsbDevice::Disconnect() {
  RemoveEndpoints();
  Device::Disconnect();
}

void UsbDevice::Reset() {
  Device::Reset();

  configuration_value_ = 0;
  config_ = nullptr;
  device_address_ = 0;

  RemoveEndpoints();
}

void UsbDevice::RemoveEndpoints() {
  for (auto endpoint : endpoints_) {
    delete endpoint;
  }
  endpoints_.clear();
}

bool UsbDevice::SaveState(MigrationWriter* writer) {
  UsbDeviceState state;
  state.set_device_address(device_address_);
  state.set_configuration_value(configuration_value_);
  state.set_remote_wakeup(remote_wakeup_);
  writer->WriteProtobuf("USB_DEVICE", state);
  return Device::SaveState(writer);
}

bool UsbDevice::LoadState(MigrationReader* reader) {
  if (!Device::LoadState(reader)) {
    return false;
  }
  UsbDeviceState state;
  if (!reader->ReadProtobuf("USB_DEVICE", state)) {
    return false;
  }
  device_address_ = state.device_address();
  remote_wakeup_ = state.remote_wakeup();
  SetConfiguration(state.configuration_value());
  return true;
}

bool UsbDevice::HandlePacket(UsbPacket* packet) {
  if (packet->endpoint_address & 0xF) {
    OnDataPacket(packet);
  } else {
    OnControlPacket(packet);
  }
  return true;
}

void UsbDevice::OnControlPacket(UsbPacket* packet) {
  uint8_t* setup_buf = (uint8_t*)&packet->control_parameter;
  uint request  = (setup_buf[0] << 8) | setup_buf[1];
  uint value    = (setup_buf[3] << 8) | setup_buf[2];
  uint index    = (setup_buf[5] << 8) | setup_buf[4];

  uint setup_len = (setup_buf[7] << 8) | setup_buf[6];
  if (debug_) {
    MV_LOG("control ep=0x%x request=0x%x value=0x%x index=0x%x setup_len=0x%x",
      packet->endpoint_address, request, value, index, setup_len);
  }
  
  uint8_t buffer[setup_len];
  if (packet->endpoint_address == 0x00 && setup_len) {
    CopyPacketData(packet, buffer, setup_len);
  }

  packet->status = USB_RET_SUCCESS;
  int ret = OnControl(request, value, index, buffer, setup_len);
  if (ret < 0) {
    packet->status = ret;
    return;
  }

  if (packet->endpoint_address == 0x80) {
    CopyPacketData(packet, buffer, ret);
  }
}

void UsbDevice::OnDataPacket(UsbPacket* packet) {
  uint8_t buffer[packet->size];
  packet->status = USB_RET_SUCCESS;

  if (packet->endpoint_address & 0x80) { // IN
    int ret = OnInputData(packet->endpoint_address, buffer, packet->size);
    if (ret < 0) {
      packet->status = ret;
    } else {
      CopyPacketData(packet, buffer, ret);
    }
  } else { // OUT
    CopyPacketData(packet, buffer, packet->size);
    int ret = OnOutputData(packet->endpoint_address, buffer, packet->size);
    if (ret < 0) {
      packet->status = ret;
    }
  }
}

/* Ask host controller to handle data packets */
void UsbDevice::NotifyEndpoint(uint endpoint_address) {
  host_->Schedule([this, endpoint_address]() {
    host_->NotifyEndpoint(this, endpoint_address);
  });
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
    if (packet->endpoint_address & 0x80) {
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

int UsbDevice::OnInputData(uint endpoint_address, uint8_t* data, int length) {
  MV_UNUSED(endpoint_address);
  MV_UNUSED(data);
  MV_UNUSED(length);
  return USB_RET_STALL;
}

int UsbDevice::OnOutputData(uint endpoint_address, uint8_t* data, int length) {
  MV_UNUSED(endpoint_address);
  MV_UNUSED(data);
  MV_UNUSED(length);
  return USB_RET_STALL;
}

int UsbDevice::OnControl(uint request, uint value, uint index, uint8_t* data, int length) {
  switch (request)
  {
  case DeviceOutRequest | USB_REQ_SET_ADDRESS:
    device_address_ = value;
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
    return GetMicrosoftOsDescriptor(index, data, length);
  }

  MV_ERROR("not implemented request=0x%x value=0x%x index=0x%x", request, value, index);
  return USB_RET_STALL;
}

void UsbDevice::SetupDescriptor(const UsbDeviceDescriptor* device_desc,
  const UsbStringsDescriptor* strings_desc) {
  device_descriptor_ = device_desc;
  strings_descriptor_ = strings_desc;
}

int UsbDevice::CopyStringsDescriptor(uint index, uint8_t* data, int length) {
  if (length < 4) {
    return USB_RET_IOERROR;
  }
  
  if (index == 0) {
    data[0] = 4;
    data[1] = USB_DT_STRING;
    data[2] = 9;
    data[3] = 4;
    return 4;
  }

  const char* str = (*strings_descriptor_)[index];
  if (str == nullptr) {
    MV_LOG("invalid string index=0x%x length=%d", index, length);
    return USB_RET_STALL;
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

int UsbDevice::CopyDeviceQualifier(uint8_t* data, int length) {
  uint8_t bLength = 0x0A;

  if (length < bLength) {
    return USB_RET_IOERROR;
  }

  data[0] = bLength;
  data[1] = USB_DT_DEVICE_QUALIFIER;

  data[2] = device_descriptor_->bcdUSB & 0xFF;
  data[3] = device_descriptor_->bcdUSB >> 8;
  data[4] = device_descriptor_->bDeviceClass;
  data[5] = device_descriptor_->bDeviceSubClass;
  data[6] = device_descriptor_->bDeviceProtocol;
  data[7] = device_descriptor_->bMaxPacketSize0;
  data[8] = device_descriptor_->bNumConfigurations;
  data[9] = 0; // reserved
  return bLength;
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

  case USB_DT_DEVICE_QUALIFIER:
    return CopyDeviceQualifier(data, length);

  default:
    MV_ERROR("unknown type=%d", type);
    return USB_RET_STALL;
  }
}

int UsbDevice::GetStatus(uint8_t* data, int length) {
  MV_PANIC("not implemented data=%d length=%d", data[0], length);
  return USB_RET_STALL;
}

int UsbDevice::SetConfiguration(uint value) {
  /* delete all endpoints */
  RemoveEndpoints();
  configuration_value_ = 0;
  config_ = nullptr;

  if (value == 0) {
    return 0;
  }

  for (uint i = 0; i < device_descriptor_->bNumConfigurations; i++) {
    auto c = &device_descriptor_->configurations[i];
    if (c->bConfigurationValue == value) {
      config_ = c;
      configuration_value_ = value;
      /* initialize interfaces */
      for (uint j = 0; j < config_->bNumInterfaces; j++) {
        auto interface = &config_->interfaces[j];
        for (uint k = 0; k < interface->bNumEndpoints; k++) {
          auto desc = &interface->endpoints[k];
          /* create endpoint */
          auto endpoint = new UsbEndpoint;
          endpoint->device = this;
          endpoint->address = desc->bEndpointAddress;
          endpoint->type = UsbEndpointType(desc->bmAttributes & 3);
          endpoint->interface = j;
          endpoint->interval = (1 << (desc->bInterval - 1)) * 125 / 1000;
          if (endpoint->interval < 1 || endpoint->interval > 1000) {
            MV_ERROR("invalid interval=%u", endpoint->interval);
            endpoint->interval = 1;
          }
          endpoints_.push_back(endpoint);
        }
      }
    }
  }
  return 0;
}

int UsbDevice::SetInterface(uint index, uint value) {
  MV_PANIC("not implemented index=0x%x value=0x%x", index, value);
  return USB_RET_STALL;
}

UsbEndpoint* UsbDevice::FindEndpoint(uint address) {
  for (auto endpoint : endpoints_) {
    if (endpoint->address == address) {
      return endpoint;
    }
  }
  return nullptr;
}

/* 
 * Not implemented yet. Maybe cellphones use this feature.
 * https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors 
 */
int UsbDevice::GetMicrosoftOsDescriptor(uint index, uint8_t* data, int length) {
  if (debug_) {
    MV_LOG("unhandled MsOsd index=%d data=%d length=%d", index, data[0], length);
  }
  return USB_RET_STALL;
}

