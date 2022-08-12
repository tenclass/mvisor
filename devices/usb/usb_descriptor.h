/* 
 * MVisor USB Descriptor
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


#ifndef _MVISOR_DEVICES_USB_USB_DESCRIPTOR_H
#define _MVISOR_DEVICES_USB_USB_DESCRIPTOR_H

#include <cstdint>

typedef const char *UsbStringsDescriptor[256];

struct UsbOtherDescriptor {
  const uint8_t*            data;
} __attribute__((packed));


struct UsbEndpointDescriptor {
  uint8_t                   bLength = 9;
  uint8_t                   bDescriptorType = 0x05;
  uint8_t                   bEndpointAddress;
  uint8_t                   bmAttributes;
  uint16_t                  wMaxPacketSize;
  uint8_t                   bInterval;
  uint8_t                   bRefresh;
  uint8_t                   bSynchAddress;

  const bool                is_audio; /* has bRefresh + bSynchAddress */
  const uint8_t*            extra;
} __attribute__((packed));

struct UsbInterfaceDescriptor {
  uint8_t                   bLength = 9;
  uint8_t                   bDescriptorType = 0x04;
  uint8_t                   bInterfaceNumber;
  uint8_t                   bAlternateSetting;
  uint8_t                   bNumEndpoints;
  uint8_t                   bInterfaceClass;
  uint8_t                   bInterfaceSubClass;
  uint8_t                   bInterfaceProtocol;
  uint8_t                   iInterface;

  uint8_t                         ndesc;
  const UsbOtherDescriptor*       descriptors;
  const UsbEndpointDescriptor*    endpoints;
} __attribute__((packed));

struct UsbConfigurationDescriptor {
  uint8_t                   bLength = 9;
  uint8_t                   bDescriptorType = 0x02;
  uint16_t                  wTotalLength = 0;
  uint8_t                   bNumInterfaces = 1;
  uint8_t                   bConfigurationValue = 1;
  uint8_t                   iConfiguration = 0;
  uint8_t                   bmAttributes;
  uint8_t                   bMaxPower = 50;

  const UsbInterfaceDescriptor*   interfaces;
} __attribute__((packed));

struct UsbDeviceDescriptor {
  uint8_t                   bLength = 18;
  uint8_t                   bDescriptorType = 0x01;
  uint16_t                  bcdUSB = 0x0200;
  uint8_t                   bDeviceClass;
  uint8_t                   bDeviceSubClass;
  uint8_t                   bDeviceProtocol;
  uint8_t                   bMaxPacketSize0 = 8;
  uint16_t                  idVendor;
  uint16_t                  idProduct;
  uint16_t                  bcdDevice;
  uint8_t                   iManufacturer = 1;
  uint8_t                   iProduct = 2;
  uint8_t                   iSerialNumber = 3;
  uint8_t                   bNumConfigurations = 1;

  const UsbConfigurationDescriptor*  configurations;
} __attribute__((packed));


#endif // _MVISOR_DEVICES_USB_USB_DESCRIPTOR_H
