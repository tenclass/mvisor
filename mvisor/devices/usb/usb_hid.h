/* 
 * MVisor USB HID
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

#ifndef _MVISOR_DEVICES_USB_USB_HID_H
#define _MVISOR_DEVICES_USB_USB_HID_H

#include "usb_device.h"

/* HID interface requests */
#define HID_GET_REPORT   0xA101
#define HID_GET_IDLE     0xA102
#define HID_GET_PROTOCOL 0xA103
#define HID_SET_REPORT   0x2109
#define HID_SET_IDLE     0x210A
#define HID_SET_PROTOCOL 0x210B

/* HID descriptor types */
#define USB_DT_HID    0x21
#define USB_DT_REPORT 0x22
#define USB_DT_PHY    0x23


class UsbHid : public UsbDevice {
 protected:
  
 public:

};


#endif // _MVISOR_DEVICES_USB_USB_HID_H
