/* 
 * MVisor USB Tablet
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

#include "usb_hid.h"
#include <deque>
#include <cstring>
#include "usb_descriptor.h"
#include "usb.h"

enum {
  STR_MANUFACTURER = 1,
  STR_PRODUCT,
  STR_CONFIG,
  STR_SERIAL
};

static const UsbStringsDescriptor strings_desc = {
  [0] = "",
  [STR_MANUFACTURER]     = "Tenclass",
  [STR_PRODUCT]          = "Tenclass USB Tablet",
  [STR_CONFIG]           = "HID Tablet",
  [STR_SERIAL]           = "28754",
};

static const UsbInterfaceDescriptor interfaces[] = {
  {
    .bInterfaceNumber   = 0,
    .bNumEndpoints      = 1,
    .bInterfaceClass    = USB_CLASS_HID,
    .ndesc              = 1,
    .descriptors = (UsbOtherDescriptor[]) {
      {
        /* HID descriptor */
        .data = (uint8_t[]) {
            0x09,          /*  u8  bLength */
            USB_DT_HID,    /*  u8  bDescriptorType */
            0x01, 0x00,    /*  u16 HID_class */
            0x00,          /*  u8  country_code */
            0x01,          /*  u8  num_descriptors */
            USB_DT_REPORT, /*  u8  type: Report */
            74, 0,         /*  u16 len */
        }
      }
    },
    .endpoints = (UsbEndpointDescriptor[]) {
      {
        .bEndpointAddress      = USB_DIR_IN | 0x01,
        .bmAttributes          = USB_ENDPOINT_XFER_INT,
        .wMaxPacketSize        = 8,
        .bInterval             = 4, /* 2 ^ (4-1) * 125 usecs = 1 ms */
      }
    }
  }
};

static const UsbDeviceDescriptor device_desc = {
  .bcdUSB                        = 0x0200,
  .bMaxPacketSize0               = 64,
  .idVendor                      = 0x0627,
  .idProduct                     = 0x0001,
  .bcdDevice                     = 0,
  .iManufacturer                 = STR_MANUFACTURER,
  .iProduct                      = STR_PRODUCT,
  .iSerialNumber                 = STR_SERIAL,
  .bNumConfigurations            = 1,
  .configurations = (UsbConfigurationDescriptor[]) {
    {
      .bNumInterfaces        = 1,
      .bConfigurationValue   = 1,
      .iConfiguration        = STR_CONFIG,
      .bmAttributes          = USB_CFG_ATT_ONE | USB_CFG_ATT_WAKEUP,
      .bMaxPower             = 50,
      .interfaces = interfaces
    }
  }
};

static const uint8_t hid_report_desc[] = {
  0x05, 0x01,		/* Usage Page (Generic Desktop) */
  0x09, 0x02,		/* Usage (Mouse) */
  0xa1, 0x01,		/* Collection (Application) */
  0x09, 0x01,		/*   Usage (Pointer) */
  0xa1, 0x00,		/*   Collection (Physical) */
  0x05, 0x09,		/*     Usage Page (Button) */
  0x19, 0x01,		/*     Usage Minimum (1) */
  0x29, 0x03,		/*     Usage Maximum (3) */
  0x15, 0x00,		/*     Logical Minimum (0) */
  0x25, 0x01,		/*     Logical Maximum (1) */
  0x95, 0x03,		/*     Report Count (3) */
  0x75, 0x01,		/*     Report Size (1) */
  0x81, 0x02,		/*     Input (Data, Variable, Absolute) */
  0x95, 0x01,		/*     Report Count (1) */
  0x75, 0x05,		/*     Report Size (5) */
  0x81, 0x01,		/*     Input (Constant) */
  0x05, 0x01,		/*     Usage Page (Generic Desktop) */
  0x09, 0x30,		/*     Usage (X) */
  0x09, 0x31,		/*     Usage (Y) */
  0x15, 0x00,		/*     Logical Minimum (0) */
  0x26, 0xff, 0x7f,	/*     Logical Maximum (0x7fff) */
  0x35, 0x00,		/*     Physical Minimum (0) */
  0x46, 0xff, 0x7f,	/*     Physical Maximum (0x7fff) */
  0x75, 0x10,		/*     Report Size (16) */
  0x95, 0x02,		/*     Report Count (2) */
  0x81, 0x02,		/*     Input (Data, Variable, Absolute) */
  0x05, 0x01,		/*     Usage Page (Generic Desktop) */
  0x09, 0x38,		/*     Usage (Wheel) */
  0x15, 0x81,		/*     Logical Minimum (-0x7f) */
  0x25, 0x7f,		/*     Logical Maximum (0x7f) */
  0x35, 0x00,		/*     Physical Minimum (same as logical) */
  0x45, 0x00,		/*     Physical Maximum (same as logical) */
  0x75, 0x08,		/*     Report Size (8) */
  0x95, 0x01,		/*     Report Count (1) */
  0x81, 0x06,		/*     Input (Data, Variable, Relative) */
  0xc0,		/*   End Collection */
  0xc0,		/* End Collection */
};

struct PointerEvent {
  int x;
  int y;
  int z;
  int buttons; 
};

class UsbTablet : public UsbHid {
 private:
  std::deque<PointerEvent> queue_;
  uint idle_;

 public:
  UsbTablet() {
    SetupDescriptor(&device_desc, &strings_desc);
  }

  virtual int OnControl(uint request, uint value, uint index, uint8_t* data, int length) {
    int ret = UsbHid::OnControl(request, value, index, data, length);
    if (ret >= 0) {
      return ret;
    }
  
    switch (request)
    {
    /* hid specific requests */
    case InterfaceRequest | USB_REQ_GET_DESCRIPTOR:
      if ((value >> 8) == 0x22) {
        size_t copy = (size_t)length < sizeof(hid_report_desc) ? length : sizeof(hid_report_desc);
        memcpy(data, hid_report_desc, copy);
        return copy;
      } else {
        return USB_RET_STALL;
      }
    case HID_SET_IDLE:
      idle_ = uint8_t(value >> 8);
      return 0;
    default:
      MV_PANIC("not implemented request=0x%x value=0x%x index=0x%x", request, value, index);
      return USB_RET_STALL;
    }
  }

  virtual int OnInputData(uint8_t* data, int length) {
    if (queue_.size() == 0) {
      return USB_RET_NAK;
    }

    MV_PANIC("send data");
    return 0;
  }
};

DECLARE_DEVICE(UsbTablet);
