/* 
 * MVisor USB Keyboard
 * Copyright (C) 2024 Terrence <terrence@tenclass.com>
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
#include "device_interface.h"
#include "device_manager.h"
#include "logger.h"

#define HID_USAGE_ERROR_ROLLOVER        0x01
#define HID_USAGE_POSTFAIL              0x02
#define HID_USAGE_ERROR_UNDEFINED       0x03

/* Indices are QEMU keycodes, values are from HID Usage Table.  Indices
 * above 0x80 are for keys that come after 0xe0 or 0xe1+0x1d or 0xe1+0x9d.  */
static const uint8_t hid_usage_keys[0x100] = {
    0x00, 0x29, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x2d, 0x2e, 0x2a, 0x2b,
    0x14, 0x1a, 0x08, 0x15, 0x17, 0x1c, 0x18, 0x0c,
    0x12, 0x13, 0x2f, 0x30, 0x28, 0xe0, 0x04, 0x16,
    0x07, 0x09, 0x0a, 0x0b, 0x0d, 0x0e, 0x0f, 0x33,
    0x34, 0x35, 0xe1, 0x31, 0x1d, 0x1b, 0x06, 0x19,
    0x05, 0x11, 0x10, 0x36, 0x37, 0x38, 0xe5, 0x55,
    0xe2, 0x2c, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e,
    0x3f, 0x40, 0x41, 0x42, 0x43, 0x53, 0x47, 0x5f,
    0x60, 0x61, 0x56, 0x5c, 0x5d, 0x5e, 0x57, 0x59,
    0x5a, 0x5b, 0x62, 0x63, 0x46, 0x00, 0x64, 0x44,
    0x45, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e,
    0xe8, 0xe9, 0x71, 0x72, 0x73, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x85, 0x00, 0x00, 0x00, 0x00,
    0x88, 0x00, 0x00, 0x87, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x8a, 0x00, 0x8b, 0x00, 0x89, 0xe7, 0x65,

    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x58, 0xe4, 0x00, 0x00,
    0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0x00,
    0x80, 0x00, 0x00, 0x00, 0x00, 0x54, 0x00, 0x46,
    0xe6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x48, 0x4a,
    0x52, 0x4b, 0x00, 0x50, 0x00, 0x4f, 0x00, 0x4d,
    0x51, 0x4e, 0x49, 0x4c, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xe3, 0xe7, 0x65, 0x66, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

enum {
  STR_MANUFACTURER = 1,
  STR_PRODUCT,
  STR_CONFIG,
  STR_SERIAL
};

static const UsbStringsDescriptor strings_desc = {
  [0] = "",
  [STR_MANUFACTURER]     = "Tenclass",
  [STR_PRODUCT]          = "Tenclass USB Keyboard",
  [STR_CONFIG]           = "HID Keyboard",
  [STR_SERIAL]           = "28755",
};

static const UsbInterfaceDescriptor interfaces[] = {
  {
    .bInterfaceNumber   = 0,
    .bNumEndpoints      = 1,
    .bInterfaceClass    = USB_CLASS_HID,
    .bInterfaceSubClass = 1, /* boot */
    .bInterfaceProtocol = 1, /* keyboard */
    .ndesc              = 1,
    .descriptors = (UsbOtherDescriptor[]) {
      {
        /* HID descriptor */
        .data = (uint8_t[]) {
            0x09,          /*  u8  bLength */
            USB_DT_HID,    /*  u8  bDescriptorType */
            0x11, 0x01,    /*  u16 HID_class */
            0x00,          /*  u8  country_code */
            0x01,          /*  u8  num_descriptors */
            USB_DT_REPORT, /*  u8  type: Report */
            0x3f, 0,       /*  u16 len */
        }
      }
    },
    .endpoints = (UsbEndpointDescriptor[]) {
      {
        .bEndpointAddress      = USB_DIR_IN | 0x01,
        .bmAttributes          = USB_ENDPOINT_XFER_INT,
        .wMaxPacketSize        = 8,
        .bInterval             = 7, /* 2 ^ (8-1) * 125 usecs = 8 ms */
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
  0x09, 0x06,		/* Usage (Keyboard) */
  0xa1, 0x01,		/* Collection (Application) */
  0x75, 0x01,		/*   Report Size (1) */
  0x95, 0x08,		/*   Report Count (8) */
  0x05, 0x07,		/*   Usage Page (Key Codes) */
  0x19, 0xe0,		/*   Usage Minimum (224) */
  0x29, 0xe7,		/*   Usage Maximum (231) */
  0x15, 0x00,		/*   Logical Minimum (0) */
  0x25, 0x01,		/*   Logical Maximum (1) */
  0x81, 0x02,		/*   Input (Data, Variable, Absolute) */
  0x95, 0x01,		/*   Report Count (1) */
  0x75, 0x08,		/*   Report Size (8) */
  0x81, 0x01,		/*   Input (Constant) */
  0x95, 0x05,		/*   Report Count (5) */
  0x75, 0x01,		/*   Report Size (1) */
  0x05, 0x08,		/*   Usage Page (LEDs) */
  0x19, 0x01,		/*   Usage Minimum (1) */
  0x29, 0x05,		/*   Usage Maximum (5) */
  0x91, 0x02,		/*   Output (Data, Variable, Absolute) */
  0x95, 0x01,		/*   Report Count (1) */
  0x75, 0x03,		/*   Report Size (3) */
  0x91, 0x01,		/*   Output (Constant) */
  0x95, 0x06,		/*   Report Count (6) */
  0x75, 0x08,		/*   Report Size (8) */
  0x15, 0x00,		/*   Logical Minimum (0) */
  0x25, 0xff,		/*   Logical Maximum (255) */
  0x05, 0x07,		/*   Usage Page (Key Codes) */
  0x19, 0x00,		/*   Usage Minimum (0) */
  0x29, 0xff,		/*   Usage Maximum (255) */
  0x81, 0x00,		/*   Input (Data, Array) */
  0xc0,		/* End Collection */
};

class UsbKeyboard : public UsbHid, public KeyboardInputInterface {
 private:
  std::deque<uint16_t> queue_;
  uint max_queue_size_ = 16;
  uint idle_;
  uint protocol_;
  uint modifiers_;
  uint leds_ = 0;
  uint keys_ = 0;
  uint8_t key_[16];

  int SetReport(uint8_t* data, int length) {
    if (length > 0) {
      /* 0x01: Num Lock LED
       * 0x02: Caps Lock LED
       * 0x04: Scroll Lock LED
       * 0x08: Compose LED
       * 0x10: Kana LED
       */
      leds_ = data[0];
      if (debug_) {
        MV_LOG("Set LED %02x", leds_);
      }
    }
    return 0;
  }

  void ProcessKeyEvent(uint16_t keycode) {
    // LCtrl, LShift, LAlt, LMeta, RCtrl, RShift, RAlt, RMeta
    uint16_t mod_keys[] = { 0x1D, 0x2A, 0x38, 0xE05B, 0xE01D, 0x36, 0xE038, 0xE05C };
    for (int i = 0; i < 8; i++) {
      if (keycode == (mod_keys[i] | 0x80)) {
        modifiers_ &= ~(1 << i);
        return;
      } else if (keycode == mod_keys[i]) {
        modifiers_ |= (1 << i);
        return;
      }
    }

    uint8_t key = keycode & 0x7f;
    uint8_t hid_code = hid_usage_keys[key];
    if (keycode & 0x80) { // key release
      for (int i = keys_ - 1; i >= 0; i--) {
        if (key_[i] == hid_code) {
          key_[i] = key_[-- keys_];
          key_[keys_] = 0x00;
          break;
        }
      }
    } else {
      int i;
      for (i = keys_ - 1; i >= 0; i--) {
        if (key_[i] == hid_code) {
          break;
        }
      }
      if (i < 0) {
        if (keys_ < sizeof(key_)) {
          key_[keys_++] = hid_code;
        }
      }
    }
  }

 public:
  UsbKeyboard() {
    SetupDescriptor(&device_desc, &strings_desc);
  }

  virtual void Reset() override {
    idle_ = 0;
    protocol_ = 1;
    modifiers_ = 0;
    bzero(key_, sizeof(key_));
    keys_ = 0;
    queue_.clear();
    UsbHid::Reset();
  }

  virtual int OnControl(uint request, uint value, uint index, uint8_t* data, int length) {
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
    case HID_SET_PROTOCOL:
      protocol_ = value;
      if (debug_) {
        MV_LOG("Set protocol %d", protocol_);
      }
      return 0;
    case HID_SET_REPORT:
      return SetReport(data, length);
    case HID_SET_IDLE:
      idle_ = uint8_t(value >> 8);
      return 0;
    default:
      return UsbHid::OnControl(request, value, index, data, length);
    }
  }

  virtual int OnInputData(uint endpoint_address, uint8_t* data, int length) {
    MV_UNUSED(endpoint_address);

    std::unique_lock<std::recursive_mutex> lock(mutex_);
    if (queue_.empty()) {
      return USB_RET_NAK;
    }

    while (!queue_.empty()) {
      auto keycode = queue_.front();
      queue_.pop_front();
      ProcessKeyEvent(keycode);
    }
    
    if (length < 2) {
      MV_ERROR("USB Keyboard: Invalid input data length %d is less than 2", length);
      return USB_RET_STALL;
    }
    data[0] = (uint8_t)modifiers_;
    data[1] = 0;
    if (keys_ > 6) {
      // Roll over
      memset(data + 2, HID_USAGE_ERROR_ROLLOVER, std::min(8, length) - 2);
    } else {
      memcpy(data + 2, key_, std::min(8, length) - 2);
    }
    return std::min(8, length);
  }

  /* This interface function is called by the UI thread, so use a mutex */
  virtual bool QueueKeyboardEvent(uint8_t scancode[10], uint8_t modifiers) override {
    MV_UNUSED(modifiers);
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    uint16_t keycode = 0;
    if (scancode[0] && scancode[1]) {
      keycode = (scancode[0] << 8) | scancode[1];
    } else {
      keycode = scancode[0];
    }
    queue_.push_back(keycode);
    while (queue_.size() > max_queue_size_) {
      queue_.pop_front();
    }
    
    NotifyEndpoint(0x81);
    return true;
  }

  virtual bool InputAcceptable() {
    return configured();
  }

};

DECLARE_DEVICE(UsbKeyboard);
