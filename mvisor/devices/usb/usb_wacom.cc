/* 
 * MVisor USB Wacom
 * Copyright (C) 2022 cair <rui.cai@tenclass.com>
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

// wacom's width/height and pressure based on
// https://github.com/t3r1337/tablet/blob/49ce3cc5ff01e8eb0bdc872bbbe28c282a5a4bf0/config/wacom.cfg
#define MAX_PRESSURE 2047
#define MAX_WIDTH 15200
#define MAX_HEIGHT 9500

enum {
  STR_MANUFACTURER = 1,
  STR_PRODUCT,
  STR_SERIAL
};

static const UsbStringsDescriptor strings_desc = {
  [0] = "",
  [STR_MANUFACTURER]     = "Tenclass",
  [STR_PRODUCT]          = "Tenclass USB Wacom",
  [STR_SERIAL]           = "8EE00L1005567",
};

static const UsbInterfaceDescriptor interfaces[] = {
  {
    .bInterfaceNumber   = 0,
    .bNumEndpoints      = 1,
    .bInterfaceClass    = USB_CLASS_HID,
    .bInterfaceSubClass = 0x01,
    .bInterfaceProtocol = 0x02,
    .ndesc              = 1,
    .descriptors = (UsbOtherDescriptor[]) {
      {
        /* HID descriptor */
        .data = (uint8_t[]) {
            0x09,          /*  u8  bLength */
            USB_DT_HID,    /*  u8  bDescriptorType */
            0x10, 0x01,    /*  u16 HID_class */
            0x00,          /*  u8  country_code */
            0x01,          /*  u8  num_descriptors */
            USB_DT_REPORT, /*  u8  type: Report */
            0xe4, 0,         /*  u16 len */
        }
      }
    },
    .endpoints = (UsbEndpointDescriptor[]) {
      {
        .bEndpointAddress      = USB_DIR_IN | 0x03,
        .bmAttributes          = USB_ENDPOINT_XFER_INT,
        .wMaxPacketSize        = 16,
        .bInterval             = 4, /* 2 ^ (4-1) * 125 usecs = 1 ms */
        .is_audio              = true
      }
    }
  },
  {
    .bInterfaceNumber   = 1,
    .bNumEndpoints      = 1,
    .bInterfaceClass    = USB_CLASS_HID,
    .bInterfaceSubClass = 0x00,
    .bInterfaceProtocol = 0x02,
    .ndesc              = 1,
    .descriptors = (UsbOtherDescriptor[]) {
      {
        /* HID descriptor */
        .data = (uint8_t[]) {
            0x09,          /*  u8  bLength */
            USB_DT_HID,    /*  u8  bDescriptorType */
            0x10, 0x01,    /*  u16 HID_class */
            0x00,          /*  u8  country_code */
            0x01,          /*  u8  num_descriptors */
            USB_DT_REPORT, /*  u8  type: Report */
            0x26, 0,         /*  u16 len */
        }
      }
    },
    .endpoints = (UsbEndpointDescriptor[]) {
      {
        .bEndpointAddress      = USB_DIR_IN | 0x04,
        .bmAttributes          = USB_ENDPOINT_XFER_INT,
        .wMaxPacketSize        = 64,
        .bInterval             = 4, /* 2 ^ (4-1) * 125 usecs = 1 ms */
        .is_audio              = true
      }
    }
  }
};

static const UsbDeviceDescriptor device_desc = {
  .bcdUSB                        = 0x0200,
  .bMaxPacketSize0               = 64,
  .idVendor                      = 0x056a,
  .idProduct                     = 0x037a,
  .bcdDevice                     = 0x0100,
  .iManufacturer                 = STR_MANUFACTURER,
  .iProduct                      = STR_PRODUCT,
  .iSerialNumber                 = STR_SERIAL,
  .bNumConfigurations            = 1,
  .configurations = (UsbConfigurationDescriptor[]) {
    {
      .wTotalLength          = 59,
      .bNumInterfaces        = 2,
      .bConfigurationValue   = 1,
      .iConfiguration        = 0,
      .bmAttributes          = 0x80,
      .bMaxPower             = 249,
      .interfaces = interfaces
    }
  }
};

static const uint8_t hid_report_desc_interface[][300] = {
  {
    // interface1 hid_report_desc size = 292
    0x05,0x01,0x09,0x02,0xa1,0x01,0x85,0x01,0x09,0x01,0xa1,0x00,0x05,0x09,0x19,0x01,
    0x29,0x05,0x15,0x00,0x25,0x01,0x95,0x05,0x75,0x01,0x81,0x02,0x95,0x01,0x75,0x03,
    0x81,0x01,0x05,0x01,0x09,0x30,0x09,0x31,0x15,0x81,0x25,0x7f,0x75,0x08,0x95,0x02,
    0x81,0x06,0xc0,0xc0,0x06,0x0d,0xff,0x09,0x01,0xa1,0x01,0x85,0x02,0xa1,0x00,0x06,
    0x00,0xff,0x09,0x01,0x15,0x00,0x26,0xff,0x00,0x75,0x08,0x95,0x09,0x81,0x02,0xc0,
    0x09,0x01,0x85,0x02,0x95,0x01,0xb1,0x02,0x09,0x01,0x85,0x03,0x95,0x01,0xb1,0x02,
    0x09,0x01,0x85,0x04,0x95,0x01,0xb1,0x02,0x09,0x01,0x85,0x05,0x95,0x01,0xb1,0x02,
    0x09,0x01,0x85,0x10,0x95,0x02,0xb1,0x02,0x09,0x01,0x85,0x11,0x95,0x10,0xb1,0x02,
    0x09,0x01,0x85,0x13,0x95,0x01,0xb1,0x02,0x09,0x01,0x85,0x14,0x95,0x0d,0xb1,0x02,
    0x09,0x01,0x85,0x15,0x95,0x0e,0xb1,0x02,0x09,0x01,0x85,0x21,0x95,0x01,0xb1,0x02,
    0x09,0x01,0x85,0x22,0x95,0x01,0xb1,0x02,0x09,0x01,0x85,0x23,0x95,0x0e,0xb1,0x02,
    0x09,0x01,0x85,0x30,0x96,0x02,0x00,0xb1,0x02,0x09,0x01,0x85,0x31,0x96,0x09,0x01,
    0xb1,0x02,0x09,0x01,0x85,0x32,0x96,0x08,0x01,0xb1,0x02,0x09,0x01,0x85,0x24,0x95,
    0x1f,0xb1,0x02,0x09,0x01,0x85,0x25,0x95,0x04,0xb1,0x1f,0x85,0xc0,0x09,0x00,0x95,
    0x09,0x81,0x02,0xc0
  },
  {
    // interface2 hid_report_desc size = 102
    0x06,0x00,0xff,0x09,0x80,0xa1,0x01,0x85,0x02,0x09,0x01,0x15,0x00,0x26,0xff,0x00,
    0x75,0x08,0x95,0x3f,0x81,0x03,0x85,0x03,0x09,0x01,0x15,0x00,0x26,0xff,0x00,0x75,
    0x08,0x95,0x3f,0x81,0x03,0xc0
  }
};

class UsbWacom : public UsbHid, public WacomInputInterface {
 private:
  std::mutex mutex_;
  uint idle_ = 0;
  std::deque<WacomEvent> queue_;
  uint max_queue_size_ = 16;
  bool started_ = false;
  IoTimer* status_report_timer_ = nullptr;
  const uint8_t hid_data_report_[10] = {0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};

 public:
  UsbWacom() { SetupDescriptor(&device_desc, &strings_desc); }

  virtual void Reset() {
    // remove status report timer
    if (status_report_timer_) {
      manager_->io()->RemoveTimer(status_report_timer_);
      status_report_timer_ = nullptr;
    }
    
    UsbHid::Reset();
  }

  void StartStatusReport() {
    auto timer_callback = [this]() {
      WacomEvent event = {0};
      QueueWacomEvent(event);
    };

    if (!status_report_timer_) {
      status_report_timer_ = manager_->io()->AddTimer(5000, true, timer_callback);
    }
  }

  virtual int OnControl(uint request, uint value, uint index, uint8_t* data, int length) {
    switch (request) {
      /* hid specific requests */
      case InterfaceRequest | USB_REQ_GET_DESCRIPTOR:
        if ((value >> 8) == 0x22) {
          MV_ASSERT(index < 2);
          auto copy = (int)std::min((size_t)length, sizeof(hid_report_desc_interface[index]));
          memcpy(data, hid_report_desc_interface[index], copy);
          return copy;
        } else {
          return USB_RET_STALL;
        }
      case DeviceRequest | USB_REQ_GET_STATUS: 
        MV_ASSERT(length >= 2);
        bzero(data, 2);
        return length;
      case HID_SET_REPORT:
        return USB_RET_SUCCESS;
      case HID_GET_REPORT:
        if (value == 0x0314) {
          auto serial_length = strlen(strings_desc[STR_SERIAL]);
          MV_ASSERT(length >= (int)serial_length + 1);
          
          // copy serial number to data at index 1
          bzero(data, length);
          memcpy(data + 1, strings_desc[STR_SERIAL], serial_length);
          return serial_length + 1;
        } else if (value == 0x0303) {
          MV_ASSERT(length == 3);
          bzero(data, length);
          memcpy(data, &value, 2);
          StartStatusReport();
          return length;
        } else {
          MV_LOG("unknown report %x", value);
          return USB_RET_SUCCESS;
        }
      case HID_SET_IDLE:
        idle_ = uint8_t(value >> 8);
        return USB_RET_SUCCESS;
      default:
        return UsbHid::OnControl(request, value, index, data, length);
    }
  }

  virtual int OnInputData(uint endpoint_address, uint8_t* data, int length) {
    if (debug_) {
      MV_LOG("request=%x data=%p length=%d", endpoint_address, data, length);
    }

    std::unique_lock<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return USB_RET_NAK;
    }

    auto event = queue_.front();
    queue_.pop_front();
    lock.unlock();

    return TranslateWacomEventToHidData(event, data, length);
  }

  int TranslateWacomEventToHidData(WacomEvent& event, uint8_t* data, int length) {
    if (!event.x && !event.y && !event.pressure && !event.buttons) {
      int data_size = sizeof(hid_data_report_);
      MV_ASSERT(length >= data_size);

      // set status report
      memcpy(data, hid_data_report_, data_size);
      return data_size;
    }

    // hid base data 
    uint8_t hid_data[] = {0x02, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    int data_size = sizeof(hid_data);
    MV_ASSERT(length >= data_size);

    // set buttons and pressure
    if (event.buttons || event.pressure) {
      hid_data[1] |= 0x20;
      if (event.buttons & 1) {
        hid_data[1] |= 1;
      }
      if (event.buttons & 2) {
        hid_data[1] |= 4;
      }
      if (event.buttons & 4) {
        hid_data[1] |= 2;
      }
      auto pressure = (uint16_t)(event.pressure * MAX_PRESSURE);
      memcpy(&hid_data[6], &pressure, sizeof(uint16_t));
    }

    // prox
    hid_data[1] |= 0x40;
    auto x = (uint16_t)(event.x * MAX_WIDTH);
    memcpy(&hid_data[2], &x, sizeof(uint16_t));
    auto y = (uint16_t)(event.y * MAX_HEIGHT);
    memcpy(&hid_data[4], &y, sizeof(uint16_t));

    // range
    hid_data[1] |= 0x80;
    hid_data[8] = 0x1f;
    memcpy(data, hid_data, data_size);
    return data_size;
  }

  /* This interface function is called by the UI thread, so use a mutex */
  virtual bool QueueWacomEvent(WacomEvent event) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!started_) {
      return false;
    }

    if (debug_) {
      MV_LOG("event data=%f data=%f data=%f data=%d data=%d data=%d", event.x,
             event.y, event.buttons, event.pressure, event.tilt_x, event.tilt_y);
    }

    queue_.push_back(event);
    while (queue_.size() > max_queue_size_) {
      queue_.pop_front();
    }

    manager_->io()->Schedule([this]() { NotifyEndpoint(0x83); });
    return true;
  }

  virtual void Start() { started_ = true; }
  virtual void Stop() { started_ = false; }
  virtual bool InputAcceptable() { return started_ && configured(); }
};

DECLARE_DEVICE(UsbWacom);
