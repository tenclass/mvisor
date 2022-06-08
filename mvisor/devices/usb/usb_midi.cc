/*
 * MVisor USB MIDI
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

#include <cstring>
#include <deque>

#include "device_interface.h"
#include "device_manager.h"
#include "logger.h"
#include "usb.h"
#include "usb_descriptor.h"
#include "usb_hid.h"

enum {
  STR_MANUFACTURER = 1,
  STR_PRODUCT,
  STR_CONFIG,
};

static const UsbStringsDescriptor strings_desc = {
  [0] = "",
  [STR_MANUFACTURER]     = "Tenclass",
  [STR_PRODUCT]          = "Tenclass USB Midi",
  [STR_CONFIG]           = "HID Midi",
  [4]                    = "HID Midi",
  [5]                    = "HID Midi",
  [6]                    = "END",
};


static const UsbDeviceDescriptor device_desc = {
  .bLength = 18,
  .bDescriptorType = 1,
  .bcdUSB = 0x110,
  .bDeviceClass = 0,
  .bDeviceSubClass = 0,
  .bDeviceProtocol = 0,
  .bMaxPacketSize0 = 64,
  .idVendor = 0x2467,
  .idProduct = 0x2016,
  .bcdDevice = 0x0032,
  .iManufacturer = STR_MANUFACTURER,
  .iProduct = STR_PRODUCT,
  .iSerialNumber = 0,
  .bNumConfigurations = 1,
  .configurations = (UsbConfigurationDescriptor[]) {
    { 
      .bLength = 9,
      .bDescriptorType = 2,
      .wTotalLength = 117,
      .bNumInterfaces = 2,
      .bConfigurationValue = 1,
      .iConfiguration = STR_CONFIG,
      .bmAttributes = 0xc0,
      .bMaxPower = 50,
      .interfaces =(UsbInterfaceDescriptor[]) {
        {
          .bLength = 9,
          .bDescriptorType = 4,
          .bInterfaceNumber = 0,
          .bAlternateSetting = 0,
          .bNumEndpoints = 0,
          .bInterfaceClass = 1,
          .bInterfaceSubClass = 1,
          .bInterfaceProtocol = 0,
          .iInterface = 0,
          .ndesc = 1,
          .descriptors = (UsbOtherDescriptor[]) {
            {
              //MIDI Adapter Class-specific AC Interface Descriptor (MIDI10.pdf Appendix B.3.2)
              .data = (uint8_t[]){
                9,			    /* sizeof(usbDescrCDC_HeaderFn): length of descriptor in bytes */
                0x24,			  /* descriptor type 0x24: CS_INTERFACE - special to USB, so not defined in usbdrv.h */
                1,			    /* header functional descriptor */
                0x0, 0x01,  /* bcdADC */
                9, 0,			  /* wTotalLength */
                1,			    /* */
                1,			    /* */
              }
            }
          }
        },
        {
          .bLength = 9,
          .bDescriptorType = 4,
          .bInterfaceNumber = 1,
          .bAlternateSetting = 0,
          .bNumEndpoints = 2,
          .bInterfaceClass = 1,
          .bInterfaceSubClass = 3,
          .bInterfaceProtocol = 0,
          .iInterface = 0,
          .ndesc = 7,
          .descriptors = (UsbOtherDescriptor[]) {
            {
              // Class-specific MIDI Streaming Interface Descriptor: Header Descriptor
              .data = (uint8_t[]) {
                7,			    /* length of descriptor in bytes */
                0x24,			  /* descriptor type 0x24: CS_INTERFACE */
                1,			    /* header functional descriptor */
                0x0, 0x01,	/* bcdADC */
                81, 0,			/* wTotalLength */
              }
            },
            {
              // Class-specific MIDI Streaming Interface Descriptor: MIDI IN Jack descriptor
              .data = (uint8_t[]) {
                6,			/* bLength */
                0x24,		/* descriptor type 0x24: CS_INTERFACE */
                2,			/* MIDI_IN_JACK desc subtype */
                1,			/* EXTERNAL bJackType */
                1,			/* bJackID */
                4,			/* iJack */
              }
            },
            {
              // Class-specific MIDI Streaming Interface Descriptor: MIDI IN Jack descriptor
              .data = (uint8_t[]) {
                6,			/* bLength */
                0x24,		/* descriptor type 0x24: CS_INTERFACE */
                2,			/* MIDI_IN_JACK desc subtype */
                2,			/* EMBEDDED bJackType */
                2,			/* bJackID */
                0,			/* iJack */
              }
            },
            {
              // Class-specific MIDI Streaming Interface Descriptor: MIDI OUT Jack descriptor
              .data = (uint8_t[]) {
                9,			/* length of descriptor in bytes */
                0x24,		/* descriptor type 0x24: CS_INTERFACE */
                3,			/* MIDI_OUT_JACK descriptor */
                1,			/* EMBEDDED bJackType */
                3,			/* bJackID */
                1,			/* No of input pins */
                2,			/* BaSourceID */
                1,			/* BaSourcePin */
                3,			/* iJack */
              }
            },
            {
              // Class-specific MIDI Streaming Interface Descriptor: MIDI OUT Jack descriptor
              .data = (uint8_t[]) {
                9,			/* bLength of descriptor in bytes */
                0x24,		/* bDescriptorType */
                3,			/* MIDI_OUT_JACK bDescriptorSubtype */
                2,			/* EXTERNAL bJackType */
                4,			/* bJackID */
                1,			/* bNrInputPins */
                1,			/* baSourceID (0) */
                1,			/* baSourcePin (0) */
                0,			/* iJack */
              }
            },
            {
              // Class-specific MIDI Streaming Interface Descriptor: MIDI IN Jack descriptor
              .data = (uint8_t[]) {
                6,			/* bLength */
                0x24,		/* descriptor type 0x24: CS_INTERFACE */
                2,			/* MIDI_IN_JACK desc subtype */
                2,			/* EXTERNAL bJackType */
                5,			/* bJackID */
                0,			/* iJack */
              }
            },
            {
              // Class-specific MIDI Streaming Interface Descriptor: MIDI OUT Jack descriptor
              .data = (uint8_t[]) {
                9,			/* bLength of descriptor in bytes */
                0x24,		/* bDescriptorType */
                3,			/* MIDI_OUT_JACK bDescriptorSubtype */
                1,			/* EXTERNAL bJackType */
                6,			/* bJackID */
                1,			/* bNrInputPins */
                5,			/* baSourceID (0) */
                1,			/* baSourcePin (0) */
                5,			/* iJack */
              }
            }
          },
          .endpoints = (UsbEndpointDescriptor[]) {
             {
              .bLength = 9,
              .bDescriptorType = 5,
              .bEndpointAddress = 0x82,
              .bmAttributes = 2,
              .wMaxPacketSize = 64,
              .bInterval = 4,
              .bRefresh = 0,
              .bSynchAddress = 0,
              .is_audio = true,

              //Class-specific MS Bulk IN Endpoint Descriptor (MIDI10.pdf Appendix Descriptor B.6.2)
              .extra = (uint8_t[]) {
                6,			/* bLength of descriptor in bytes */
                0x25,		/* bDescriptorType 0x25: CS_ENDPOINT */
                1,			/* bDescriptorSubtype */
                2,			/* bNumEmbMIDIJack (0) */
                3,			/* baAssocJackID (0) */
                6       /* baAssocJackID (0) */
              }
            },
            {
              .bLength = 9,
              .bDescriptorType = 5,
              .bEndpointAddress = 2,
              .bmAttributes = 2,
              .wMaxPacketSize = 64,
              .bInterval = 4,
              .bRefresh = 0,
              .bSynchAddress = 0,
              .is_audio = true,

              //Class-specific MS Bulk OUT Endpoint (MIDI10.pdf Appendix Descriptor B.5.2)
              .extra = (uint8_t[]) {
                5,			/* bLength of descriptor in bytes */
                0x25,		/* bDescriptorType 0x25: CS_ENDPOINT */
                1,			/* bDescriptorSubtype */
                1,			/* bNumEmbMIDIJack  */
                1,			/* baAssocJackID (0) */
              }
            }
          }
        }
      }
    }
  }
};

class UsbMidi : public UsbHid, public MidiInputInterface {
 private:
  std::mutex mutex_;
  std::deque<MidiEvent> queue_;
  uint max_queue_size_ = 16;
  bool is_start_ = false;

 public:
  UsbMidi() { SetupDescriptor(&device_desc, &strings_desc); }

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

    auto data_size = sizeof(MidiEvent);
    MV_ASSERT(length >= (int)data_size);
    memcpy(data, &event, data_size);
    return data_size;
  }

  /* This interface function is called by the UI thread, so use a mutex */
  virtual bool QueueMidiEvent(MidiEvent event) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_start_) {
      return false;
    }

    queue_.push_back(event);
    while (queue_.size() > max_queue_size_) {
      queue_.pop_front();
    }

    manager_->io()->Schedule([this]() { NotifyEndpoint(0x82); });
    return true;
  }

  virtual bool InputAcceptable() { return configured(); }
  virtual void Start() { is_start_ = true; }
  virtual void Stop() { is_start_ = false; }
  virtual bool is_start() { return is_start_; }
};

DECLARE_DEVICE(UsbMidi);
