/* 
 * MVisor USB 3.0 XHCI
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

#ifndef _MVISOR_DEVICES_USB_XHCI_HOST_H
#define _MVISOR_DEVICES_USB_XHCI_HOST_H

#include <cstdint>

struct XhciCapabilityRegisters {
  uint8_t   capability_length;
  uint8_t   reserved1;
  uint16_t  interface_version;
  /* HCSPARAMS 1 */
  uint8_t   max_slots;
  uint16_t  max_interrupts; // 10 bits (1024)
  uint8_t   max_ports;
  /* HCSPARAMS 2 / 3 */
  uint32_t  hcs_params2;
  uint32_t  hcs_params3;
  uint32_t  capability_params1;
  uint32_t  doorbell_offset;
  uint32_t  runtime_registers_offset;
  uint32_t  capability_params2;
  /* Extended capabilites */
  uint32_t  extended_capabilities[8];
} __attribute__((packed));


struct XhciOperationalRegisters {
  uint32_t  usb_command;
  uint32_t  usb_status;
  uint32_t  page_size;
  uint32_t  reserved1[2];
  uint32_t  device_notification_control;
  uint64_t  command_ring_control;
  uint32_t  reserved2[4];
  uint64_t  context_base_array_pointer;
  uint32_t  configure;
} __attribute__((packed));


struct XhciPortRegisters {
  uint32_t  status_control;
  uint32_t  pm_status_control;
  uint32_t  link_info;
  uint32_t  reserved;
  /* User defined state variables */
} __attribute__((packed));

struct XhciEventRingSegment {
    uint64_t start;
    uint32_t size;
    uint32_t reserved;
} __attribute__((packed));

struct XhciInterruptRegisters {
  uint32_t  management;
  uint32_t  moderation;
  uint32_t  event_ring_table_size;
  uint32_t  reserved;
  uint64_t  event_ring_table_base;
  uint64_t  event_ring_dequeue_pointer;
  /* User defined state variables */
  XhciEventRingSegment event_ring_segment;
  uint64_t  event_ring_dequeue_index;
  uint64_t  event_ring_enqueue_index;
} __attribute__((packed));


struct XhciRuntimeRegisters {
  uint32_t  microframe_index;
  uint32_t  reserved[7];
} __attribute__((packed));



enum TRBType {
    TRB_RESERVED = 0,
    TR_NORMAL,
    TR_SETUP,
    TR_DATA,
    TR_STATUS,
    TR_ISOCH,
    TR_LINK,
    TR_EVDATA,
    TR_NOOP,
    CR_ENABLE_SLOT,
    CR_DISABLE_SLOT,
    CR_ADDRESS_DEVICE,
    CR_CONFIGURE_ENDPOINT,
    CR_EVALUATE_CONTEXT,
    CR_RESET_ENDPOINT,
    CR_STOP_ENDPOINT,
    CR_SET_TR_DEQUEUE,
    CR_RESET_DEVICE,
    CR_FORCE_EVENT,
    CR_NEGOTIATE_BW,
    CR_SET_LATENCY_TOLERANCE,
    CR_GET_PORT_BANDWIDTH,
    CR_FORCE_HEADER,
    CR_NOOP,
    ER_TRANSFER = 32,
    ER_COMMAND_COMPLETE,
    ER_PORT_STATUS_CHANGE,
    ER_BANDWIDTH_REQUEST,
    ER_DOORBELL,
    ER_HOST_CONTROLLER,
    ER_DEVICE_NOTIFICATION,
    ER_MFINDEX_WRAP,
    /* vendor specific bits */
    CR_VENDOR_NEC_FIRMWARE_REVISION  = 49,
    CR_VENDOR_NEC_CHALLENGE_RESPONSE = 50,
};


/* bit definitions */
#define USBCMD_RS       (1<<0)
#define USBCMD_HCRST    (1<<1)
#define USBCMD_INTE     (1<<2)
#define USBCMD_HSEE     (1<<3)
#define USBCMD_LHCRST   (1<<7)
#define USBCMD_CSS      (1<<8)
#define USBCMD_CRS      (1<<9)
#define USBCMD_EWE      (1<<10)
#define USBCMD_EU3S     (1<<11)

#define USBSTS_HCH      (1<<0)
#define USBSTS_HSE      (1<<2)
#define USBSTS_EINT     (1<<3)
#define USBSTS_PCD      (1<<4)
#define USBSTS_SSS      (1<<8)
#define USBSTS_RSS      (1<<9)
#define USBSTS_SRE      (1<<10)
#define USBSTS_CNR      (1<<11)
#define USBSTS_HCE      (1<<12)


#define PORTSC_CCS          (1<<0)
#define PORTSC_PED          (1<<1)
#define PORTSC_OCA          (1<<3)
#define PORTSC_PR           (1<<4)
#define PORTSC_PLS_SHIFT        5
#define PORTSC_PLS_MASK     0xf
#define PORTSC_PP           (1<<9)
#define PORTSC_SPEED_SHIFT      10
#define PORTSC_SPEED_MASK   0xf
#define PORTSC_SPEED_FULL   (1<<10)
#define PORTSC_SPEED_LOW    (2<<10)
#define PORTSC_SPEED_HIGH   (3<<10)
#define PORTSC_SPEED_SUPER  (4<<10)
#define PORTSC_PIC_SHIFT        14
#define PORTSC_PIC_MASK     0x3
#define PORTSC_LWS          (1<<16)
#define PORTSC_CSC          (1<<17)
#define PORTSC_PEC          (1<<18)
#define PORTSC_WRC          (1<<19)
#define PORTSC_OCC          (1<<20)
#define PORTSC_PRC          (1<<21)
#define PORTSC_PLC          (1<<22)
#define PORTSC_CEC          (1<<23)
#define PORTSC_CAS          (1<<24)
#define PORTSC_WCE          (1<<25)
#define PORTSC_WDE          (1<<26)
#define PORTSC_WOE          (1<<27)
#define PORTSC_DR           (1<<30)
#define PORTSC_WPR          (1<<31)

#define CRCR_RCS        (1<<0)
#define CRCR_CS         (1<<1)
#define CRCR_CA         (1<<2)
#define CRCR_CRR        (1<<3)

#define IMAN_IP         (1<<0)
#define IMAN_IE         (1<<1)

#define ERDP_EHB        (1<<3)

#define TRB_SIZE 16

enum {
    PLS_U0              =  0,
    PLS_U1              =  1,
    PLS_U2              =  2,
    PLS_U3              =  3,
    PLS_DISABLED        =  4,
    PLS_RX_DETECT       =  5,
    PLS_INACTIVE        =  6,
    PLS_POLLING         =  7,
    PLS_RECOVERY        =  8,
    PLS_HOT_RESET       =  9,
    PLS_COMPILANCE_MODE = 10,
    PLS_TEST_MODE       = 11,
    PLS_RESUME          = 15,
};

#define get_field(data, field)                  \
    (((data) >> field##_SHIFT) & field##_MASK)

#define set_field(data, newval, field) do {                     \
        uint32_t val = *data;                                   \
        val &= ~(field##_MASK << field##_SHIFT);                \
        val |= ((newval) & field##_MASK) << field##_SHIFT;      \
        *data = val;                                            \
    } while (0)


#endif // _MVISOR_DEVICES_USB_XHCI_HOST_H
