/* 
 * MVisor USB 1.0 UHCI
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

#ifndef _MVISOR_DEVICES_USB_UHCI_INTERNAL_H
#define _MVISOR_DEVICES_USB_UHCI_INTERNAL_H

#include <cstdint>

#define UHCI_LINK_TERM    (1 << 0)
#define UHCI_LINK_QUEUE   (1 << 1)
#define UHCI_LINK_VF      (1 << 2)

struct UhciTransferDescriptor {
  uint32_t  link;

  uint32_t  actual_length : 11;
  uint32_t  reserved1     : 5;

  /* status */
  uint32_t  reserved4     : 1;
  uint32_t  bitstuff      : 1;
  uint32_t  timeout       : 1;
  uint32_t  nak           : 1;
  uint32_t  babble        : 1;
  uint32_t  buffer_error  : 1;
  uint32_t  stalled       : 1;
  uint32_t  active        : 1;

  uint32_t  interrupt     : 1;
  uint32_t  isochronous   : 1;
  uint32_t  low_speed     : 1;
  uint32_t  error_count   : 2;
  uint32_t  short_packet  : 1;
  uint32_t  reserved2     : 2;

  uint32_t  pid           : 8;
  uint32_t  device        : 7;
  uint32_t  endpoint      : 4;
  uint32_t  data01        : 1;
  uint32_t  reserved3     : 1;
  uint32_t  max_length    : 11;

  uint32_t  buffer;
} __attribute__((packed));

struct UhciQueueHead {
  uint32_t  link;
  uint32_t  element_link;
} __attribute__((packed));

struct UhciRegisters {
  uint16_t  usb_command;
  uint16_t  usb_status;
  uint16_t  usb_interrupt_enable;
  uint16_t  frame_number;
  uint32_t  frame_list_base;
  uint8_t   frame_timing;
  uint8_t   reserved[3];
  uint16_t  port_sc[8];
} __attribute__((packed));


#define UHCI_CMD_FGR      (1 << 4)
#define UHCI_CMD_EGSM     (1 << 3)
#define UHCI_CMD_GRESET   (1 << 2)
#define UHCI_CMD_HCRESET  (1 << 1)
#define UHCI_CMD_RS       (1 << 0)

#define UHCI_STS_HCHALTED (1 << 5)
#define UHCI_STS_HCPERR   (1 << 4)
#define UHCI_STS_HSERR    (1 << 3)
#define UHCI_STS_RD       (1 << 2)
#define UHCI_STS_USBERR   (1 << 1)
#define UHCI_STS_USBINT   (1 << 0)

#define TD_CTRL_SPD     (1 << 29)
#define TD_CTRL_ERROR_SHIFT  27
#define TD_CTRL_IOS     (1 << 25)
#define TD_CTRL_IOC     (1 << 24)
#define TD_CTRL_ACTIVE  (1 << 23)
#define TD_CTRL_STALL   (1 << 22)
#define TD_CTRL_BABBLE  (1 << 20)
#define TD_CTRL_NAK     (1 << 19)
#define TD_CTRL_TIMEOUT (1 << 18)

#define UHCI_PORT_SUSPEND (1 << 12)
#define UHCI_PORT_RESET (1 << 9)
#define UHCI_PORT_LSDA  (1 << 8)
#define UHCI_PORT_RSVD1 (1 << 7)
#define UHCI_PORT_RD    (1 << 6)
#define UHCI_PORT_ENC   (1 << 3)
#define UHCI_PORT_EN    (1 << 2)
#define UHCI_PORT_CSC   (1 << 1)
#define UHCI_PORT_CCS   (1 << 0)

#define UHCI_PORT_READ_ONLY    (0x1bb)
#define UHCI_PORT_WRITE_CLEAR  (UHCI_PORT_CSC | UHCI_PORT_ENC)

#endif // _MVISOR_DEVICES_USB_UHCI_INTERNAL_H
