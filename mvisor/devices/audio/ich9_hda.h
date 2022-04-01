/* 
 * MVisor ICH9 High Definition Audio Device
 * Copyright (C) 2021 Terrence <terrence@tenclass.com>
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

#ifndef _MVISOR_DEVICES_AUDIO_ICH9_HDA_H
#define _MVISOR_DEVICES_AUDIO_ICH9_HDA_H

#include <cstdint>

struct Ich9HdaBdlEntry {
  uint64_t address;
  uint32_t length;
  uint32_t flags;
} __attribute__((packed));

struct Ich9HdaStream {
  uint8_t   control;
  uint8_t   pad0;
  uint8_t   stripe_control : 2;
  uint8_t   traffic_priority : 1;
  uint8_t   bidirectional : 1;
  uint8_t   stream_id : 4;
  uint8_t   status;
  uint32_t  link_position_in_buffer;
  uint32_t  cyclic_buffer_length;
  uint16_t  last_valid_index;
  uint8_t   pad1[2];

  uint16_t  fifo_size;
  uint16_t  format;
  uint8_t   pad2[4];
  uint32_t  bdl_base0;
  uint32_t  bdl_base1;
} __attribute__((packed));

struct Ich9HdaRegisters {
  /* 0x00-0x20 global */
  uint16_t  global_capabilities;      // includes number of DMA engines for input and output streams
  uint8_t   minor_version;
  uint8_t   major_version;
  uint16_t  output_payload_capacity;
  uint16_t  input_payload_capacity;
  uint32_t  global_control;           // used to reset the link and codec
  uint16_t  wake_enable;
  uint16_t  state_change_status;
  uint8_t   pad1[0x10];
  /* 0x20-0x30 interrupts */
  uint32_t  interrupt_control;
  uint32_t  interrupt_status;
  uint8_t   pad2[0x8];
  /* 0x30-0x40 misc */
  uint32_t  wall_clock_counter;
  uint8_t   pad3[0xC];
  /* 0x40-0x60 dma engine */
  uint32_t  corb_base0;
  uint32_t  corb_base1;
  uint16_t  corb_write_pointer;
  uint16_t  corb_read_pointer;
  uint8_t   corb_control;
  uint8_t   corb_status;
  uint8_t   corb_size;
  uint8_t   pad4[0x1];
  uint32_t  rirb_base0;
  uint32_t  rirb_base1;
  uint16_t  rirb_write_pointer;
  uint16_t  rirb_interrupt_count;
  uint8_t   rirb_control;
  uint8_t   rirb_status;
  uint8_t   rirb_size;
  uint8_t   pad5[0x1];
  /* 0x60-0x80 not used */
  uint32_t  icw;
  uint32_t  irr;
  uint32_t  ics;
  uint8_t   pad6[0x4];
  uint64_t  dp_base;
  uint8_t   pad7[0x8];
  /* 0x80-0x180 streams */
  Ich9HdaStream streams[8];
} __attribute__((packed));

#endif // _MVISOR_DEVICES_AUDIO_ICH9_HDA_H
