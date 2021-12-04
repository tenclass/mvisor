/* 
 * MVisor QXL
 * Copyright (C) 2021 Terrence <terrence@tenclass.com>
 * Copy and modified from QEMU
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

#ifndef _MVISOR_DEVICES_QXL_H
#define _MVISOR_DEVICES_QXL_H

#include <cstdint>
#include "spice/start-packed.h"
#include "spice/ipc_ring.h"

typedef struct SPICE_ATTR_PACKED QXLPoint {
  int32_t x;
  int32_t y;
} QXLPoint;

typedef struct SPICE_ATTR_PACKED QXLPoint16 {
  int16_t x;
  int16_t y;
} QXLPoint16;

typedef struct SPICE_ATTR_PACKED QXLRect {
  int32_t top;
  int32_t left;
  int32_t bottom;
  int32_t right;
} QXLRect;

struct QXLURect {
  uint32_t top;
  uint32_t left;
  uint32_t bottom;
  uint32_t right;
} __attribute__((packed));

/* qxl-1 compat: append only */
struct QXLRom {
  uint32_t magic;
  uint32_t id;
  uint32_t update_id;
  uint32_t compression_level;
  uint32_t log_level;
  uint32_t mode;                    /* qxl-1 */
  uint32_t modes_offset;
  uint32_t num_pages;
  uint32_t pages_offset;            /* qxl-1 */
  uint32_t draw_area_offset;        /* qxl-1 */
  uint32_t surface0_area_size;      /* qxl-1 name: draw_area_size */
  uint32_t ram_header_offset;
  uint32_t mm_clock;
  /* appended for qxl-2 */
  uint32_t n_surfaces;
  uint64_t flags;
  uint8_t slots_start;
  uint8_t slots_end;
  uint8_t slot_gen_bits;
  uint8_t slot_id_bits;
  uint8_t slot_generation;
  /* appended for qxl-4 */
  uint8_t client_present;
  uint8_t client_capabilities[58];
  uint32_t client_monitors_config_crc;
  struct {
    uint16_t count;
    uint16_t padding;
    QXLURect heads[64];
  } client_monitors_config;
} __attribute__((packed));

#define CLIENT_MONITORS_CONFIG_CRC32_POLY 0xedb88320

/* qxl-1 compat: fixed */
struct QXLMode {
  uint32_t id;
  uint32_t x_res;
  uint32_t y_res;
  uint32_t bits;
  uint32_t stride;
  uint32_t x_mili;
  uint32_t y_mili;
  uint32_t orientation;
} __attribute__((packed));

/* qxl-1 compat: fixed */
struct QXLModes {
  uint32_t n_modes;
  QXLMode modes[0];
} __attribute__((packed));


/* qxl-1 compat: append only */
typedef enum QXLCmdType {
    QXL_CMD_NOP,
    QXL_CMD_DRAW,
    QXL_CMD_UPDATE,
    QXL_CMD_CURSOR,
    QXL_CMD_MESSAGE,
    QXL_CMD_SURFACE,
} QXLCmdType;

/* qxl-1 compat: fixed */
struct QXLCommand {
    uint64_t data;
    uint32_t type;
    uint32_t padding;
} __attribute__((packed));

#define QXL_COMMAND_FLAG_COMPAT          (1<<0)
#define QXL_COMMAND_FLAG_COMPAT_16BPP    (2<<0)

struct QXLCommandExt {
    QXLCommand cmd;
    uint32_t group_id;
    uint32_t flags;
} __attribute__((packed));

struct QXLMemSlot {
    uint64_t mem_start;
    uint64_t mem_end;
} __attribute__((packed));

#define QXL_SURF_TYPE_PRIMARY      0

#define QXL_SURF_FLAG_KEEP_DATA    (1 << 0)

struct QXLSurfaceCreate {
    uint32_t width;
    uint32_t height;
    int32_t stride;
    uint32_t format;
    uint32_t position;
    uint32_t mouse_mode;
    uint32_t flags;
    uint32_t type;
    uint64_t mem;
} __attribute__((packed));


#define QXL_COMMAND_RING_SIZE 32
#define QXL_CURSOR_RING_SIZE 32
#define QXL_RELEASE_RING_SIZE 8

SPICE_RING_DECLARE(QXLCommandRing, QXLCommand, QXL_COMMAND_RING_SIZE);
SPICE_RING_DECLARE(QXLCursorRing, QXLCommand, QXL_CURSOR_RING_SIZE);

SPICE_RING_DECLARE(QXLReleaseRing, uint64_t, QXL_RELEASE_RING_SIZE);

#define QXL_LOG_BUF_SIZE 4096

#define QXL_INTERRUPT_DISPLAY (1 << 0)
#define QXL_INTERRUPT_CURSOR (1 << 1)
#define QXL_INTERRUPT_IO_CMD (1 << 2)
#define QXL_INTERRUPT_ERROR  (1 << 3)
#define QXL_INTERRUPT_CLIENT (1 << 4)
#define QXL_INTERRUPT_CLIENT_MONITORS_CONFIG  (1 << 5)

/* qxl-1 compat: append only */
typedef struct SPICE_ATTR_ALIGNED(4) SPICE_ATTR_PACKED QXLRam {
  uint32_t magic;
  uint32_t int_pending;
  uint32_t int_mask;
  uint8_t log_buf[QXL_LOG_BUF_SIZE];
  QXLCommandRing cmd_ring;
  QXLCursorRing cursor_ring;
  QXLReleaseRing release_ring;
  QXLRect update_area;
  /* appended for qxl-2 */
  uint32_t update_surface;
  QXLMemSlot mem_slot;
  QXLSurfaceCreate create_surface;
  uint64_t flags;

  /* appended for qxl-4 */

  /* used by QXL_IO_MONITORS_CONFIG_ASYNC */
  uint64_t monitors_config;

} QXLRam;


#include "spice/end-packed.h"

#endif // _MVISOR_DEVICES_QXL_H
