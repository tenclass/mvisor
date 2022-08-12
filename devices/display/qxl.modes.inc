/* 
 * MVisor QXL Modes
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
#ifndef _MVISOR_DEVICES_QXL_MODES_H
#define _MVISOR_DEVICES_QXL_MODES_H

#include "spice/qxl_dev.h"

#define PIXEL_SIZE 0.2936875 // 1280x1024 is 14.8" x 11.9" 

#define QXL_MODE(_x, _y, _b, _o)                  \
    {   .x_res = _x,                              \
        .y_res = _y,                              \
        .bits  = _b,                              \
        .stride = (_x) * (_b) / 8,                \
        .x_mili = (uint32_t)(PIXEL_SIZE * (_x)),  \
        .y_mili = (uint32_t)(PIXEL_SIZE * (_y)),  \
        .orientation = _o,                        \
    }

#define QXL_MODE_16_32(x_res, y_res, orientation) \
    QXL_MODE(x_res, y_res, 16, orientation),      \
    QXL_MODE(x_res, y_res, 32, orientation)

#define QXL_MODE_EX(x_res, y_res)                 \
    QXL_MODE_16_32(x_res, y_res, 0),              \
    QXL_MODE_16_32(x_res, y_res, 1)

static QXLMode qxl_modes[] = {
    QXL_MODE_EX(640, 480),
    QXL_MODE_EX(800, 480),
    QXL_MODE_EX(800, 600),
    QXL_MODE_EX(832, 624),
    QXL_MODE_EX(960, 640),
    QXL_MODE_EX(1024, 600),
    QXL_MODE_EX(1024, 768),
    QXL_MODE_EX(1152, 864),
    QXL_MODE_EX(1152, 870),
    QXL_MODE_EX(1280, 720),
    QXL_MODE_EX(1280, 760),
    QXL_MODE_EX(1280, 768),
    QXL_MODE_EX(1280, 800),
    QXL_MODE_EX(1280, 960),
    QXL_MODE_EX(1280, 1024),
    QXL_MODE_EX(1360, 768),
    QXL_MODE_EX(1366, 768),
    QXL_MODE_EX(1400, 1050),
    QXL_MODE_EX(1440, 900),
    QXL_MODE_EX(1600, 900),
    QXL_MODE_EX(1600, 1200),
    QXL_MODE_EX(1680, 1050),
    QXL_MODE_EX(1920, 1080),
    /* these modes need more than 8 MB video memory */
    QXL_MODE_EX(1920, 1200),
    QXL_MODE_EX(1920, 1440),
    QXL_MODE_EX(2000, 2000),
    QXL_MODE_EX(2048, 1536),
    QXL_MODE_EX(2048, 2048),
    QXL_MODE_EX(2560, 1440),
    QXL_MODE_EX(2560, 1600),
    /* these modes need more than 16 MB video memory */
    QXL_MODE_EX(2560, 2048),
    QXL_MODE_EX(2800, 2100),
    QXL_MODE_EX(3200, 2400),
    /* these modes need more than 32 MB video memory */
    QXL_MODE_EX(3840, 2160), /* 4k mainstream */
    QXL_MODE_EX(4096, 2160), /* 4k            */
    /* these modes need more than 64 MB video memory */
    QXL_MODE_EX(7680, 4320), /* 8k mainstream */
    /* these modes need more than 128 MB video memory */
    QXL_MODE_EX(8192, 4320), /* 8k            */
};

#endif // _MVISOR_DEVICES_QXL_MODES_H
