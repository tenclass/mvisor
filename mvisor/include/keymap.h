/* 
 * MVisor
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

#ifndef _MVSIOR_KEY_MAP_H
#define _MVSIOR_KEY_MAP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
int TranslateScancode(uint8_t scancode, int pressed, uint8_t transcoded_size[10]);
#ifdef __cplusplus
}
#endif

#endif // _MVSIOR_KEY_MAP_H
 