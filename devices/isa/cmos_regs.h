/* 
 * MVisor
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

#ifndef _MVISOR_DEVICES_CMOS_H
#define _MVISOR_DEVICES_CMOS_H

#define RTC_BASE_ADDRESS 0x70

/*
 * MC146818 RTC registers
 * https://bochs.sourceforge.io/techspec/CMOS-reference.txt
 * https://wiki.osdev.org/CMOS
 */
#define RTC_SECONDS       0x00
#define RTC_SECONDS_ALARM 0x01
#define RTC_MINUTES       0x02
#define RTC_MINUTES_ALARM 0x03
#define RTC_HOURS         0x04
#define RTC_HOURS_ALARM   0x05
#define RTC_DAY_OF_WEEK   0x06
#define RTC_DAY_OF_MONTH  0x07
#define RTC_MONTH         0x08
#define RTC_YEAR          0x09
#define RTC_CENTURY       0x32

#define RTC_IRQ           8

#define RTC_REG_A   0x0A
#define RTC_REG_B   0x0B
#define RTC_REG_C   0x0C
#define RTC_REG_D   0x0D

#define REG_A_UIP   0x80 // Update in progress (0 = Date and time can be read, 1 = Time update in progress)

#define REG_B_SET   0x80 // Clock cycle update (0 = Update normally, 1 = Abort update in progress)
#define REG_B_PIE   0x40 // Periodic interrupt (0 = Disable interrupt (default), 1 = Enable interrupt)
#define REG_B_AIE   0x20 // Alarm interrupt (0 = Disable interrupt (default), 1 = Enable interrupt)
#define REG_B_UIE   0x10 // Update ended interrupt (0 = Disable interrupt (default), 1 = Enable interrupt)
#define REG_B_SQWE  0x08 // Status register A square wave frequency (0 = Disable square wave (default), 1 = Enable square wave)
#define REG_B_DM    0x04 // Data mode (0 = BCD, 1: Binary)
#define REG_B_24H   0x02 // 24 hour clock (0 = 24 hour mode (default), 1 = 12 hour mode)
#define REG_B_DSE   0x01 // Daylight savings enable (0 = Disable, 1 = Enable)

#define REG_C_IRQF  0x80 // Interrupt flag (1 when any or all of bits 6-4 are 1 and appropriate enables)
#define REG_C_PF    0x40 // Periodic interrupt flag
#define REG_C_AF    0x20 // Alarm interrupt flag
#define REG_C_UF    0x10 // Update ended interrupt flag
#define REG_C_MASK  0x70

#define REG_D_VALID_RAM 0x80 // Valid RAM - 1 indicates batery power good, 0 if dead or disconnected.

#endif // _MVISOR_DEVICES_CMOS_H
