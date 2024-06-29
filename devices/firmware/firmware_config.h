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

#ifndef _MVISOR_FIRMWARE_CONFIG_H
#define _MVISOR_FIRMWARE_CONFIG_H

#include "device.h"
#include <string>
#include <map>


/* selector key values for "well-known" fw_cfg entries */
#define FW_CFG_SIGNATURE 0x00
#define FW_CFG_ID  0x01
#define FW_CFG_UUID  0x02
#define FW_CFG_RAM_SIZE  0x03
#define FW_CFG_NOGRAPHIC 0x04
#define FW_CFG_NB_CPUS  0x05
#define FW_CFG_MACHINE_ID 0x06
#define FW_CFG_KERNEL_ADDR 0x07
#define FW_CFG_KERNEL_SIZE 0x08
#define FW_CFG_KERNEL_CMDLINE 0x09
#define FW_CFG_INITRD_ADDR 0x0a
#define FW_CFG_INITRD_SIZE 0x0b
#define FW_CFG_BOOT_DEVICE 0x0c
#define FW_CFG_NUMA  0x0d
#define FW_CFG_BOOT_MENU 0x0e
#define FW_CFG_MAX_CPUS  0x0f
#define FW_CFG_KERNEL_ENTRY 0x10
#define FW_CFG_KERNEL_DATA 0x11
#define FW_CFG_INITRD_DATA 0x12
#define FW_CFG_CMDLINE_ADDR 0x13
#define FW_CFG_CMDLINE_SIZE 0x14
#define FW_CFG_CMDLINE_DATA 0x15
#define FW_CFG_SETUP_ADDR 0x16
#define FW_CFG_SETUP_SIZE 0x17
#define FW_CFG_SETUP_DATA 0x18
#define FW_CFG_FILE_DIR  0x19

#define FW_CFG_FILE_FIRST 0x20
#define FW_CFG_FILE_SLOTS_MIN 0x10

#define FW_CFG_WRITE_CHANNEL 0x4000
#define FW_CFG_ARCH_LOCAL 0x8000
#define FW_CFG_ENTRY_MASK (~(FW_CFG_WRITE_CHANNEL | FW_CFG_ARCH_LOCAL))

#define FW_CFG_ACPI_TABLES      (FW_CFG_ARCH_LOCAL + 0)
#define FW_CFG_SMBIOS_ENTRIES   (FW_CFG_ARCH_LOCAL + 1)
#define FW_CFG_IRQ0_OVERRIDE    (FW_CFG_ARCH_LOCAL + 2)
#define FW_CFG_E820_TABLE       (FW_CFG_ARCH_LOCAL + 3)
#define FW_CFG_HPET             (FW_CFG_ARCH_LOCAL + 4)

#define FW_CFG_INVALID  0xffff

/* width in bytes of fw_cfg control register */
#define FW_CFG_CTL_SIZE  0x02

/* size in bytes of fw_cfg signature */
#define FW_CFG_SIG_SIZE 4

/* FW_CFG_ID bits */
#define FW_CFG_VERSION      0x01
#define FW_CFG_VERSION_DMA  0x02

/* fw_cfg "file name" is up to 56 characters (including terminating nul) */
#define FW_CFG_MAX_FILE_PATH 56

/* fw_cfg file directory entry type */
struct fw_cfg_file {
  uint32_t size;
  uint16_t select;
  uint16_t reserved;
  char name[FW_CFG_MAX_FILE_PATH];
};

#define FW_CFG_MAX_FILES 256

struct fw_cfg_files {
    uint32_t  count;
    fw_cfg_file files[FW_CFG_MAX_FILES];
};

/* FW_CFG_DMA_CONTROL bits */
#define FW_CFG_DMA_CTL_ERROR  0x01
#define FW_CFG_DMA_CTL_READ   0x02
#define FW_CFG_DMA_CTL_SKIP   0x04
#define FW_CFG_DMA_CTL_SELECT 0x08
#define FW_CFG_DMA_CTL_WRITE  0x10

#define FW_CFG_DMA_SIGNATURE    0x51454d5520434647ULL /* "QEMU CFG" */

/* Control as first field allows for different structures selected by this
 * field, which might be useful in the future
 */
struct fw_cfg_dma_access {
  uint32_t control;
  uint32_t length;
  uint64_t address;
};

#define FW_CFG_VMCOREINFO_FILENAME "etc/vmcoreinfo"

#define FW_CFG_VMCOREINFO_FORMAT_NONE 0x0
#define FW_CFG_VMCOREINFO_FORMAT_ELF  0x1

struct fw_cfg_vmcoreinfo {
  uint16_t host_format;
  uint16_t guest_format;
  uint32_t size;
  uint64_t paddr;
};

/* e820 types */
#define E820_RAM        1
#define E820_RESERVED   2

struct e820_entry {
    uint64_t address;
    uint64_t length;
    uint32_t type;
} __attribute((packed));

#define FW_CFG_IO_BASE        0x510
#define FW_CFG_DMA_IO_BASE    0x514


enum FirmwareConfigEntryType {
  kFirmwareConfigEntryTypeString,
  kFirmwareConfigEntryTypeUInt16,
  kFirmwareConfigEntryTypeUInt32,
  kFirmwareConfigEntryTypeUInt64,
  kFirmwareConfigEntryTypeBytes,
  kFirmwareConfigEntryTypeFile
};

struct FirmwareConfigEntry {
  FirmwareConfigEntryType type;
  std::string file;
  std::string bytes;
};



#endif // _MVISOR_FIRMWARE_CONFIG_H
