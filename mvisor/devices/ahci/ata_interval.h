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

#ifndef _MVISOR_DEVICE_ATA_INTERNAL_H
#define _MVISOR_DEVICE_ATA_INTERNAL_H

// The Command/Status Port returns a bit mask referring to the status of a channel when read.
#define ATA_SR_BSY     0x80    // Busy
#define ATA_SR_DRDY    0x40    // Drive ready
#define ATA_SR_DF      0x20    // Drive write fault
#define ATA_SR_DSC     0x10    // Drive seek complete
#define ATA_SR_DRQ     0x08    // Data request ready
#define ATA_SR_CORR    0x04    // Corrected data
#define ATA_SR_IDX     0x02    // Index
#define ATA_SR_ERR     0x01    // Error


#define ATA_CB_ER_ICRC 0x80    // ATA Ultra DMA bad CRC
#define ATA_CB_ER_BBK  0x80    // ATA bad block
#define ATA_CB_ER_UNC  0x40    // ATA uncorrected error
#define ATA_CB_ER_MC   0x20    // ATA media change
#define ATA_CB_ER_IDNF 0x10    // ATA id not found
#define ATA_CB_ER_MCR  0x08    // ATA media change request
#define ATA_CB_ER_ABRT 0x04    // ATA command aborted
#define ATA_CB_ER_NTK0 0x02    // ATA track 0 not found
#define ATA_CB_ER_NDAM 0x01    // ATA address mark not found

#define ATA_CB_ER_P_SNSKEY 0xf0   // ATAPI sense key (mask)
#define ATA_CB_ER_P_MCR    0x08   // ATAPI Media Change Request
#define ATA_CB_ER_P_ABRT   0x04   // ATAPI command abort
#define ATA_CB_ER_P_EOM    0x02   // ATAPI End of Media
#define ATA_CB_ER_P_ILI    0x01   // ATAPI Illegal Length Indication

// ATA_CMD_IDENTIFY_PACKET and ATA_CMD_IDENTIFY return a buffer of 512 bytes called the identification space;
// the following definitions are used to read information from the identification space.

#define ATA_IDENT_DEVICETYPE   0
#define ATA_IDENT_CYLINDERS    2
#define ATA_IDENT_HEADS        6
#define ATA_IDENT_SECTORS      12
#define ATA_IDENT_SERIAL       20
#define ATA_IDENT_MODEL        54
#define ATA_IDENT_CAPABILITIES 98
#define ATA_IDENT_FIELDVALID   106
#define ATA_IDENT_MAX_LBA      120
#define ATA_IDENT_COMMANDSETS  164
#define ATA_IDENT_MAX_LBA_EXT  200

// When you select a drive, you should specify the interface type and whether it is the master or slave:

#define IDE_ATA        0x00
#define IDE_ATAPI      0x01
 
#define ATA_MASTER     0x00
#define ATA_SLAVE      0x01

// Task File is a range of 8 ports which are offsets from BAR0 (primary channel) and/or
// BAR2 (secondary channel). To exemplify:

// BAR0 + 0 is first port.
// BAR0 + 1 is second port.
// BAR0 + 2 is the third

#define ATA_REG_DATA       0x00
#define ATA_REG_ERROR      0x01
#define ATA_REG_FEATURES   0x01
#define ATA_REG_SECCOUNT0  0x02
#define ATA_REG_LBA0       0x03
#define ATA_REG_LBA1       0x04
#define ATA_REG_LBA2       0x05
#define ATA_REG_HDDEVSEL   0x06
#define ATA_REG_COMMAND    0x07
#define ATA_REG_STATUS     0x07
#define ATA_REG_SECCOUNT1  0x08
#define ATA_REG_LBA3       0x09
#define ATA_REG_LBA4       0x0A
#define ATA_REG_LBA5       0x0B

#define ATA_REG_CONTROL    0x02
#define ATA_REG_ALTSTATUS  0x02
#define ATA_REG_DEVADDRESS 0x03

// FROM SeaBIOS
#define PORT_ATA2_CMD_BASE     0x0170
#define PORT_ATA1_CMD_BASE     0x01f0
#define PORT_ATA2_CTRL_BASE    0x0374
#define PORT_ATA1_CTRL_BASE    0x03f4

// Global defines -- ATA register and register bits.
// command block & control block regs
#define ATA_CB_DATA  0   // data reg         in/out pio_base_addr1+0
#define ATA_CB_ERR   1   // error            in     pio_base_addr1+1
#define ATA_CB_FR    1   // feature reg         out pio_base_addr1+1
#define ATA_CB_SC    2   // sector count     in/out pio_base_addr1+2
#define ATA_CB_SN    3   // sector number    in/out pio_base_addr1+3
#define ATA_CB_CL    4   // cylinder low     in/out pio_base_addr1+4
#define ATA_CB_CH    5   // cylinder high    in/out pio_base_addr1+5
#define ATA_CB_DH    6   // device head      in/out pio_base_addr1+6
#define ATA_CB_STAT  7   // primary status   in     pio_base_addr1+7
#define ATA_CB_CMD   7   // command             out pio_base_addr1+7

#define ATA_CB_ASTAT 2   // alternate status in     pio_base_addr2+2
#define ATA_CB_DC    2   // device control      out pio_base_addr2+2
#define ATA_CB_DA    3   // device address   in     pio_base_addr2+3

// ATAPI Interrupt Reason bits in the Sector Count reg (CB_SC)
#define ATA_CB_SC_P_TAG    0xf8   // ATAPI tag (mask)
#define ATA_CB_SC_P_REL    0x04   // ATAPI release
#define ATA_CB_SC_P_IO     0x02   // ATAPI I/O
#define ATA_CB_SC_P_CD     0x01   // ATAPI C/D

// bits 7-4 of the device/head (CB_DH) reg
#define ATA_CB_DH_DEV0 0xa0    // select device 0
#define ATA_CB_DH_DEV1 0xb0    // select device 1
#define ATA_CB_DH_LBA 0x40    // use LBA

// status reg (CB_STAT and CB_ASTAT) bits
#define ATA_CB_STAT_BSY  0x80  // busy
#define ATA_CB_STAT_RDY  0x40  // ready
#define ATA_CB_STAT_DF   0x20  // device fault
#define ATA_CB_STAT_WFT  0x20  // write fault (old name)
#define ATA_CB_STAT_SKC  0x10  // seek complete
#define ATA_CB_STAT_SERV 0x10  // service
#define ATA_CB_STAT_DRQ  0x08  // data request
#define ATA_CB_STAT_CORR 0x04  // corrected
#define ATA_CB_STAT_IDX  0x02  // index
#define ATA_CB_STAT_ERR  0x01  // error (ATA)
#define ATA_CB_STAT_CHK  0x01  // check (ATAPI)

// device control reg (CB_DC) bits
#define ATA_CB_DC_HD15   0x08  // bit should always be set to one
#define ATA_CB_DC_SRST   0x04  // soft reset
#define ATA_CB_DC_NIEN   0x02  // disable interrupts

// Most mandtory and optional ATA commands (from ATA-3),
#define ATA_CMD_NOP                          0x00
#define ATA_CMD_CFA_REQUEST_EXT_ERR_CODE     0x03
#define ATA_CMD_DEVICE_RESET                 0x08
#define ATA_CMD_RECALIBRATE                  0x10
#define ATA_CMD_READ_SECTORS                 0x20
#define ATA_CMD_READ_SECTORS_EXT             0x24
#define ATA_CMD_READ_DMA_EXT                 0x25
#define ATA_CMD_READ_DMA_QUEUED_EXT          0x26
#define ATA_CMD_READ_NATIVE_MAX_ADDRESS_EXT  0x27
#define ATA_CMD_READ_MULTIPLE_EXT            0x29
#define ATA_CMD_READ_LOG_EXT                 0x2F
#define ATA_CMD_WRITE_SECTORS                0x30
#define ATA_CMD_WRITE_SECTORS_EXT            0x34
#define ATA_CMD_WRITE_DMA_EXT                0x35
#define ATA_CMD_WRITE_DMA_QUEUED_EXT         0x36
#define ATA_CMD_SET_MAX_ADDRESS_EXT          0x37
#define ATA_CMD_CFA_WRITE_SECTORS_WO_ERASE   0x38
#define ATA_CMD_WRITE_MULTIPLE_EXT           0x39
#define ATA_CMD_WRITE_VERIFY                 0x3C
#define ATA_CMD_WRITE_LOG_EXT                0x3F
#define ATA_CMD_READ_VERIFY_SECTORS          0x40
#define ATA_CMD_READ_VERIFY_SECTORS_EXT      0x42
#define ATA_CMD_FORMAT_TRACK                 0x50
#define ATA_CMD_SEEK                         0x70
#define ATA_CMD_CFA_TRANSLATE_SECTOR         0x87
#define ATA_CMD_EXECUTE_DEVICE_DIAGNOSTIC    0x90
#define ATA_CMD_INITIALIZE_DEVICE_PARAMETERS 0x91
#define ATA_CMD_STANDBY_IMMEDIATE2           0x94
#define ATA_CMD_IDLE_IMMEDIATE2              0x95
#define ATA_CMD_STANDBY2                     0x96
#define ATA_CMD_IDLE2                        0x97
#define ATA_CMD_CHECK_POWER_MODE2            0x98
#define ATA_CMD_SLEEP2                       0x99
#define ATA_CMD_PACKET                       0xA0
#define ATA_CMD_IDENTIFY_PACKET_DEVICE       0xA1
#define ATA_CMD_CFA_ERASE_SECTORS            0xC0
#define ATA_CMD_READ_MULTIPLE                0xC4
#define ATA_CMD_WRITE_MULTIPLE               0xC5
#define ATA_CMD_SET_MULTIPLE_MODE            0xC6
#define ATA_CMD_READ_DMA_QUEUED              0xC7
#define ATA_CMD_READ_DMA                     0xC8
#define ATA_CMD_WRITE_DMA                    0xCA
#define ATA_CMD_WRITE_DMA_QUEUED             0xCC
#define ATA_CMD_CFA_WRITE_MULTIPLE_WO_ERASE  0xCD
#define ATA_CMD_STANDBY_IMMEDIATE            0xE0
#define ATA_CMD_IDLE_IMMEDIATE               0xE1
#define ATA_CMD_STANDBY                      0xE2
#define ATA_CMD_IDLE                         0xE3
#define ATA_CMD_READ_BUFFER                  0xE4
#define ATA_CMD_CHECK_POWER_MODE             0xE5
#define ATA_CMD_SLEEP                        0xE6
#define ATA_CMD_FLUSH_CACHE                  0xE7
#define ATA_CMD_WRITE_BUFFER                 0xE8
#define ATA_CMD_IDENTIFY_DEVICE              0xEC
#define ATA_CMD_SET_FEATURES                 0xEF
#define ATA_CMD_READ_NATIVE_MAX_ADDRESS      0xF8
#define ATA_CMD_SET_MAX                      0xF9

#define ATA_SET_FEATRUE_TRANSFER_MODE        0x03
#define ATA_TRANSFER_MODE_ULTRA_DMA          0x40
#define ATA_TRANSFER_MODE_MULTIWORD_DMA      0x20
#define ATA_TRANSFER_MODE_PIO_FLOW_CTRL      0x08
#define ATA_TRANSFER_MODE_DEFAULT_PIO        0x00

inline static const char* __get_register_name(size_t index) {
  static const char* names[] = {
    "ATA_REG_DATA",
    "ATA_REG_ERROR/ATA_REG_FEATURES",
    "ATA_REG_SECCOUNT0",
    "ATA_REG_LBA0",
    "ATA_REG_LBA1",
    "ATA_REG_LBA2",
    "ATA_REG_HDDEVSEL",
    "ATA_REG_COMMAND",
    "ATA_REG_STATUS",
    "ATA_REG_SECCOUNT1",
    "ATA_REG_LBA3",
    "ATA_REG_LBA4",
    "ATA_REG_LBA5",
    "ATA_REG_CONTROL",
    "ATA_REG_ALTSTATUS",
    "ATA_REG_DEVADDRESS"
  };
  size_t max_index = (sizeof(names) / sizeof(names[0]));
  return index < max_index ? names[index] : "INVALID";
}

// Channels:
#define      ATA_PRIMARY      0x00
#define      ATA_SECONDARY    0x01
 
// Directions:
#define      ATA_READ      0x00
#define      ATA_WRITE     0x01


// SCSI

struct CBD_RW_DATA10 {
    uint8_t command;
    uint8_t flags;
    uint32_t lba;
    uint8_t resreved_06;
    uint16_t count;
    uint8_t reserved_09;
    uint8_t pad[6];
} __attribute__((packed));


struct CBD_READ_CAPACITY {
    uint8_t command;
    uint8_t flags;
    uint8_t resreved_02[8];
    uint8_t pad[6];
} __attribute__((packed));

struct CBD_RES_READ_CAPACITY {
    uint32_t sectors;
    uint32_t blksize;
} __attribute__((packed));


struct CBD_REQUEST_SENSE {
    uint8_t command;
    uint8_t flags;
    uint16_t reserved_02;
    uint8_t length;
    uint8_t reserved_05;
    uint8_t pad[10];
} __attribute__((packed));

struct CBD_RES_REQUEST_SENSE {
    uint8_t errcode;
    uint8_t segment;
    uint8_t flags;
    uint32_t info;
    uint8_t additional;
    uint32_t specific;
    uint8_t asc;
    uint8_t ascq;
    uint32_t reserved_0e;
} __attribute__((packed));

#define SCSI_TYPE_DISK  0x00
#define SCSI_TYPE_CDROM 0x05

struct CBD_RES_INQUERY {
    uint8_t pdt;
    uint8_t removable;
    uint8_t reserved_02[2];
    uint8_t additional;
    uint8_t reserved_05[3];
    char vendor[8];
    char product[16];
    char rev[4];
} __attribute__((packed));

#define MODE_PAGE_HD_GEOMETRY 0x04

struct CBD_MODE_SENSE {
    uint8_t command;
    uint8_t flags;
    uint8_t page;
    uint32_t reserved_03;
    uint16_t count;
    uint8_t reserved_09;
    uint8_t pad[6];
} __attribute__((packed));

struct CBD_RES_MODE_SENSE_GEOM {
    uint8_t unused_00[3];
    uint8_t read_only;
    uint32_t unused_04;
    uint8_t page;
    uint8_t length;
    uint8_t cyl[3];
    uint8_t heads;
    uint8_t precomp[3];
    uint8_t reduced[3];
    uint16_t step_rate;
    uint8_t landing[3];
    uint16_t rpm;
} __attribute__((packed));

/* The generic packet command opcodes for CD/DVD Logical Units,
 * From Table 57 of the SFF8090 Ver. 3 (Mt. Fuji) draft standard. */
#define GPCMD_BLANK			    0xa1
#define GPCMD_CLOSE_TRACK		    0x5b
#define GPCMD_FLUSH_CACHE		    0x35
#define GPCMD_FORMAT_UNIT		    0x04
#define GPCMD_GET_CONFIGURATION		    0x46
#define GPCMD_GET_EVENT_STATUS_NOTIFICATION 0x4a
#define GPCMD_GET_PERFORMANCE		    0xac
#define GPCMD_INQUIRY			    0x12
#define GPCMD_LOAD_UNLOAD		    0xa6
#define GPCMD_MECHANISM_STATUS		    0xbd
#define GPCMD_MODE_SELECT_10		    0x55
#define GPCMD_MODE_SENSE_10		    0x5a
#define GPCMD_PAUSE_RESUME		    0x4b
#define GPCMD_PLAY_AUDIO_10		    0x45
#define GPCMD_PLAY_AUDIO_MSF		    0x47
#define GPCMD_PLAY_AUDIO_TI		    0x48
#define GPCMD_PLAY_CD			    0xbc
#define GPCMD_PREVENT_ALLOW_MEDIUM_REMOVAL  0x1e
#define GPCMD_READ_10			    0x28
#define GPCMD_READ_12			    0xa8
#define GPCMD_READ_CDVD_CAPACITY	    0x25
#define GPCMD_READ_CD			    0xbe
#define GPCMD_READ_CD_MSF		    0xb9
#define GPCMD_READ_DISC_INFO		    0x51
#define GPCMD_READ_DVD_STRUCTURE	    0xad
#define GPCMD_READ_FORMAT_CAPACITIES	    0x23
#define GPCMD_READ_HEADER		    0x44
#define GPCMD_READ_TRACK_RZONE_INFO	    0x52
#define GPCMD_READ_SUBCHANNEL		    0x42
#define GPCMD_READ_TOC_PMA_ATIP		    0x43
#define GPCMD_REPAIR_RZONE_TRACK	    0x58
#define GPCMD_REPORT_KEY		    0xa4
#define GPCMD_REQUEST_SENSE		    0x03
#define GPCMD_RESERVE_RZONE_TRACK	    0x53
#define GPCMD_SCAN			    0xba
#define GPCMD_SEEK			    0x2b
#define GPCMD_SEND_DVD_STRUCTURE	    0xad
#define GPCMD_SEND_EVENT		    0xa2
#define GPCMD_SEND_KEY			    0xa3
#define GPCMD_SEND_OPC			    0x54
#define GPCMD_SET_READ_AHEAD		    0xa7
#define GPCMD_SET_STREAMING		    0xb6
#define GPCMD_START_STOP_UNIT		    0x1b
#define GPCMD_STOP_PLAY_SCAN		    0x4e
#define GPCMD_TEST_UNIT_READY		    0x00
#define GPCMD_VERIFY_10			    0x2f
#define GPCMD_WRITE_10			    0x2a
#define GPCMD_WRITE_AND_VERIFY_10	    0x2e

/* This is listed as optional in ATAPI 2.6, but is (curiously)
 * missing from Mt. Fuji, Table 57.  It _is_ mentioned in Mt. Fuji
 * Table 377 as an MMC command for SCSi devices though...  Most ATAPI
 * drives support it. */
#define GPCMD_SET_SPEED			    0xbb
/* This seems to be a SCSI specific CD-ROM opcode
 * to play data at track/index */
#define GPCMD_PLAYAUDIO_TI		    0x48
/*
 * From MS Media Status Notification Support Specification. For
 * older drives only.
 */
#define GPCMD_GET_MEDIA_STATUS		    0xda
#define GPCMD_MODE_SENSE_6		    0x1a


#endif // _MVISOR_DEVICE_ATA_INTERNAL_H
