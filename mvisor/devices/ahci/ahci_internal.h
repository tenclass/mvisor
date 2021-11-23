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

#ifndef _MVISOR_DEVICES_AHCI_INTERNAL_H
#define _MVISOR_DEVICES_AHCI_INTERNAL_H

#include <stdint.h>

#define AHCI_MEM_BAR_SIZE         0x1000
#define AHCI_MAX_PORTS            32
#define AHCI_MAX_SG               168 /* hardware max is 64K */
#define AHCI_DMA_BOUNDARY         0xffffffff
#define AHCI_USE_CLUSTERING       0
#define AHCI_MAX_CMDS             32
#define AHCI_CMD_SZ               32
#define AHCI_CMD_SLOT_SZ          (AHCI_MAX_CMDS * AHCI_CMD_SZ)
#define AHCI_RX_FIS_SZ            256
#define AHCI_CMD_TBL_CDB          0x40
#define AHCI_CMD_TBL_HDR_SZ       0x80
#define AHCI_CMD_TBL_SZ           (AHCI_CMD_TBL_HDR_SZ + (AHCI_MAX_SG * 16))
#define AHCI_CMD_TBL_AR_SZ        (AHCI_CMD_TBL_SZ * AHCI_MAX_CMDS)
#define AHCI_PORT_PRIV_DMA_SZ     (AHCI_CMD_SLOT_SZ + AHCI_CMD_TBL_AR_SZ + \
                                   AHCI_RX_FIS_SZ)

#define AHCI_IRQ_ON_SG            (1U << 31)
#define AHCI_CMD_ATAPI            (1 << 5)
#define AHCI_CMD_WRITE            (1 << 6)
#define AHCI_CMD_PREFETCH         (1 << 7)
#define AHCI_CMD_RESET            (1 << 8)
#define AHCI_CMD_CLR_BUSY         (1 << 10)

#define RX_FIS_DMA_SETUP          0x00
#define RX_FIS_PIO_SETUP          0x20
#define RX_FIS_REG_D2H            0x40 /* offset of D2H Register FIS data */
#define RX_FIS_SET_DEVICE_BITS    0x58 /* offset of SDB FIS data */
#define RX_FIS_UNKNOWN            0x60 /* offset of Unknown FIS data */

/* global controller registers */
enum AhciHostReg {
  kAhciHostRegCapabilities = 0,  /* CAP: host capabilities */
  kAhciHostRegControl      = 1,  /* GHC: global host control */
  kAhciHostRegIrqStatus    = 2,  /* IS: interrupt status */
  kAhciHostRegPortsImplemented = 3,  /* PI: bitmap of implemented ports */
  kAhciHostRegVersion      = 4,  /* VS: AHCI spec. version compliancy */
  kAhciHostRegCccControl   = 5,  /* CCC_CTL: CCC Control */
  kAhciHostRegCccPorts     = 6,  /* CCC_PORTS: CCC Ports */
  kAhciHostRegEmLocation   = 7,  /* EM_LOC: Enclosure Mgmt Location */
  kAhciHostRegEmControl    = 8,  /* EM_CTL: Enclosure Mgmt Control */
  kAhciHostRegCapabilities2= 9,  /* CAP2: host capabilities, extended */
  kAhciHostRegBoch         = 10, /* BOHC: firmare/os handoff ctrl & status */
  kAhciHostRegCount     = 11
};

/* HOST_CONTROL bits */
#define HOST_CONTROL_RESET                (1 << 0)   /* reset controller; self-clear */
#define HOST_CONTROL_IRQ_ENABLE           (1 << 1)   /* global IRQ enable */
#define HOST_CONTROL_AHCI_ENABLE          (1U << 31) /* AHCI enabled */

/* HOST_CAP bits */
#define HOST_CAP_SLUMBER_CAPABLE         (1 << 14) /* Slumber capable */
#define HOST_CAP_AHCI                    (1 << 18) /* AHCI only */
#define HOST_CAP_COMMAND_LIST_OVERRIDE   (1 << 24) /* Command List Override support */
#define HOST_CAP_STAGGERED_SPIN_UP       (1 << 27) /* Staggered Spin-up */
#define HOST_CAP_NCQ                     (1 << 30) /* Native Command Queueing */
#define HOST_CAP_64                      (1U << 31) /* PCI DAC (64-bit DMA) support */

/* registers for each SATA port */
enum AhciPortReg {
  kAhciPortRegCommandListBase0 = 0, /* PxCLB: command list DMA addr */
  kAhciPortRegCommandListBase1 = 1, /* PxCLBU: command list DMA addr hi */
  kAhciPortRegReceivedFisBase0 = 2, /* PxFB: FIS rx buf addr */
  kAhciPortRegReceivedFisBase1 = 3, /* PxFBU: FIX rx buf addr hi */
  kAhciPortRegIrqStatus        = 4, /* PxIS: interrupt status */
  kAhciPortRegIrqMask          = 5, /* PxIE: interrupt enable/disable mask */
  kAhciPortRegCommand          = 6, /* PxCMD: port command */
  /* RESERVED */
  kAhciPortRegTaskFileData     = 8, /* PxTFD: taskfile data */
  kAhciPortRegSignature        = 9, /* PxSIG: device TF signature */
  kAhciPortRegSataStatus       = 10, /* PxSSTS: SATA phy register: SStatus */
  kAhciPortRegSataControl      = 11, /* PxSCTL: SATA phy register: SControl */
  kAhciPortRegSataError        = 12, /* PxSERR: SATA phy register: SError */
  kAhciPortRegSataActive       = 13, /* PxSACT: SATA phy register: SActive */
  kAhciPortRegCommandIssue     = 14, /* PxCI: command issue */
  kAhciPortRegSataNotification = 15, /* PxSNTF: SATA phy register: SNotification */
  kAhciPortRegFisControl       = 16, /* PxFBS: Port multiplier switching ctl */
  kAhciPortRegDeviceSleepControl = 17, /* PxDEVSLP: device sleep control */
  /* RESERVED */
  kAhciPortRegVendor1    = 28, /* PxVS: Vendor Specific */
  kAhciPortRegVendor2    = 29,
  kAhciPortRegVendor3    = 30,
  kAhciPortRegVendor4    = 31,
  kAhciPortRegCount      = 32
};

/* Port interrupt bit descriptors */
enum AhciPortIrq {
  kAhciPortIrqBitDeviceToHostFis = 0,
  kAhciPortIrqBitPioSetupFis     = 1,
  kAhciPortIrqBitDmaSetupFis     = 2,
  kAhciPortIrqBitSetDeviceBitsFis = 3,
  kAhciPortIrqBitUnknownFis      = 4,
  kAhciPortIrqBitDescriptorProcessed = 5,
  kAhciPortIrqBitPortConnectChanged = 6,
  kAhciPortIrqBitDeviceMechanicalPresense = 7,
  /* RESERVED */
  kAhciPortIrqBitPhyscalReadyChange = 22,
  kAhciPortIrqBitIncorrectPortMultiplier = 23,
  kAhciPortIrqBitOverflow  = 24,
  /* RESERVED */
  kAhciPortIrqBitInterfaceNonFatalError = 26,
  kAhciPortIrqBitInterfaceFatalError = 27,
  kAhciPortIrqBitHostBusDataError = 28,
  kAhciPortIrqBitHostBusFatalError = 29,
  kAhciPortIrqBitTaskFileError = 30,
  kAhciPortIrqBitColdPortDetect = 31,
  kAhciPortIrqCount = 32
};

/* PORT_CMD bits */
#define PORT_CMD_ATAPI            (1 << 24) /* Device is ATAPI */
#define PORT_CMD_LIST_ON          (1 << 15) /* cmd list DMA engine running */
#define PORT_CMD_FIS_ON           (1 << 14) /* FIS DMA engine running */
#define PORT_CMD_FIS_RX           (1 << 4)  /* Enable FIS receive DMA engine */
#define PORT_CMD_CLO              (1 << 3)  /* Command list override */
#define PORT_CMD_POWER_ON         (1 << 2)  /* Power up device */
#define PORT_CMD_SPIN_UP          (1 << 1)  /* Spin up device */
#define PORT_CMD_START            (1 << 0)  /* Enable port DMA engine */

#define PORT_CMD_ICC_MASK         (0xfU << 28) /* i/f ICC state mask */
#define PORT_CMD_ICC_ACTIVE       (0x1 << 28)  /* Put i/f in active state */
#define PORT_CMD_ICC_PARTIAL      (0x2 << 28)  /* Put i/f in partial state */
#define PORT_CMD_ICC_SLUMBER      (0x6 << 28)  /* Put i/f in slumber state */

#define PORT_CMD_RO_MASK          0x007dffe0   /* Which CMD bits are read only? */

/* ap->flags bits */
#define AHCI_FLAG_NO_NCQ                  (1 << 24)
#define AHCI_FLAG_IGNORE_IRQ_IF_ERR       (1 << 25) /* ignore IRQ_IF_ERR */
#define AHCI_FLAG_HONOR_PORTS_IMPL        (1 << 26) /* honor PORTS_IMPL */
#define AHCI_FLAG_IGNORE_SERR_INTERNAL    (1 << 27) /* ignore SERR_INTERNAL */
#define AHCI_FLAG_32BIT_ONLY              (1 << 28) /* force 32bit */

#define AHCI_SCR_SCTL_DET                 0xf

#define AHCI_CMD_HDR_CMD_FIS_LEN           0x1f
#define AHCI_CMD_HDR_PRDT_LEN              16

#define AHCI_GENERIC_HOST_CONTROL_REGS_MAX_ADDR 0x2c

#define AHCI_NUM_COMMAND_SLOTS             31
#define AHCI_SUPPORTED_SPEED               20
#define AHCI_SUPPORTED_SPEED_GEN1          1
#define AHCI_VERSION_1_0                   0x10000

#define AHCI_PROGMODE_MAJOR_REV_1          1

#define AHCI_PRDT_SIZE_MASK                0x3fffff

#define NCQ_FIS_FUA_MASK                   0x80
#define NCQ_FIS_RARC_MASK                  0x01

#define ATA_CAP_SIZE           0x8
#define ATA_CAP_REV            0x2
#define ATA_CAP_BAR            0x4

#define IDE_FEATURE_DMA                    1
#define ATA_SOFT_RESET                    (1 << 2)  /* software reset */

#define ATA_SCR_SSTATUS_DET_NODEV        0x0
#define ATA_SCR_SSTATUS_DET_DEV_PRESENT_PHY_UP 0x3

#define ATA_SCR_SSTATUS_SPD_NODEV        0x00
#define ATA_SCR_SSTATUS_SPD_GEN1         0x10

#define ATA_SCR_SSTATUS_IPM_NODEV        0x000
#define ATA_SCR_SSTATUS_IPM_ACTIVE       0X100

#define ATA_SIGNATURE_CDROM               0xeb140101
#define ATA_SIGNATURE_DISK                0x00000101


struct AhciPrdtEntry {
  uint64_t    address;
  uint32_t    reserved0;
  uint32_t    size : 22;    // 4MB max
  uint32_t    reserved1 : 9;
  uint32_t    interrupt : 1; // interrupt on completion
} __attribute__((packed));

struct AhciCommandTable {
  // 0x00
  uint8_t       command_fis[64];
  // 0x40
  uint8_t       atapi_command[16];
  // 0x50
  uint8_t       reserved[48];
  // 0x80
  AhciPrdtEntry prdt_entries[1];
} __attribute__((packed));


struct AhciSetDeviceBitsFis {
  uint8_t type;
  uint8_t flags;
  uint8_t status;
  uint8_t error;
  uint32_t payload;
} __attribute__((packed));


enum AhciFisType
{
  kAhciFisTypeRegH2D = 0x27, // Register FIS - host to device
  kAhciFisTypeRegD2H = 0x34, // Register FIS - device to host
  kAhciFisTypeDmaActivate = 0x39, // DMA activate FIS - device to host
  kAhciFisTypeDmaSetup = 0x41, // DMA setup FIS - bidirectional
  kAhciFisTypeData = 0x46, // Data FIS - bidirectional
  kAhciFisTypeSelfTest = 0x58, // Built-in Self Test activate FIS - bidirectional
  kAhciFisTypePioSetup = 0x5F, // PIO setup FIS - device to host
  kAhciFisTypeDeviceBits = 0xA1, // Set device bits FIS - device to host
};

struct AhciFisRegH2D
{
  // DWORD 0
  uint8_t  fis_type;             // kAhciFisTypeRegH2D
 
  uint8_t  port_multiplier : 4;  // Port multiplier
  uint8_t  reserved0 : 3;        // Reserved
  uint8_t  is_command : 1;       // 1: Command, 0: Control
 
  uint8_t  command;              // Command register
  uint8_t  feature0;             // Feature register, 7:0
 
  // DWORD 1
  uint8_t  lba0;                 // LBA low register, 7:0
  uint8_t  lba1;                 // LBA mid register, 15:8
  uint8_t  lba2;                 // LBA high register, 23:16
  uint8_t  device;               // Device register
 
  // DWORD 2
  uint8_t  lba3;                 // LBA register, 31:24
  uint8_t  lba4;                 // LBA register, 39:32
  uint8_t  lba5;                 // LBA register, 47:40
  uint8_t  feature1;             // Feature register, 15:8
 
  // DWORD 3
  uint8_t  count0;               // Count register, 7:0
  uint8_t  count1;               // Count register, 15:8
  uint8_t  icc;                  // Isochronous command completion
  uint8_t  control;              // Control register
 
  // DWORD 4
  uint8_t  reserved1[4];         // Reserved
} __attribute__((packed));

struct AhciFisRegD2H
{
  // DWORD 0
  uint8_t  fis_type;             // kAhciFisTypeRegD2H
 
  uint8_t  port_multiplier : 4;  // Port multiplier
  uint8_t  reserved0 : 2;        // Reserved
  uint8_t  interrupt : 1;        // Interrupt bit
  uint8_t  reserved1 : 1;        // Reserved
 
  uint8_t  status;               // Status register
  uint8_t  error;                // Error register
 
  // DWORD 1
  uint8_t  lba0;                 // LBA low register, 7:0
  uint8_t  lba1;                 // LBA mid register, 15:8
  uint8_t  lba2;                 // LBA high register, 23:16
  uint8_t  device;               // Device register
 
  // DWORD 2
  uint8_t  lba3;                 // LBA register, 31:24
  uint8_t  lba4;                 // LBA register, 39:32
  uint8_t  lba5;                 // LBA register, 47:40
  uint8_t  reserved2;            // Reserved
 
  // DWORD 3
  uint8_t  count0;               // Count register, 7:0
  uint8_t  count1;               // Count register, 15:8
  uint8_t  reserved3[2];         // Reserved
 
  // DWORD 4
  uint8_t  reserved4[4];         // Reserved
} __attribute__((packed));

struct AhciFisData
{
  // DWORD 0
  uint8_t  fis_type;             // kAhciFisTypeData
 
  uint8_t  port_multiplier : 4;  // Port multiplier
  uint8_t  reserved0 : 4;        // Reserved
 
  uint8_t  reserved1[2];         // Reserved
 
  // DWORD 1 ~ N
  uint32_t data[1];              // Payload
} __attribute__((packed));

enum AhciFisPioDireciton {
  kAhciFisPioDirectionToDevice = 0,
  kAhciFisPioDirectionToHost   = 1
};

struct AhciFisPioSetup
{
  // DWORD 0
  uint8_t  fis_type;             // kAhciFisTypePioSetup
 
  uint8_t  port_multiplier : 4;  // Port multiplier
  uint8_t  reserved0 : 1;        // Reserved
  uint8_t  direction : 1;        // Data transfer direction, 1 - device to host
  uint8_t  interrupt : 1;        // Interrupt bit
  uint8_t  reserved1 : 1;        // Reserved
 
  uint8_t  status;               // Status register
  uint8_t  error;                // Error register
 
  // DWORD 1
  uint8_t  lba0;                 // LBA low register, 7:0
  uint8_t  lba1;                 // LBA mid register, 15:8
  uint8_t  lba2;                 // LBA high register, 23:16
  uint8_t  device;               // Device register
 
  // DWORD 2
  uint8_t  lba3;                 // LBA register, 31:24
  uint8_t  lba4;                 // LBA register, 39:32
  uint8_t  lba5;                 // LBA register, 47:40
  uint8_t  reserved2;            // Reserved
 
  // DWORD 3
  uint8_t  count0;               // Count register, 7:0
  uint8_t  count1;               // Count register, 15:8
  uint8_t  reserved3;            // Reserved
  uint8_t  e_status;             // New value of status register
 
  // DWORD 4
  uint16_t transfer_count;       // Transfer count
  uint8_t  reserved4[2];         // Reserved
} __attribute__((packed));

struct AhciFisDmaSetup
{
  // DWORD 0
  uint8_t  fis_type;             // kAhciFisTypeDmaSetup
 
  uint8_t  port_multiplier : 4;  // Port multiplier
  uint8_t  reserved0 : 1;        // Reserved
  uint8_t  direction : 1;        // Data transfer direction, 1 - device to host
  uint8_t  interrupt : 1;        // Interrupt bit
  uint8_t  auto_activate : 1;    // Auto-activate. Specifies if DMA Activate FIS is needed
 
  uint8_t  reserved1[2];         // Reserved
 
  // DWORD 1 & 2
  uint64_t dma_buffer_id;        // DMA Buffer Identifier. Used to Identify DMA buffer in host memory.
                                 // SATA Spec says host specific and not in Spec. Trying AHCI spec might work.
 
  // DWORD 3
  uint32_t reserved2;            // More reserved

  // DWORD 4
  uint32_t dma_buffer_offset;    // Byte offset into buffer. First 2 bits must be 0

  // DWORD 5
  uint32_t transfer_count;       // Number of bytes to transfer. Bit 0 must be 0

  // DWORD 6
  uint32_t reserved3;            // Reserved
} __attribute__((packed));

struct AhciRxFis
{
  // 0x00
  AhciFisDmaSetup       dma_fis;
  uint8_t               pad0[4];
  // 0x20
  AhciFisPioSetup       pio_fis;
  uint8_t               pad1[12];
  // 0x40
  AhciFisRegD2H         d2h_fis;
  uint8_t               pad2[4];
  // 0x58
  AhciSetDeviceBitsFis  sdb_fis;
  // 0x60
  uint8_t               unknown_fis[64];
  // 0xA0
  uint8_t               reserved[0x100 - 0xA0];
} __attribute__((packed));

#endif
