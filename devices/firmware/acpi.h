#ifndef _MVISOR_DEVICES_FIRMWARE_ACPI_H
#define _MVISOR_DEVICES_FIRMWARE_ACPI_H

#include <cstdint>

/*
 * ACPI 2.0 Generic Address Space definition.
 */
struct acpi_20_generic_address {
  uint8_t  address_space_id;
  uint8_t  bit_width;
  uint8_t  bit_offset;
  uint8_t  access_width;
  uint64_t address;
} __attribute__((packed));


struct acpi_rdsp_table {          /* Root System Descriptor Pointer */
  uint8_t  signature [8];         /* ACPI signature, contains "RSD PTR " */
  uint8_t  checksum;              /* To make sum of struct == 0 */
  uint8_t  oem_id [6];            /* OEM identification */
  uint8_t  revision;              /* Must be 0 for 1.0, 2 for 2.0 */
  uint32_t rsdt_physical_address; /* 32-bit physical address of RSDT */
  uint32_t length;                /* XSDT Length in bytes including hdr */
  uint64_t xsdt_physical_address; /* 64-bit physical address of XSDT */
  uint8_t  extended_checksum;     /* Checksum of entire table */
  uint8_t  reserved [3];          /* Reserved field must be 0 */
};


struct acpi_table_header {        /* Common ACPI table header */
  uint32_t signature;             /* ACPI signature (4 ASCII characters) */ \
  uint32_t length;                /* Length of table, in bytes, including header */ \
  uint8_t  revision;              /* ACPI Specification minor version # */ \
  uint8_t  checksum;              /* To make sum of entire table == 0 */ \
  uint8_t  oem_id [6];            /* OEM identification */ \
  uint8_t  oem_table_id [8];      /* OEM table identification */ \
  uint32_t oem_revision;          /* OEM revision number */ \
  uint8_t  asl_compiler_id [4];   /* ASL compiler vendor ID */ \
  uint32_t asl_compiler_revision; /* ASL compiler revision number */
};


struct acpi_rsdt_table {          /* Root System Descriptor Table */
  acpi_table_header header;
  uint32_t entry [0];             /* Array of pointers to other ACPI tables */
};


/*
 * Fixed ACPI Description Table Fixed Feature Flags
 */
#define    ACPI_FADT_F_WBINVD            (1 << 0)
#define    ACPI_FADT_F_WBINVD_FLUSH      (1 << 1)
#define    ACPI_FADT_F_PROC_C1           (1 << 2)
#define    ACPI_FADT_F_P_LVL2_UP         (1 << 3)
#define    ACPI_FADT_F_PWR_BUTTON        (1 << 4)
#define    ACPI_FADT_F_SLP_BUTTON        (1 << 5)
#define    ACPI_FADT_F_FIX_RTC           (1 << 6)
#define    ACPI_FADT_F_RTC_S4            (1 << 7)
#define    ACPI_FADT_F_TMR_VAL_EXT       (1 << 8)
#define    ACPI_FADT_F_DCK_CAP           (1 << 9)
#define    ACPI_FADT_F_RESET_REG_SUP     (1 << 10)
#define    ACPI_FADT_F_SEALED_CASE       (1 << 11)
#define    ACPI_FADT_F_HEADLESS          (1 << 12)
#define    ACPI_FADT_F_CPU_SW_SLP        (1 << 13)
#define    ACPI_FADT_F_PCI_EXP_WAK       (1 << 14)
#define    ACPI_FADT_F_USE_PLATFORM_CLOCK (1 << 15)
#define    ACPI_FADT_F_S4_RTC_STS_VALID   (1 << 16)
#define    ACPI_FADT_F_REMOTE_POWER_ON_CAPABLE  (1 << 17)
#define    ACPI_FADT_F_FORCE_APIC_CLUSTER_MODEL  (1 << 18)
#define    ACPI_FADT_F_FORCE_APIC_PHYSICAL_DESTINATION_MODE  (1 << 19)
#define    ACPI_FADT_F_HW_REDUCED_ACPI    (1 << 20)
#define    ACPI_FADT_F_LOW_POWER_S0_IDLE_CAPABLE  (1 << 21)

/*
 * ACPI 1.0 Fixed ACPI Description Table (FADT)
 */
#define FACP_SIGNATURE 0x50434146 // FACP
struct acpi_fadt_table
{
  acpi_table_header header;
  uint32_t firmware_ctrl;          /* Physical address of FACS */
  uint32_t dsdt;                   /* Physical address of DSDT */
  uint8_t  interrupt_model;                  /* System Interrupt Model */
  uint8_t  reserved1;              /* Reserved */
  uint16_t sci_int;                /* System vector of SCI interrupt */
  uint32_t smi_cmd;                /* Port address of SMI command port */
  uint8_t  acpi_enable;            /* Value to write to smi_cmd to enable ACPI */
  uint8_t  acpi_disable;           /* Value to write to smi_cmd to disable ACPI */
  uint8_t  S4bios_req;             /* Value to write to SMI CMD to enter S4BIOS state */
  uint8_t  reserved2;              /* Reserved - must be zero */
  uint32_t pm1a_evt_blk;           /* Port address of Power Mgt 1a acpi_event Reg Blk */
  uint32_t pm1b_evt_blk;           /* Port address of Power Mgt 1b acpi_event Reg Blk */
  uint32_t pm1a_cnt_blk;           /* Port address of Power Mgt 1a Control Reg Blk */
  uint32_t pm1b_cnt_blk;           /* Port address of Power Mgt 1b Control Reg Blk */
  uint32_t pm2_cnt_blk;            /* Port address of Power Mgt 2 Control Reg Blk */
  uint32_t pm_tmr_blk;             /* Port address of Power Mgt Timer Ctrl Reg Blk */
  uint32_t gpe0_blk;               /* Port addr of General Purpose acpi_event 0 Reg Blk */
  uint32_t gpe1_blk;               /* Port addr of General Purpose acpi_event 1 Reg Blk */
  uint8_t  pm1_evt_len;            /* Byte length of ports at pm1_x_evt_blk */
  uint8_t  pm1_cnt_len;            /* Byte length of ports at pm1_x_cnt_blk */
  uint8_t  pm2_cnt_len;            /* Byte Length of ports at pm2_cnt_blk */
  uint8_t  pm_tmr_len;             /* Byte Length of ports at pm_tm_blk */
  uint8_t  gpe0_blk_len;           /* Byte Length of ports at gpe0_blk */
  uint8_t  gpe1_blk_len;           /* Byte Length of ports at gpe1_blk */
  uint8_t  gpe1_base;              /* Offset in gpe model where gpe1 events start */
  uint8_t  reserved3;              /* Reserved */
  uint16_t plvl2_lat;              /* Worst case HW latency to enter/exit C2 state */
  uint16_t plvl3_lat;              /* Worst case HW latency to enter/exit C3 state */
  uint16_t flush_size;             /* Size of area read to flush caches */
  uint16_t flush_stride;           /* Stride used in flushing caches */
  uint8_t  duty_offset;            /* Bit location of duty cycle field in p_cnt reg */
  uint8_t  duty_width;             /* Bit width of duty cycle field in p_cnt reg */
  uint8_t  day_alrm;               /* Index to day-of-month alarm in RTC CMOS RAM */
  uint8_t  mon_alrm;               /* Index to month-of-year alarm in RTC CMOS RAM */
  uint8_t  century;                /* Index to century in RTC CMOS RAM */
  uint16_t iapc_boot_arch;         /* Boot Architecture Flags */
  uint8_t  reserved4;              /* Reserved */
  uint32_t flags;                  /* Fixed Feature Flags */
  acpi_20_generic_address  reset_reg;         /* Reset Register Address */
  uint8_t  reset_value;            /* Value to write to reset_reg to reset system */
  uint16_t arm_boot_arch;          /* ARM Boot Architecture Flags */
  uint8_t  fadt_minor_version;     /* FADT Minor Version */
  uint8_t  x_firmware_ctrl;        /* X_Firmware Control */
  uint8_t  x_dsdt;                 /* X_DSDT */
  acpi_20_generic_address  x_pm1a_evt_blk;    /* X_PM1a Event Block Address */
  acpi_20_generic_address  x_pm1b_evt_blk;    /* X_PM1b Event Block Address */
  acpi_20_generic_address  x_pm1a_cnt_blk;    /* X_PM1a Control Block Address */
  acpi_20_generic_address  x_pm1b_cnt_blk;    /* X_PM1b Control Block Address */
  acpi_20_generic_address  x_pm2_cnt_blk;     /* X_PM2 Control Block Address */
  acpi_20_generic_address  x_pm_tmr_blk;      /* X_PM Timer Block Address */
  acpi_20_generic_address  x_gpe0_blk;        /* X_GPE0 Block Address */
  acpi_20_generic_address  x_gpe1_blk;        /* X_GPE1 Block Address */
  acpi_20_generic_address  sleep_control_reg; /* Sleep Control Register */
  acpi_20_generic_address  sleep_status_reg;  /* Sleep Status Register */
  uint8_t  hypervisor_vendor_id [12]; /* Hypervisor Vendor Identity */
} __attribute__((packed));


/*
 * MADT values and structures
 */

/* Values for MADT PCATCompat */

#define DUAL_PIC                0
#define MULTIPLE_APIC           1

/* Master MADT */

#define APIC_SIGNATURE 0x43495041 // APIC
struct multiple_apic_table
{
  acpi_table_header header;
  uint32_t local_apic_address;     /* Physical address of local APIC */
  uint32_t flags;
} __attribute__((packed));

/* Values for Type in APIC sub-headers */

#define APIC_PROCESSOR          0
#define APIC_IO                 1
#define APIC_XRUPT_OVERRIDE     2
#define APIC_NMI                3
#define APIC_LOCAL_NMI          4
#define APIC_ADDRESS_OVERRIDE   5
#define APIC_IO_SAPIC           6
#define APIC_LOCAL_SAPIC        7
#define APIC_XRUPT_SOURCE       8
#define APIC_RESERVED           9           /* 9 and greater are reserved */

/*
 * MADT sub-structures (Follow MULTIPLE_APIC_DESCRIPTION_TABLE)
 */
#define ACPI_SUB_HEADER_DEF   /* Common ACPI sub-structure header */\
    uint8_t  type;                               \
    uint8_t  length;

/* Sub-structures for MADT */

struct madt_processor_apic
{
  uint8_t  type;
  uint8_t  length;
  uint8_t  processor_id;           /* ACPI processor id */
  uint8_t  local_apic_id;          /* Processor's local APIC id */
  uint32_t flags;
} __attribute__((packed));

struct madt_io_apic
{
  uint8_t  type;
  uint8_t  length;
  uint8_t  io_apic_id;             /* I/O APIC ID */
  uint8_t  reserved;               /* Reserved - must be zero */
  uint32_t address;                /* APIC physical address */
  uint32_t interrupt;              /* Global system interrupt where INTI
                                * lines start */
} __attribute__((packed));

struct madt_intsrcovr {
  uint8_t  type;
  uint8_t  length;
  uint8_t  bus;
  uint8_t  source;
  uint32_t gsi;
  uint16_t flags;
} __attribute__((packed));

struct madt_local_nmi {
  uint8_t  type;
  uint8_t  length;
  uint8_t  processor_id;           /* ACPI processor id */
  uint16_t flags;                  /* MPS INTI flags */
  uint8_t  lint;                   /* Local APIC LINT# */
} __attribute__((packed));


/*
 * ACPI 1.0 Firmware ACPI Control Structure (FACS)
 */
struct acpi_facs_table
{
  uint32_t signature;              /* ACPI Signature */
  uint32_t length;                 /* Length of structure, in bytes */
  uint32_t hardware_signature;     /* Hardware configuration signature */
  uint32_t firmware_waking_vector; /* ACPI OS waking vector */
  uint32_t global_lock;            /* Global Lock */
  uint32_t flags;
  uint8_t  resverved3 [40];        /* Reserved - must be zero */
} __attribute__((packed));

/* PCI fw r3.0 MCFG table. */
/* Subtable */
struct acpi_mcfg_allocation {
  uint64_t  address;                /* Base address, processor-relative */
  uint16_t  pci_segment;            /* PCI segment group number */
  uint8_t   start_bus_number;       /* Starting PCI Bus number */
  uint8_t   end_bus_number;         /* Final PCI Bus number */
  uint32_t  reserved;
} __attribute__((packed));

#define MCFG_SIGNATURE 0x4746434d       // MCFG
struct acpi_mcfg_table {
  acpi_table_header header;
  uint8_t reserved[8];
  struct acpi_mcfg_allocation allocation[1];
} __attribute__((packed));


/*
 * Windows ACPI Emulated Devices Table
 * (Version 1.0 - April 6, 2009)
 * Spec: http://download.microsoft.com/download/7/E/7/7E7662CF-CBEA-470B-A97E-CE7CE0D98DC2/WAET.docx
 *
 * Helpful to speedup Windows guests and ignored by others.
 */
struct acpi_waet_table
{
  acpi_table_header header;
  uint32_t emulated_device_flags;
} __attribute__((packed));

#endif // _MVISOR_DEVICES_FIRMWARE_ACPI_H_
