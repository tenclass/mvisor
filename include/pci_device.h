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

#ifndef _MVISOR_PCI_DEVICE_H
#define _MVISOR_PCI_DEVICE_H

#include <linux/pci_regs.h>
#include "device.h"
#include "logger.h"

#define PCI_MULTI_FUNCTION    0x80
#define PCI_MAX_MSIX_ENTRIES  32

struct MsiMessage {
  uint32_t address_lo; /* low 32 bits of msi message address */
  uint32_t address_hi; /* high 32 bits of msi message address */
  uint32_t data;       /* 16 bits of msi message data */
} __attribute__((packed));


struct MsiXTableEntry {
  struct MsiMessage message;
  uint32_t control;
} __attribute__((packed));

struct MsiXCapability {
  uint8_t capability;
  uint8_t next;
  uint16_t control;
  uint32_t table_offset;
  uint32_t pba_offset;
} __attribute__((packed));

struct MsiCapability64 {
  uint8_t capability;
  uint8_t next;
  uint16_t control;
  uint32_t address0;
  uint32_t address1;
  uint16_t data;
  uint16_t _align;
  uint32_t mask_bits;
  uint32_t pend_bits;
} __attribute__((packed));

struct MsiCapability32 {
  uint8_t capability;
  uint8_t next;
  uint16_t control;
  uint32_t address;
  uint16_t data;
  uint16_t _align;
  uint32_t mask_bits;
  uint32_t pend_bits;
} __attribute__((packed));

struct PciMsiConfig {
  bool      enabled;
  bool      is_64bit;
  bool      is_msix;
  uint8_t   offset;
  uint8_t   length;
  union {
    MsiCapability32* msi32;
    MsiCapability64* msi64;
    MsiXCapability*  msix;
  };
  /* MSI-X BAR */
  uint8_t        msix_bar;
  uint16_t       msix_table_size;
  uint64_t       msix_space_offset;
  uint64_t       msix_space_size;
  MsiXTableEntry msix_table[PCI_MAX_MSIX_ENTRIES];
};

struct PciCapabilityHeader {
  uint8_t type;
  uint8_t next;
} __attribute__((packed));

union PciConfigAddress {
  struct {
    unsigned reg_offset  : 2;  /* 1  .. 0  */
    unsigned reg_number  : 6;  /* 7  .. 2  */
    unsigned function    : 3;  /* 10 .. 8 */
    unsigned slot        : 5;  /* 15 .. 11 */
    unsigned bus         : 8;  /* 23 .. 16 */
    unsigned reserved    : 7;  /* 30 .. 24 */
    unsigned enabled     : 1;  /* 31       */
  };
  uint8_t   data[4];
  uint32_t  value;
};

#define PCI_BAR_OFFSET(b) (offsetof(struct PciConfigHeader, bars[b]))
#define PCI_DEVICE_CONFIG_SIZE 256
#define PCI_DEVICE_CONFIG_MASK (PCI_DEVICE_CONFIG_SIZE - 1)
#define PCI_BAR_NUMS 6
#define PCIE_DEVICE_CONFIG_SIZE 0x1000

#define Q35_MASK(bit, ms_bit, ls_bit) \
  ((uint##bit##_t)(((1ULL << ((ms_bit) + 1)) - 1) & ~((1ULL << ls_bit) - 1)))

struct PciConfigHeader {
  /* Configuration space, as seen by the guest */
  union {
    struct {
      uint16_t   vendor_id;
      uint16_t   device_id;
      uint16_t   command;
      uint16_t   status;
      unsigned   revision_id  : 8;
      unsigned   class_code   : 24;
      uint8_t    cacheline_size;
      uint8_t    latency_timer;
      uint8_t    header_type;
      uint8_t    bist;
      uint32_t   bars[6];
      uint32_t   card_bus;
      uint16_t   subsys_vendor_id;
      uint16_t   subsys_id;
      uint32_t   rom_bar;
      uint8_t    capability;
      uint8_t    reserved1[3];
      uint32_t   reserved2;
      uint8_t    irq_line;
      uint8_t    irq_pin;
      uint8_t    min_gnt;
      uint8_t    max_lat;
    } __attribute__((packed));
    /* Pad to PCI config space size */
    uint8_t data[PCIE_DEVICE_CONFIG_SIZE];
  };
};

struct PciBarInfo {
  IoResourceType        type;
  uint32_t              size;
  uint32_t              address;
  uint32_t              address_mask;
  uint32_t              special_bits;
  uint64_t              address64;
  bool                  active;
  void*                 host_memory;
};

struct PciRomBarInfo {
  uint32_t size;
  void* data;
  const MemoryRegion* mapped_region;
};

/* Get last byte of a range from offset + length.
 * Undefined for ranges that wrap around 0. */
static inline uint64_t range_get_last(uint64_t offset, uint64_t len)
{
  MV_ASSERT(len > 0);
  return offset + len - 1;
}

/* Check whether 2 given ranges overlap.
 * Undefined if ranges that wrap around 0. */
static inline int ranges_overlap(uint64_t first1, uint64_t len1,
                                 uint64_t first2, uint64_t len2)
{
  uint64_t last1 = range_get_last(first1, len1);
  uint64_t last2 = range_get_last(first2, len2);

  return !(last2 < first1 || last1 < first2);
}

class PciDevice : public Device {
 public:
  PciDevice();
  virtual ~PciDevice();
  virtual void Disconnect();

  inline uint8_t bus() { return bus_; }
  inline uint8_t slot() { return slot_; }
  inline uint8_t function() { return function_; }
  inline const PciConfigHeader& pci_header() { return pci_header_; }
  inline const PciBarInfo& pci_bar(uint8_t index) { return pci_bars_[index]; }
  inline uint  pci_config_size() { return is_pcie_ ? PCIE_DEVICE_CONFIG_SIZE : PCI_DEVICE_CONFIG_SIZE; }

  virtual void ReadPciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length);
  virtual void WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length);
  virtual void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void WritePciCommand(uint16_t command);

  /* PCI Migration */
  virtual bool SaveState(MigrationWriter* writer);
  virtual bool LoadState(MigrationReader* reader);

 protected:
  friend class DeviceManager;

  /* PCI devices may override these members to handle bar regsiter events */
  virtual bool ActivatePciBar(uint index);
  virtual bool DeactivatePciBar(uint index);

  void UpdateRomBarAddress(uint32_t address);
  void UpdatePciBarAddress(uint index, uint32_t address);
  void LoadRomFile(const char* path);
  void SetupPciBar(uint index, uint32_t size, IoResourceType type);
  void SetupPciBar64(uint index, uint64_t size, IoResourceType type);
  uint8_t* AddCapability(uint8_t cap, const uint8_t* data, uint8_t length);
  void AddMsiCapability();
  void AddMsiXCapability(uint bar_index, uint16_t table_size, uint64_t space_offset, uint64_t space_size);
  void SignalMsi(int vector = 0);
  void SetIrq(uint level);

  uint16_t          bus_;
  uint8_t           slot_;
  uint8_t           function_;
  PciConfigHeader   pci_header_;
  PciBarInfo        pci_bars_[PCI_BAR_NUMS];
  PciRomBarInfo     pci_rom_;
  PciMsiConfig      msi_config_;
  uint16_t          next_capability_offset_;
  bool              is_pcie_;
};

#endif // _MVISOR_PCI_DEVICE_H
