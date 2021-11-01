#ifndef _MVISOR_PCI_DEVICE_H
#define _MVISOR_PCI_DEVICE_H

#include <linux/pci_regs.h>
#include "device.h"

/*
 * PCI Configuration Mechanism #1 I/O ports. See Section 3.7.4.1.
 * ("Configuration Mechanism #1") of the PCI Local Bus Specification 2.1 for
 * details.
 */
#define PCI_CONFIG_ADDRESS 0xcf8
#define PCI_CONFIG_DATA    0xcfc
#define PCI_CONFIG_BUS_FORWARD 0xcfa
#define PCI_IO_SIZE        0x100
#define PCI_IOPORT_START   0x6200
#define PCI_CONFIG_SIZE (1ULL << 24)

struct msi_msg {
  uint32_t address_lo; /* low 32 bits of msi message address */
  uint32_t address_hi; /* high 32 bits of msi message address */
  uint32_t data;  /* 16 bits of msi message data */
};


struct msix_table {
  struct msi_msg msg;
  uint32_t ctrl;
};

struct msix_cap {
  uint8_t cap;
  uint8_t next;
  uint16_t ctrl;
  uint32_t table_offset;
  uint32_t pba_offset;
};

struct msi_cap_64 {
  uint8_t cap;
  uint8_t next;
  uint16_t ctrl;
  uint32_t address_lo;
  uint32_t address_hi;
  uint16_t data;
  uint16_t _align;
  uint32_t mask_bits;
  uint32_t pend_bits;
};

struct msi_cap_32 {
  uint8_t cap;
  uint8_t next;
  uint16_t ctrl;
  uint32_t address_lo;
  uint16_t data;
  uint16_t _align;
  uint32_t mask_bits;
  uint32_t pend_bits;
};

struct pci_cap_hdr {
  uint8_t type;
  uint8_t next;
};

union PciConfigAddress {
  struct {
    unsigned reg_offset  : 2;  /* 1  .. 0  */
    unsigned reg_number  : 6;  /* 7  .. 2  */
    unsigned devfn     : 8;
    unsigned bus      : 8;  /* 23 .. 16 */
    unsigned reserved   : 7;  /* 30 .. 24 */
    unsigned enabled    : 1;  /* 31       */
  };
  uint32_t data;
};

#define PCI_BAR_OFFSET(b) (offsetof(struct PciConfigHeader, bar[b]))
#define PCI_DEVICE_CONFIG_SIZE 256
#define PCI_DEVICE_CONFIG_MASK (PCI_DEVICE_CONFIG_SIZE - 1)

struct PciConfigHeader {
  /* Configuration space, as seen by the guest */
  union {
    struct {
      uint16_t   vendor_id;
      uint16_t   device_id;
      uint16_t   command;
      uint16_t   status;
      uint8_t    revision_id;
      uint8_t    class_code[3];
      uint8_t    cacheline_size;
      uint8_t    latency_timer;
      uint8_t    header_type;
      uint8_t    bist;
      uint32_t   bar[6];
      uint32_t   card_bus;
      uint16_t   subsys_vendor_id;
      uint16_t   subsys_id;
      uint32_t   exp_rom_bar;
      uint8_t    capabilities;
      uint8_t    reserved1[3];
      uint32_t   reserved2;
      uint8_t    irq_line;
      uint8_t    irq_pin;
      uint8_t    min_gnt;
      uint8_t    max_lat;
      struct msix_cap msix;
    } __attribute__((packed));
    /* Pad to PCI config space size */
    uint8_t __pad[PCI_DEVICE_CONFIG_SIZE];
  };
};

/* Get last byte of a range from offset + length.
 * Undefined for ranges that wrap around 0. */
static inline uint64_t range_get_last(uint64_t offset, uint64_t len)
{
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
  PciDevice(DeviceManager* manager) : Device(manager) {}

  uint8_t bus() { return 0; }
  uint8_t devfn() { return devfn_; }

  virtual void ReadPciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length);
  virtual void WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length);
  void WritePciCommand(uint16_t command);
  void WritePciBar(uint8_t index, uint32_t value);
 protected:
  friend class DeviceManager;
  void ActivatePciBar(uint8_t index);
  void DeactivatePciBar(uint8_t index);

  PciConfigHeader header_ = { 0 };
  uint32_t bar_size_[6] = { 0 };
  bool bar_active_[6] = { 0 };
  uint8_t devfn_ = 0;
};

#endif // _MVISOR_PCI_DEVICE_H
