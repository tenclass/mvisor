#include "pci_device.h"
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include "logger.h"
#include "machine.h"


PciDevice::PciDevice() {
  devfn_ = 0;
  bzero(&pci_header_, sizeof(pci_header_));
  bzero(&pci_bars_, sizeof(pci_bars_));
  bzero(&pci_rom_, sizeof(pci_rom_));
}

/* Some PCI device has ROM file */
void PciDevice::LoadRomFile(const char* path) {
  /* Load rom file */
  int fd = open(path, O_RDONLY);
  MV_ASSERT(fd >= 0);
  struct stat st;
  fstat(fd, &st);

  pci_rom_.size = (st.st_size / PAGE_SIZE + 1) * PAGE_SIZE;
  pci_rom_.data = valloc(pci_rom_.size);
  read(fd, pci_rom_.data, st.st_size);
  close(fd);
}

PciDevice::~PciDevice() {
  if (pci_rom_.data) {
    free(pci_rom_.data);
  }
}

void PciDevice::ReadPciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
  memcpy(data, pci_header_.data + offset, length);
}

void PciDevice::WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
  if (offset == PCI_COMMAND) {
    MV_ASSERT(length == 2);
    WritePciCommand(*(uint16_t*)data);
    memcpy(pci_header_.data + offset, data, length);
    return;
  }

  uint8_t bar = (offset - PCI_BAR_OFFSET(0)) / sizeof(uint32_t);
  if (bar >= 0 && bar < PCI_BAR_NUMS) {
    MV_ASSERT(length == 4);;
    WritePciBar(bar, *(uint32_t*)data);
    return;
  } else if (bar == 8) {
    MV_ASSERT(length == 4);
    /* write rom bar address */
    uint32_t value = *(uint32_t*)data;
    if ((value & 0xfffff800) == 0xfffff800) {
      uint32_t mask = value;
      pci_header_.rom_bar = ~(pci_rom_.size - 1) & mask;
      return;
    } else if (value) {
      UpdateRomMapAddress(value & 0xfffff800);
    }
  }

  memcpy(pci_header_.data + offset, data, length);
}

/* Guest rewrite the ROM address, remmap to new address */
void PciDevice::UpdateRomMapAddress(uint32_t address) {
  auto mm = manager_->machine()->memory_manager();
  if (pci_rom_.mapped_region) {
    if (pci_rom_.mapped_region->gpa == address) {
      /* unchanged */
      return;
    }
    mm->Unmap(&pci_rom_.mapped_region);
  }

  pci_rom_.mapped_region = mm->Map(address, pci_rom_.size, pci_rom_.data,
    kMemoryTypeRam, "PCI ROM");
}

/* Handle IO, MMIO ON or OFF */
void PciDevice::WritePciCommand(uint16_t new_command) {
  int i;
  bool toggle_io, toggle_mem;

  toggle_io = (pci_header_.command ^ new_command) & PCI_COMMAND_IO;
  toggle_mem = (pci_header_.command ^ new_command) & PCI_COMMAND_MEMORY;

  for (i = 0; i < PCI_BAR_NUMS; i++) {
    if (!pci_header_.bars[i])
      continue;

    bool bar_is_io = pci_header_.bars[i] & PCI_BASE_ADDRESS_SPACE_IO;
    if (toggle_io && bar_is_io) {
      if (new_command & PCI_COMMAND_IO)
        ActivatePciBar(i);
      else
        DeactivatePciBar(i);
    }

    if (toggle_mem && !bar_is_io) {
      if (new_command & PCI_COMMAND_MEMORY)
        ActivatePciBar(i);
      else
        DeactivatePciBar(i);
    }
  }
}

/* Call this function in device constructor */
void PciDevice::AddPciBar(uint8_t index, uint32_t size, IoResourceType type) {
  auto &bar = pci_bars_[index];
  MV_ASSERT(bar.size == 0 && pci_header_.bars[index] == 0);
  bar.size = size;
  bar.active = false;
  bar.type = type;
  if (type == kIoResourceTypePio) {
    bar.address_mask = PCI_BASE_ADDRESS_IO_MASK;
    bar.special_bits = PCI_BASE_ADDRESS_SPACE_IO;
  } else if (type == kIoResourceTypeMmio) {
    bar.address_mask = PCI_BASE_ADDRESS_MEM_MASK;
    bar.special_bits = 0;
  } else if (type == kIoResourceTypeRam) {
    bar.address_mask = PCI_BASE_ADDRESS_MEM_MASK;
    bar.special_bits = 8; // Prefetchable
  }
  pci_header_.bars[index] |= bar.special_bits;
}

/* Called when an bar is activate by guest BIOS or OS */
bool PciDevice::ActivatePciBar(uint8_t index) {
  auto &bar = pci_bars_[index];

  if (bar.type == kIoResourceTypePio) {
    AddIoResource(kIoResourceTypePio, bar.address, bar.size, "PCI BAR IO");
  } else if (bar.type == kIoResourceTypeMmio) {
    AddIoResource(kIoResourceTypeMmio, bar.address, bar.size, "PCI BAR MMIO");
  } else if (bar.type == kIoResourceTypeRam) {
    MV_ASSERT(bar.host_memory != nullptr && bar.mapped_region == nullptr);
    auto mm = manager_->machine()->memory_manager();
    bar.mapped_region = mm->Map(bar.address, bar.size, bar.host_memory,
      kMemoryTypeRam, "PCI BAR RAM");
  }
  bar.active = true;
  return true;
}

/* Release resources when resetting the bar address */
bool PciDevice::DeactivatePciBar(uint8_t index) {
  auto &bar = pci_bars_[index];
  if (bar.type == kIoResourceTypePio) {
    RemoveIoResource(kIoResourceTypePio, bar.address);
  } else if (bar.type == kIoResourceTypeMmio) {
    RemoveIoResource(kIoResourceTypeMmio, bar.address);
  } else if (bar.type == kIoResourceTypeRam) {
    MV_ASSERT(bar.mapped_region != nullptr);
    auto mm = manager_->machine()->memory_manager();
    mm->Unmap(&bar.mapped_region);
  }
  bar.active = false;
  return true;
}

void PciDevice::WritePciBar(uint8_t index, uint32_t value) {
  auto &bar = pci_bars_[index];

  /*
   * If the kernel masks the BAR, it will expect to find the size of the
   * BAR there next time it reads from it. After the kernel reads the
   * size, it will write the address back.
   *
   * According to the PCI local bus specification REV 3.0: The number of
   * upper bits that a device actually implements depends on how much of
   * the address space the device will respond to. A device that wants a 1
   * MB memory address space (using a 32-bit base address register) would
   * build the top 12 bits of the address register, hardwiring the other
   * bits to 0.
   *
   * Furthermore, software can determine how much address space the device
   * requires by writing a value of all 1's to the register and then
   * reading the value back. The device will return 0's in all don't-care
   * address bits, effectively specifying the address space required.
   *
   * Software computes the size of the address space with the formula
   * S =  ~B + 1, where S is the memory size and B is the value read from
   * the BAR. This means that the BAR value that kvmtool should return is
   * B = ~(S - 1).
   */
  if (value == 0xffffffff) {
    value = ~(bar.size - 1);
    pci_header_.bars[index] = (value & bar.address_mask) | bar.special_bits;
    return;
  }

  uint64_t new_address = value & bar.address_mask;
  if (bar.address == new_address) {
    pci_header_.bars[index] = bar.address | bar.special_bits;
    return;
  }

  /* Don't toggle emulation when region type access is disabled. */
  if (bar.type == kIoResourceTypePio && !(pci_header_.command & PCI_COMMAND_IO)) {
    bar.address = new_address;
    pci_header_.bars[index] = bar.address | bar.special_bits;
    return;
  }

  if (bar.type != kIoResourceTypePio && !(pci_header_.command & PCI_COMMAND_MEMORY)) {
    bar.address = new_address;
    pci_header_.bars[index] = bar.address | bar.special_bits;
    return;
  }

  if (bar.active) {
    if (!DeactivatePciBar(index))
      return;
  }

  bar.address = new_address;
  pci_header_.bars[index] = bar.address | bar.special_bits;

  if (bar.address && !bar.active) {
    ActivatePciBar(index);
  }
}
