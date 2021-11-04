#include "pci_device.h"
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include "logger.h"
#include "machine.h"

void PciDevice::LoadRomFile(const char* path) {
  /* Load rom file */
  int fd = open(path, O_RDONLY);
  MV_ASSERT(fd >= 0);
  struct stat st;
  fstat(fd, &st);
  rom_bar_size_ = (st.st_size / PAGE_SIZE + 1) * PAGE_SIZE;
  rom_data_ = valloc(rom_bar_size_);
  read(fd, rom_data_, st.st_size);
  close(fd);
}

PciDevice::~PciDevice() {
  if (rom_data_) {
    free(rom_data_);
  }
}

void PciDevice::ReadPciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
  memcpy(data, header_.data + offset, length);
  MV_LOG("%s offset=0x%lx data=0x%lx length=0x%x",
    name_.c_str(), offset, *(uint32_t*)data, length);
}

void PciDevice::WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
  uint32_t value = 0;
  MV_LOG("%s offset=0x%lx data=0x%lx length=0x%x",
    name_.c_str(), offset, *(uint32_t*)data, length);

  if (offset == PCI_COMMAND) {
    memcpy(&value, data, length);
    WritePciCommand(value);
    return;
  }

  uint8_t bar = (offset - PCI_BAR_OFFSET(0)) / sizeof(uint32_t);
  if (bar >= 0 && bar < PCI_BAR_NUMS) {
    memcpy(&value, data, length);
    WritePciBar(bar, value);
    return;
  } else if (bar == 8) {
    /* rom bar */
    memcpy(&value, data, length);
    if (value == 0xfffff800 || value == 0xfffffffe) {
      uint32_t mask = value;
      value = ~(rom_bar_size_ - 1);
      /* Preserve the special bits. */
      value = (value & mask) | (header_.rom_bar & ~mask);
      header_.rom_bar = value;
      return;
    } else if (value > 0) {
      UpdateRomMapAddress(value & 0xfffff800);
    }
  }

  memcpy(header_.data + offset, data, length);
}

void PciDevice::UpdateRomMapAddress(uint32_t address) {
  auto mm = manager_->machine()->memory_manager();
  if (rom_bar_memory_region_) {
    if (rom_bar_memory_region_->gpa == address) {
      /* unchanged */
      return;
    }
    mm->Unmap((const MemoryRegion*)rom_bar_memory_region_);
  }
  rom_bar_memory_region_ = mm->Map(address, rom_bar_size_, rom_data_, kMemoryTypeRam);
}

void PciDevice::WritePciCommand(uint16_t new_command) {
  int i;
  bool toggle_io, toggle_mem;

  toggle_io = (header_.command ^ new_command) & PCI_COMMAND_IO;
  toggle_mem = (header_.command ^ new_command) & PCI_COMMAND_MEMORY;

  for (i = 0; i < 6; i++) {
    if (!header_.bar[i])
      continue;

    bool bar_is_io = header_.bar[i] & PCI_BASE_ADDRESS_SPACE_IO;
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

bool PciDevice::ActivatePciBar(uint8_t bar) {
  MV_LOG("bar=%d", bar);
  bar_active_[bar] = true;
  return true;
}

bool PciDevice::DeactivatePciBar(uint8_t bar) {
  MV_LOG("bar=%d", bar);
  bar_active_[bar] = false;
  return true;
}

bool PciDevice::ActivatePciBarsWithinRegion(uint32_t base, uint32_t size) {
  // Not implemented
  return true;
}

bool PciDevice::DeactivatePciBarsWithinRegion(uint32_t base, uint32_t size) {
  // Not implemented
  return true;
}

void PciDevice::WritePciBar(uint8_t bar, uint32_t value) {
  uint32_t mask;
  int bar_is_io = header_.bar[bar] & PCI_BASE_ADDRESS_SPACE_IO;
  if (bar_is_io)
    mask = (uint32_t)PCI_BASE_ADDRESS_IO_MASK;
  else
    mask = (uint32_t)PCI_BASE_ADDRESS_MEM_MASK;

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
    value = ~(bar_size_[bar] - 1);
    /* Preserve the special bits. */
    value = (value & mask) | (header_.bar[bar] & ~mask);
    header_.bar[bar] = value;
    return;
  }

  value = (value & mask) | (header_.bar[bar] & ~mask);

  /* Don't toggle emulation when region type access is disbled. */
  if (bar_is_io && !(header_.command & PCI_COMMAND_IO)) {
    header_.bar[bar] = value;
    return;
  }

  if (!bar_is_io && !(header_.command & PCI_COMMAND_MEMORY)) {
    header_.bar[bar] = value;
    return;
  }
  

  /*
   * BAR reassignment can be done while device access is enabled and
   * memory regions for different devices can overlap as long as no access
   * is made to the overlapping memory regions. To implement BAR
   * reasignment, we deactivate emulation for the region described by the
   * BAR value that the guest is changing, we disable emulation for the
   * regions that overlap with the new one (by scanning through all PCI
   * devices), we enable emulation for the new BAR value and finally we
   * enable emulation for all device regions that were overlapping with
   * the old value.
   */
  uint32_t old_addr = pci_bar_address(header_.bar[bar]);
  uint32_t new_addr = pci_bar_address(value);
  uint32_t bar_size = bar_size_[bar];

  if (bar_active_[bar]) {
    if (!DeactivatePciBar(bar))
      return;

    if (!DeactivatePciBarsWithinRegion(new_addr, bar_size)) {
      /*
      * We cannot update the BAR because of an overlapping region
      * that failed to deactivate emulation, so keep the old BAR
      * value and re-activate emulation for it.
      */
      ActivatePciBar(bar);
      return;
    }
  }

  header_.bar[bar] = value;

  if (!bar_active_[bar]) {
    if (!ActivatePciBar(bar)) {
      ActivatePciBarsWithinRegion(new_addr, bar_size);
      return;
    }
    ActivatePciBarsWithinRegion(old_addr, bar_size);
  }
}
