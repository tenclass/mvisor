#include "pci_device.h"
#include <cstring>
#include "logger.h"

void PciDevice::ReadPciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
  uint8_t* base = (uint8_t*)&header_;
  memcpy(data, base + offset, length);
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

void PciDevice::ActivatePciBar(uint8_t bar) {
  MV_PANIC("not implemented bar=%d", bar);
}

void PciDevice::DeactivatePciBar(uint8_t bar) {
  MV_PANIC("not implemented bar=%d", bar);
}

void PciDevice::WritePciBar(uint8_t bar, uint32_t value) {
  MV_PANIC("not implemented bar=%d value=%x", bar, value);
}

void PciDevice::WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
  uint8_t* base = (uint8_t*)&header_;
	uint32_t value = 0;
	MV_LOG("%s offset=0x%lx data=0x%lx length=0x%x",
		name_.c_str(), offset, *(uint32_t*)data, length);
	/*
	 * legacy hack: ignore writes to uninitialized regions (e.g. ROM BAR).
	 * Not very nice but has been working so far.
	 * if (*(uint32_t *)(base + offset) == 0)
	 *   return;
	 */
	if (offset == PCI_COMMAND) {
		memcpy(&value, data, length);
    WritePciCommand(value);
		return;
	}

	uint8_t bar = (offset - PCI_BAR_OFFSET(0)) / sizeof(uint32_t);
	if (bar < 6) {
		memcpy(&value, data, length);
		WritePciBar(bar, value);
		return;
	}

	memcpy(base + offset, data, length);
}

