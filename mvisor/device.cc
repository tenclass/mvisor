#include "device.h"


Device::Device(DeviceManager* manager)
  : manager_(manager) {

}

Device::~Device() {

}

void Device::AddIoResource(IoResourceType type, uint64_t base, uint64_t length) {
  io_resources_.emplace_back(IoResource {
    .type = type,
    .base = base,
    .length = length
  });
}

/*

      if (io->port == 0x64) {
        // Keyboard
        break;
      } else if (io->port == 0x70 || io->port == 0x71) {
        // RTC
        break;
      } else if (io->port == 0x92) {
        // A20
        if (io->direction == KVM_EXIT_IO_IN) {
          *data = 0x02;
        }
      } else if (io->port == 0x402) {
        // SeaBIOS debug
        if (io->direction == KVM_EXIT_IO_OUT) {
          putchar(*data);
          fflush(stdout);
        } else {
          *data = getchar();
        }
      } else if (io->port == 0xcf8 || io->port == 0xcfc) {
        // PCI
      } else {
        MV_LOG("unhandled io %s port: 0x%x size: %d data: %016lx count: %d",
          io->direction == KVM_EXIT_IO_OUT ? "out" : "in",
          io->port, io->size, *(uint64_t*)data, io->count);
        // PrintRegisters();
        // getchar();
      }
      break;
  */
