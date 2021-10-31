#include "device.h"
#include "logger.h"

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

void Device::OnRead(uint64_t base, uint8_t* data, uint32_t size) {
  MV_PANIC("not implemented OnRead for %s base=0x%lx size=%d", name_.c_str(), base, size);
}

void Device::OnWrite(uint64_t base, uint8_t* data, uint32_t size) {
  MV_PANIC("not implemented OnWrite for %s base=0x%lx size=%d", name_.c_str(), base, size);
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
