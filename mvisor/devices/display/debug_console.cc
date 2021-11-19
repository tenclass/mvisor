#include "device.h"

class DebugConsole : public Device {
 public:
  DebugConsole() {
    AddIoResource(kIoResourceTypePio, 0x402, 1, "SeaBIOS Output");
  }

  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
    putchar(*data);
  }

  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
    *data = 0xe9;
  }

};

DECLARE_DEVICE(DebugConsole);
