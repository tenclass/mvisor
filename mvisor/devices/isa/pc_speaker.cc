#include "device.h"

/* The speaker is part of PIT. We use KVM in-kernel PIT, the speaker could be
 * implemented or dummied in userspace.
 */
class PcSpeaker : public Device {
 private:
  uint8_t state_ = 0;

 public:
  PcSpeaker () {
    AddIoResource(kIoResourceTypePio, 0x61, 1, "PcSpeaker Controller");
  }

  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
    /* FIXME: this might be incorrect */
    data[0] = state_;
  }

  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
    state_ = data[0];
    MV_LOG("speaker %s, state=0x%x", (state_ & 1) ? "enabled" : "disabled", state_);
  }
};

DECLARE_DEVICE(PcSpeaker)
