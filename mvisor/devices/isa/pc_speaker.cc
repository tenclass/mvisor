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
