/* 
 * MVisor I8042 PS/2
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

#include <cstring>
#include <mutex>
#include "logger.h"
#include "device_manager.h"
#include "machine.h"
#include "device_interface.h"
#include "ps2.pb.h"

#define STATUS_OFULL    0x01
#define STATUS_SYSFLAG  0x04
#define STATUS_COMMAND  0x08
#define STATUS_KEYLOCK  0x10
#define STATUS_AUXDATA  0x20
#define STATUS_TIMEOUT  0x40

#define MODE_KBD_INTERRUPT  0x01
#define MODE_AUX_INTERRUPT  0x02
#define MODE_KBD_DISABLED   0x10
#define MODE_AUX_DISABLED   0x20

#define OUTPORT_RESET   0x01
#define OUTPORT_A20     0x02
#define OUTPORT_OFULL   0x10
#define OUTPORT_AUXDATA 0x20
#define OUTPORT_CC      0xCC

#define RESPONSE_ACK 0xFA

#define KEYBOARD_IRQ 1
#define MOUSE_IRQ 12

/* Reference: https://wiki.osdev.org/%228042%22_PS/2_Controller
 */

class Ps2 : public Device, public KeyboardInputInterface {
 private:
  std::deque<uint8_t> keyboard_queue_;
  std::deque<uint8_t> mouse_queue_;

  uint8_t status_;
  uint8_t mode_;
  uint8_t last_command_;
  uint8_t output_data_;
  bool    output_data_is_read_;
  int     raised_irq_;
  uint8_t a20_gate_ = 0;

  struct {
    uint8_t buttons;
    int     dx;
    int     dy;
    uint8_t resolution;
    uint8_t sample_rate;
    uint8_t command;
    uint8_t scaling;
    uint8_t id;
    uint8_t stream_mode;
    bool    disable_streaming;
  } mouse_;

  struct {
    uint8_t leds;
    uint8_t scancode_set;
    bool    disable_scanning;
  } keyboard_;

  bool SaveState(MigrationWriter* writer) {
    Ps2State state;
    state.set_mode(mode_);
    state.set_status(status_);
    state.set_a20_gate(a20_gate_);

    auto keyboard = state.mutable_keyboard();
    keyboard->set_scancode_set(keyboard_.scancode_set);
    keyboard->set_leds(keyboard_.leds);

    auto mouse = state.mutable_mouse();
    mouse->set_resolution(mouse_.resolution);
    mouse->set_buttons(mouse_.buttons);
    mouse->set_sample_rate(mouse_.sample_rate);
    mouse->set_scaling(mouse_.scaling);
    mouse->set_stream_mode(mouse_.stream_mode);
    mouse->set_dx(mouse_.dx);
    mouse->set_dy(mouse_.dy);

    writer->WriteProtobuf("PS2", state);
    return Device::SaveState(writer);
  }

  bool LoadState(MigrationReader* reader) {
    Ps2State state;
    if (!reader->ReadProtobuf("PS2", state)) {
      return false;
    }
    mode_ = state.mode();
    status_ = state.status();
    a20_gate_ = state.a20_gate();

    /* For compatibility, defaults to enabled a20 gate */
    if (!a20_gate_) {
      a20_gate_ = OUTPORT_A20;
    }
    
    auto& keyboard = state.keyboard();
    keyboard_.scancode_set = keyboard.scancode_set();
    keyboard_.leds = keyboard.leds();

    auto& mouse = state.mouse();
    mouse_.resolution = mouse.resolution();
    mouse_.buttons = mouse.buttons();
    mouse_.sample_rate = mouse.sample_rate();
    mouse_.scaling = mouse.scaling();
    mouse_.stream_mode = mouse.stream_mode();
    mouse_.dx = mouse.dx();
    mouse_.dy = mouse.dy();
    return Device::LoadState(reader);
  }
  
  void Reset() {
    Device::Reset();

    status_ = STATUS_KEYLOCK | STATUS_COMMAND;
    mode_ = 5;
    raised_irq_ = -1;
    last_command_ = 0;
    output_data_is_read_ = true;
    a20_gate_ &= ~OUTPORT_RESET;
    
    ResetKeyboard();
    ResetMouse();
  }

  void ResetKeyboard() {
    keyboard_queue_.clear();
    keyboard_.scancode_set = 2;
    keyboard_.disable_scanning = false;
  }

  void ResetMouse() {
    mouse_queue_.clear();
    mouse_.command = 0;
    mouse_.resolution = 4;
    mouse_.sample_rate = 100;
    mouse_.scaling = 1;
    mouse_.dx = mouse_.dy = 0;
    mouse_.disable_streaming = false;
    mouse_.stream_mode = 1;
    /* 
     * https://wiki.osdev.org/PS/2_Mouse
     * 0: Normal 2 buttons mouse
     * 3: Use 3 buttons
     * 5: Use 5 buttons
     */
    mouse_.id = 3;
  }

  void RaiseIrq(int irq) {
    bool enabled = false;
    status_ |= STATUS_OFULL;
    if (irq == MOUSE_IRQ) {
      status_ |= STATUS_AUXDATA;
      enabled = mode_ & MODE_AUX_INTERRUPT;
    } else {
      status_ &= ~STATUS_AUXDATA;
      enabled = mode_ & MODE_KBD_INTERRUPT;
    }

    if (enabled) {
      manager_->SetGsiLevel(irq, 0);
      manager_->SetGsiLevel(irq, 1);
      raised_irq_ = irq;
    }
  }

  void FillOutputData() {
    if (!output_data_is_read_) {
      if (!keyboard_queue_.empty()) {
        RaiseIrq(KEYBOARD_IRQ);
      } else if (!mouse_queue_.empty()) {
        RaiseIrq(MOUSE_IRQ);
      }
      return;
    }

    if (!keyboard_queue_.empty()) {
      output_data_ = keyboard_queue_.front();
      keyboard_queue_.pop_front();
      output_data_is_read_ = false;
      RaiseIrq(KEYBOARD_IRQ);
    } else if (!mouse_queue_.empty()) {
      output_data_ = mouse_queue_.front();
      mouse_queue_.pop_front();
      output_data_is_read_ = false;
      RaiseIrq(MOUSE_IRQ);
    }
  }


  void PushKeyboard(uint8_t data) {
    keyboard_queue_.push_back(data);
    FillOutputData();
  }

  void PushMouse(uint8_t data) {
    mouse_queue_.push_back(data);
    FillOutputData();
  }

  void PushMouse4(uint8_t data[3]) {
    mouse_queue_.push_back(data[0]);
    mouse_queue_.push_back(data[1]);
    mouse_queue_.push_back(data[2]);
    mouse_queue_.push_back(data[3]);
    FillOutputData();
  }

  uint8_t ReadData() {
    status_ &= ~(STATUS_AUXDATA | STATUS_OFULL);

    if (raised_irq_ != -1) {
      manager_->SetGsiLevel(raised_irq_, 0);
      raised_irq_ = -1;
    }
    
    uint8_t ret = output_data_;
    output_data_is_read_ = true;
    FillOutputData();
    return ret;
  }

  void WriteMouseCommand(uint8_t data) {
    switch (mouse_.command)
    {
    case 0xE8: // resolution
      mouse_.resolution = data;
      mouse_.command = 0;
      PushMouse(RESPONSE_ACK);
      break;
    case 0xF3: // sample rate
      mouse_.sample_rate = data;
      mouse_.command = 0;
      PushMouse(RESPONSE_ACK);
      break;
    case 0:
      switch (data)
      {
      case 0x00:
      case 0x0A:
      case 0x88:
      case 0xE1:
        /* Linux uses these commands, why ??? */
        PushMouse(0xFE);
        break;
      case 0xE6:
        mouse_.scaling = 1;
        PushMouse(RESPONSE_ACK);
        break;
      case 0xE7:
        mouse_.scaling = 2;
        PushMouse(RESPONSE_ACK);
        break;
      case 0xE8: // set resolution
        mouse_.command = data;
        PushMouse(RESPONSE_ACK);
        break;
      case 0xE9: // send status
        PushMouse(RESPONSE_ACK);
        PushMouse(mouse_.stream_mode << 6 | (!mouse_.disable_streaming << 5) | (!mouse_.scaling << 4) | mouse_.buttons);
        PushMouse(mouse_.resolution);
        PushMouse(mouse_.sample_rate);
        break;
      case 0xEA: // set stream mode
        MV_ASSERT(data == 1);
        PushMouse(RESPONSE_ACK);
        break;
      case 0xF2: // get mouse ID
        PushMouse(RESPONSE_ACK);
        PushMouse(mouse_.id);
        break;
      case 0xF3: // set sample rate
        mouse_.command = data;
        PushMouse(RESPONSE_ACK);
        break;
      case 0xF4: // enable mouse
        mouse_.disable_streaming = false;
        PushMouse(RESPONSE_ACK);
        break;
      case 0xF5: // disable mouse
        mouse_.disable_streaming = true;
        PushMouse(RESPONSE_ACK);
        break;
      case 0xF6: // set defaults
        ResetMouse();
        PushMouse(RESPONSE_ACK);
        break;
      case 0xFF: // reset mouse
        ResetMouse();
        PushMouse(RESPONSE_ACK);
        PushMouse(0xAA);
        PushMouse(0x00);
        break;
      default:
        MV_ERROR("unhandled mouse command = 0x%x", data);
        break;
      }
      break;
    default:
      MV_ERROR("unhandled mouse command = 0x%x", mouse_.command);
      break;
    }
  }

  void WritePs2Command(uint8_t data) {
    switch (data)
    {
    case 0xED: // set LED state
      last_command_ = data;
      PushKeyboard(RESPONSE_ACK);
      break;
    case 0xF0: // get/set keyboard scancode set
      last_command_ = data;
      PushKeyboard(RESPONSE_ACK);
      break;
    case 0xF2: // set LED state
      PushKeyboard(RESPONSE_ACK);
      PushKeyboard(0xAB);
      PushKeyboard(0x41);
      break;
    case 0xF3: // set typematic rate
      last_command_ = data;
      PushKeyboard(RESPONSE_ACK);
      break;
    case 0xF4: // enable keyboard scanning
      keyboard_.disable_scanning = false;
      PushKeyboard(RESPONSE_ACK);
      break;
    case 0xF5: // disable keyboard scanning
      keyboard_.disable_scanning = true;
      PushKeyboard(RESPONSE_ACK);
      break;
    case 0xF6: // reset keyboard and enable scanning
      Reset();
      PushKeyboard(RESPONSE_ACK);
      break;
    case 0xFF: // reset keyboard
      ResetKeyboard();
      PushKeyboard(RESPONSE_ACK);
      PushKeyboard(0xAA);
      break;
    default:
      MV_ERROR("unhandled ps2 command=0x%x", data);
      break;
    }
  }

  void WriteCommandPort(uint8_t command) {
    switch (command)
    {
    case 0x20: // read 
      PushKeyboard(mode_ | (status_ & STATUS_SYSFLAG));
      break;
    case 0x60: // control mode
      last_command_ = command;
      break;
    case 0xA7: // disable mouse
      mode_ |= MODE_AUX_DISABLED;
      break;
    case 0xA8: // enable mouse
      mode_ &= ~MODE_AUX_DISABLED;
      FillOutputData();
      if (!output_data_is_read_) {
        RaiseIrq(MOUSE_IRQ);
      }
      break;
    case 0xA9: // Test mouse port
      PushKeyboard(0);
      break;
    case 0xAA: // Test controller
      status_ |= STATUS_SYSFLAG;
      PushKeyboard(0x55);
      break;
    case 0xAB: // Test keyboard
      PushKeyboard(0);
      break;
    case 0xAD: // disable keyboard
      mode_ |= MODE_KBD_DISABLED;
      break;
    case 0xAE: // enable keyboard
      mode_ &= ~MODE_KBD_DISABLED;
      FillOutputData();
      if (!output_data_is_read_) {
        RaiseIrq(KEYBOARD_IRQ);
      }
      break;
    case 0xD1 ... 0xD4: // outport utilities
      last_command_ = command;
      break;
    case 0xFE ... 0xFF: // pulse output line
      if ((command & 1) == 0) {
        if (debug_) {
          MV_LOG("system reset");
        }
        manager_->machine()->Reset();
      }
      break;
    default:
      MV_ERROR("unhandled command=0x%x", command);
      break;
    }
    status_ |= STATUS_COMMAND;
  }

  void WriteDataPort(uint8_t data) {
    uint8_t command = last_command_;
    last_command_ = 0;
    switch (command)
    {
    case 0:
      WritePs2Command(data);
      break;
    case 0x60: // write to control mode
      mode_ = data;
      break;
    case 0xD1: // write to outport (Windows use this to control a20 gate)
      if (data & OUTPORT_A20) {
        a20_gate_ |= OUTPORT_A20;
      } else {
        a20_gate_ &= ~OUTPORT_A20;
      }
      break;
    case 0xD3: // mouse loop
      PushMouse(data);
      PushMouse(RESPONSE_ACK);
      break;
    case 0xD4: // mouse status
      WriteMouseCommand(data);
      break;
    case 0xED: // set leds
      PushKeyboard(RESPONSE_ACK);
      keyboard_.leds = data;
      break;
    case 0xF0: // keyboard scancode set
      PushKeyboard(RESPONSE_ACK);
      if (data == 0) {
        PushKeyboard(keyboard_.scancode_set);
      } else {
        keyboard_.scancode_set = data;
        MV_ASSERT(keyboard_.scancode_set == 2);
      }
      break;
    case 0xF3: // set typematic rate
      PushKeyboard(RESPONSE_ACK);
      break;
    default:
      MV_ERROR("unhandled command=0x%x data=0x%x", command, data);
      break;
    }
  }


 public:
  Ps2() {
    set_default_parent_class("Ich9Lpc", "Piix3");

    AddIoResource(kIoResourceTypePio, 0x92, 1, "A20 Gate");
    AddIoResource(kIoResourceTypePio, 0x60, 1, "PS2 Data");
    AddIoResource(kIoResourceTypePio, 0x64, 1, "PS2 Command");
  }

  void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    MV_ASSERT(size == 1);
    MV_UNUSED(offset);

    switch (resource->base)
    {
    case 0x64: // command port
      status_ &= ~STATUS_TIMEOUT;
      *data = status_;
      break;
    case 0x60: // data port
      *data = ReadData();
      break;
    case 0x92: // A20 gate
      /* Always return enabled A20 gate */
      *data = a20_gate_;
      break;
    }
    if (debug_) {
      MV_LOG("read %x %x", resource->base, *data);
    }
  }


  void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    MV_ASSERT(size == 1);
    MV_UNUSED(offset);

    if (debug_) {
      MV_LOG("write %x %x", resource->base, *data);
    }

    if (resource->base == 0x64) { // command port
      WriteCommandPort(*data);
    } else if (resource->base == 0x60) { // data port
      WriteDataPort(*data);
    } else if (resource->base == 0x92) { // port 92
      a20_gate_ = *data;
      if (a20_gate_ & 1) {
        if (debug_) {
          MV_LOG("system reset");
          manager_->machine()->Reset();
        }
      }
    }
  }

  /* Called by UI thread */
  bool QueueKeyboardEvent(uint8_t scancode[10], uint8_t modifiers) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (keyboard_.disable_scanning) {
      return false;
    }

    if (modifiers != keyboard_.leds) {
      uint8_t diff = modifiers ^ keyboard_.leds;
      if (diff & 1) {
        PushKeyboard(0x46);
        PushKeyboard(0x46 | 0x80);
      } else if (diff & 2) {
        PushKeyboard(0x45);
        PushKeyboard(0x45 | 0x80);
      } else if (diff & 4) {
        PushKeyboard(0x3A);
        PushKeyboard(0x3A | 0x80);
      }
    }

    /* skip modifiers input */
    uint8_t code = (scancode[0] == 0xE0 ? scancode[1] : scancode[0]) & 0x7F;
    if (code == 0x45 || code == 0x46 || code == 0x3A) {
      return true;
    }
  
    for (int i = 0; i < 10 && scancode[i]; i++) {
      PushKeyboard(scancode[i]);
    }
    return true;
  }

  /* Called by UI thread */
  bool QueueMouseEvent(uint button_state, int rel_x, int rel_y, int rel_z) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (mouse_.disable_streaming) {
      return false;
    }
  
    uint8_t state = mouse_.buttons = (uint8_t)button_state;
    rel_y = -rel_y;
    state |= 8; // Always 1
    if (rel_x < 0) {
      rel_x = 0x100 + rel_x;
      state |= 0x10;
    }
    if (rel_y < 0) {
      rel_y = 0x100 + rel_y;
      state |= 0x20;
    }
    uint8_t data[] = { state, (uint8_t)rel_x, (uint8_t)rel_y, (uint8_t)rel_z };
    PushMouse4(data);
    return true;
  }

  bool InputAcceptable() {
    return !keyboard_.disable_scanning && !mouse_.disable_streaming;
  }
};

DECLARE_DEVICE(Ps2);
