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


#define RESPONSE_ACK 0xFA

#define KEYBOARD_IRQ 1
#define MOUSE_IRQ 12

/* Reference: https://wiki.osdev.org/%228042%22_PS/2_Controller
 */

class Keyboard : public Device, public KeyboardInputInterface {
 private:
  std::deque<uint8_t> keyboard_queue_;
  std::deque<uint8_t> mouse_queue_;
  std::mutex mutex_;

  uint8_t status_;
  uint8_t mode_;
  uint8_t last_command_;
  uint8_t output_data_;
  int raised_irq_;
  bool output_data_is_read_;

  uint8_t mouse_button_state_;
  uint8_t mouse_resolution_;
  uint8_t mouse_sample_rate_;
  uint8_t mouse_command_;
  uint8_t mouse_scaling_;
  uint8_t mouse_id_;
  uint8_t mouse_stream_mode_;
  bool mouse_disable_streaming_;

  // Mouse relative X, Y
  int mouse_dx_, mouse_dy_;
  
  uint8_t keyboard_scancode_set_;
  bool keyboard_disable_scanning_;

  
  void Reset() {
    status_ = STATUS_KEYLOCK | STATUS_COMMAND;
    mode_ = 5;
    raised_irq_ = -1;
    output_data_is_read_ = true;
    
    ResetKeyboard();
    ResetMouse();
  }

  void ResetKeyboard() {
    keyboard_queue_.clear();
    keyboard_scancode_set_ = 2;
    keyboard_disable_scanning_ = false;
  }

  void ResetMouse() {
    mouse_queue_.clear();
    mouse_command_ = 0;
    mouse_resolution_ = 4;
    mouse_sample_rate_ = 100;
    mouse_scaling_ = 1;
    mouse_dx_ = mouse_dy_ = 0;
    mouse_disable_streaming_ = false;
    mouse_stream_mode_ = 1;
    /* 
     * https://wiki.osdev.org/PS/2_Mouse
     * 0: Normal 2 buttons mouse
     * 3: Use 3 buttons
     * 5: Use 5 buttons
     */
    mouse_id_ = 3;
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
      manager_->SetIrq(irq, 0);
      manager_->SetIrq(irq, 1);
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
      manager_->SetIrq(raised_irq_, 0);
      raised_irq_ = -1;
    }
    
    uint8_t ret = output_data_;
    output_data_is_read_ = true;
    FillOutputData();
    return ret;
  }

  void WriteMouseCommand(uint8_t data) {
    switch (mouse_command_)
    {
    case 0xE8: // resolution
      mouse_resolution_ = data;
      mouse_command_ = 0;
      PushMouse(RESPONSE_ACK);
      break;
    case 0xF3: // sample rate
      mouse_sample_rate_ = data;
      mouse_command_ = 0;
      PushMouse(RESPONSE_ACK);
      break;
    case 0:
      switch (data)
      {
      case 0xE6:
        mouse_scaling_ = 1;
        PushMouse(RESPONSE_ACK);
        break;
      case 0xE7:
        mouse_scaling_ = 2;
        PushMouse(RESPONSE_ACK);
        break;
      case 0xE8: // set resolution
        mouse_command_ = data;
        PushMouse(RESPONSE_ACK);
        break;
      case 0xE9: // send status
        PushMouse(RESPONSE_ACK);
        PushMouse(mouse_stream_mode_ << 6 | (!mouse_disable_streaming_ << 5) | (!mouse_scaling_ << 4) | mouse_button_state_);
        PushMouse(mouse_resolution_);
        PushMouse(mouse_sample_rate_);
        break;
      case 0xEA: // set stream mode
        MV_ASSERT(data == 1);
        PushMouse(RESPONSE_ACK);
        break;
      case 0xF2: // get mouse ID
        PushMouse(RESPONSE_ACK);
        PushMouse(mouse_id_);
        break;
      case 0xF3: // set sample rate
        mouse_command_ = data;
        PushMouse(RESPONSE_ACK);
        break;
      case 0xF4: // enable mouse
        mouse_disable_streaming_ = false;
        PushMouse(RESPONSE_ACK);
        break;
      case 0xF5: // disable mouse
        mouse_disable_streaming_ = true;
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
        MV_PANIC("unhandled mouse command = 0x%x", data);
        break;
      }
      break;
    default:
      MV_PANIC("unhandled mouse command = 0x%x", mouse_command_);
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
    case 0xF3: // set typematic rate
      last_command_ = data;
      PushKeyboard(RESPONSE_ACK);
    case 0xF4: // enable keyboard scanning
      keyboard_disable_scanning_ = false;
      PushKeyboard(RESPONSE_ACK);
      break;
    case 0xF5: // disable keyboard scanning
      keyboard_disable_scanning_ = true;
      PushKeyboard(RESPONSE_ACK);
      break;
    case 0xFF: // reset keyboard
      ResetKeyboard();
      PushKeyboard(RESPONSE_ACK);
      PushKeyboard(0xAA);
      break;
    default:
      MV_PANIC("unhandled ps2 command=0x%x", data);
      break;
    }
  }

  void WriteCommandPort(uint8_t command) {
    status_ &= ~STATUS_COMMAND;

    switch (command)
    {
    case 0x20: // read 
      PushKeyboard(mode_ | (status_ & STATUS_SYSFLAG));
      break;
    case 0x60: // control mode
      status_ |= STATUS_COMMAND;
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
      status_ |= STATUS_COMMAND;
      last_command_ = command;
      break;
    case 0xFE ... 0xFF: // pulse output line
      if ((command & 1) == 0) {
        MV_LOG("system reset");
        manager_->machine()->Reset();
      }
      break;
    default:
      MV_PANIC("unhandled command=0x%x", command);
      break;
    }
  }

  void WriteDataPort(uint8_t data) {
    status_ &= STATUS_COMMAND;

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
    case 0xD1: // controller output gate
      /* Windows wrote 0xDF, not supported yet */
      break;
    case 0xD4: // mouse status
      WriteMouseCommand(data);
      break;
    case 0xED: // set leds
      PushKeyboard(RESPONSE_ACK);
      MV_LOG("set leds %d", data);
      break;
    case 0xF0: // keyboard scancode set
      PushKeyboard(RESPONSE_ACK);
      if (data == 0) {
        PushKeyboard(keyboard_scancode_set_);
      } else {
        keyboard_scancode_set_ = data;
        MV_ASSERT(keyboard_scancode_set_ == 2);
      }
      break;
    case 0xF3: // set typematic rate
      PushKeyboard(RESPONSE_ACK);
      break;
    default:
      MV_PANIC("unhandled command=0x%x data=0x%x", command, data);
      break;
    }
  }


 public:
  Keyboard() {
    Reset();

    AddIoResource(kIoResourceTypePio, 0x92, 1, "A20 Gate");
    AddIoResource(kIoResourceTypePio, 0x60, 1, "PS2 Data");
    AddIoResource(kIoResourceTypePio, 0x64, 1, "PS2 Command");
  }

  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
    std::lock_guard<std::mutex> lock(mutex_);
    MV_ASSERT(size == 1);

    switch (ir.base)
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
      *data = 2;
      break;
    }
    // MV_LOG("read %x %x", ir.base, *data);
  }


  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
    std::lock_guard<std::mutex> lock(mutex_);
    MV_ASSERT(size == 1);
    // MV_LOG("write %x %x", ir.base, *data);

    if (ir.base == 0x64) { // command port
      WriteCommandPort(*data);
    } else if (ir.base == 0x60) { // data port
      WriteDataPort(*data);
    }
  }

  void QueueKeyboardEvent(uint8_t scancode[10]) {
    if (keyboard_disable_scanning_) {
      return;
    }
  
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < 10 && scancode[i]; i++) {
      PushKeyboard(scancode[i]);
    }
  }

  void QueueMouseEvent(uint8_t button_state, int rel_x, int rel_y, int rel_z) {
    if (mouse_disable_streaming_) {
      return;
    }
  
    std::lock_guard<std::mutex> lock(mutex_);
    uint8_t state = mouse_button_state_ = button_state;
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
  }

  bool CanAcceptInput() {
    return !keyboard_disable_scanning_ && !mouse_disable_streaming_;
  }
};

DECLARE_DEVICE(Keyboard);
