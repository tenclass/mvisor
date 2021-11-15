#include "devices/keyboard.h"
#include <cstring>
#include "logger.h"
#include "device_manager.h"

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


KeyboardDevice::KeyboardDevice() {
  name_ = "keyboard";

  Reset();

  AddIoResource(kIoResourceTypePio, 0x92, 1);
  AddIoResource(kIoResourceTypePio, 0x60, 1);
  AddIoResource(kIoResourceTypePio, 0x64, 1);
}

void KeyboardDevice::Reset() {
  status_ = STATUS_KEYLOCK | STATUS_COMMAND;
  mode_ = 5;
  raised_irq_ = -1;
  data_read_ = true;
  
  ResetKeyboard();
  ResetMouse();
}

void KeyboardDevice::ResetKeyboard() {
  keyboard_queue_.clear();
  keyboard_scancode_set_ = 2;
  keyboard_disable_scanning_ = false;
}

void KeyboardDevice::ResetMouse() {
  mouse_queue_.clear();
  mouse_command_ = 0;
  mouse_resolution_ = 4;
  mouse_sample_rate_ = 100;
  mouse_scaling_ = 1;
  mouse_dx_ = mouse_dy_ = 0;
  mouse_disable_streaming_ = false;
}

void KeyboardDevice::RaiseIrq(int irq) {
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

void KeyboardDevice::FillOutputData() {
  if (!data_read_) {
    return;
  }

  if (!keyboard_queue_.empty()) {
    output_data_ = keyboard_queue_.front();
    keyboard_queue_.pop_front();
    data_read_ = false;
    RaiseIrq(KEYBOARD_IRQ);
  } else if (!mouse_queue_.empty()) {
    output_data_ = mouse_queue_.front();
    mouse_queue_.pop_front();
    data_read_ = false;
    RaiseIrq(MOUSE_IRQ);
  }
}


void KeyboardDevice::PushKeyboard(uint8_t data) {
  keyboard_queue_.push_back(data);
  FillOutputData();
}

void KeyboardDevice::PushMouse(uint8_t data) {
  mouse_queue_.push_back(data);
  FillOutputData();
}

uint8_t KeyboardDevice::ReadData() {
  status_ &= ~(STATUS_AUXDATA | STATUS_OFULL);

  if (raised_irq_ != -1) {
    manager_->SetIrq(raised_irq_, 0);
    raised_irq_ = -1;
  }
  
  uint8_t ret = output_data_;
  data_read_ = true;
  FillOutputData();
  return ret;
}

void KeyboardDevice::Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
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


void KeyboardDevice::Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  MV_ASSERT(size == 1);
  // MV_LOG("write %x %x", ir.base, *data);

  if (ir.base == 0x64) { // command port
    WriteCommandPort(*data);
  } else if (ir.base == 0x60) { // data port
    WriteDataPort(*data);
  }
}

void KeyboardDevice::WriteMouseCommand(uint8_t data) {
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

void KeyboardDevice::WritePs2Command(uint8_t data) {
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

void KeyboardDevice::WriteCommandPort(uint8_t command) {
  status_ &= ~STATUS_COMMAND;

  switch (command)
  {
  case 0x60: // control mode
    status_ |= STATUS_COMMAND;
    last_command_ = command;
    break;
  case 0xA7: // disable mouse
    mode_ |= MODE_AUX_DISABLED;
    break;
  case 0xA8: // enable mouse
    mode_ &= ~MODE_AUX_DISABLED;
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
    break;
  case 0xD1 ... 0xD4: // outport utilities
    status_ |= STATUS_COMMAND;
    last_command_ = command;
    break;
  case 0xFE ... 0xFF: // pulse output line
    if ((command & 1) == 0) {
      MV_PANIC("system reset");
    }
    break;
  default:
    MV_PANIC("unhandled command=0x%x", command);
    break;
  }
}

void KeyboardDevice::WriteDataPort(uint8_t data) {
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
  case 0xF0: // keyboard scancode set
    PushKeyboard(RESPONSE_ACK);
    if (data == 0) {
      PushKeyboard(keyboard_scancode_set_);
    } else {
      keyboard_scancode_set_ = data;
      MV_ASSERT(keyboard_scancode_set_ == 2);
    }
    break;
  default:
    MV_PANIC("unhandled command=0x%x data=0x%x", command, data);
    break;
  }
}

void KeyboardDevice::QueueKeyboardEvent(uint8_t scancode) {
  if (!keyboard_disable_scanning_) {
    PushKeyboard(scancode);
  }
}

