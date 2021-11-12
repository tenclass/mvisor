#ifndef _MVISOR_DEVICES_KEYBOARD_H
#define _MVISOR_DEVICES_KEYBOARD_H

#include <deque>
#include "devices/device.h"


class KeyboardDevice : public Device {
 public:
  KeyboardDevice();
  void Reset();
  void ResetKeyboard();
  void ResetMouse();

  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);

  void QueueKeyboardEvent(uint8_t scancode);

 private:
  void PushKeyboard(uint8_t data);
  void PushMouse(uint8_t data);

  uint8_t ReadData();
  void RaiseIrq(int irq);

  void WriteCommandPort(uint8_t command);
  void WriteDataPort(uint8_t data);
  void WritePs2Command(uint8_t data);
  void WriteMouseCommand(uint8_t data);
  void FillOutputData();

  std::deque<uint8_t> keyboard_queue_;
  std::deque<uint8_t> mouse_queue_;

  uint8_t status_;
  uint8_t mode_;
  uint8_t last_command_;
  uint8_t output_data_;
  int raised_irq_;
  bool data_read_;

  uint8_t mouse_button_state_;
  uint8_t mouse_resolution_;
  uint8_t mouse_sample_rate_;
  uint8_t mouse_command_;
  uint8_t mouse_scaling_;
  uint8_t mouse_id_;
  bool mouse_disable_streaming_;

  // Mouse relative X, Y
  int mouse_dx_, mouse_dy_;
  
  uint8_t keyboard_scancode_set_;
  bool keyboard_disable_scanning_;
};

#endif // _MVISOR_DEVICES_KEYBOARD_H

