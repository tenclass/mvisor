#ifndef _MVISOR_DEVICE_INTERFACES_H
#define _MVISOR_DEVICE_INTERFACES_H

class KeyboardInterface {
 public:
  virtual void QueueKeyboardEvent(uint8_t scancode) = 0;
};

enum DisplayMode {
  kDisplayTextMode,
  kDisplayVgaMode,
  kDisplayVbeMode
};

class DisplayInterface {
 public:
  virtual void GetCursorLocation(uint8_t* x, uint8_t* y, uint8_t* sel_start, uint8_t* sel_end) = 0;
  virtual uint8_t* GetVRamHostAddress() = 0;
  virtual void GetDisplayMode(DisplayMode *mode, uint16_t* w, uint16_t* h, uint16_t* b) = 0;
  virtual const uint8_t* GetPallete() const = 0;
};


#endif // _MVISOR_DEVICE_INTERFACES_H
