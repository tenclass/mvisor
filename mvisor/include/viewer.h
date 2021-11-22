#ifndef _MVISOR_VIEWER_H
#define _MVISOR_VIEWER_H

#include <SDL/SDL.h>
#include "machine.h"
#include "device_manager.h"
#include "device_interface.h"

class Viewer {
 public:
  Viewer(Machine* machine);
  ~Viewer();
  int MainLoop();

 private:
  void DrawTextMode();
  void DrawGraphicsMode();
  void DrawTextCursor();
  void DrawCharacter(int x, int y, int character, int attribute, uint8_t* font);
  void UpdateWindow();
  void AcquireDisplayFrame();
  void UpdateCaption();

  Machine* machine_;
  DeviceManager* device_manager_;
  DisplayInterface* display_;
  SDL_Surface* screen_surface_ = nullptr;
  bool requested_update_window_ = false;
  DisplayMode mode_;
  uint16_t width_;
  uint16_t height_;
  uint16_t bpp_;
  bool grab_input_ = false;
};

#endif // _MVISOR_VIEWER_H
