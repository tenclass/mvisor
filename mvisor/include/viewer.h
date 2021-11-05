#ifndef _MVISOR_VIEWER_H
#define _MVISOR_VIEWER_H

#include <SDL/SDL.h>
#include "machine.h"
#include "device_manager.h"
#include "devices/vga.h"

class Viewer {
 public:
  Viewer(Machine* machine);
  ~Viewer();
  int MainLoop();

 private:
  void DrawTextMode();
  void DrawGraphicMode();
  void DrawTextCursor();
  void DrawCharacter(int x, int y, int character, int attribute);

  Machine* machine_;
  DeviceManager* device_manager_;
  VgaDevice* vga_device_;
  SDL_Surface* screen_surface_ = nullptr;
};

#endif // _MVISOR_VIEWER_H
