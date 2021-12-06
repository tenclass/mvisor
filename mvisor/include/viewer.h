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

#ifndef _MVISOR_VIEWER_H
#define _MVISOR_VIEWER_H

#include <SDL/SDL.h>
#include "machine.h"
#include "device_manager.h"
#include "device_interface.h"
#include <mutex>
#include <deque>

class Viewer {
 public:
  Viewer(Machine* machine);
  ~Viewer();
  int MainLoop();

 private:
  void Render();
  void UpdateWindow();
  void UpdateCaption();
  void RenderPartial(const DisplayPartialBitmap* partial);
  void RenderCursor(const DisplayCursorUpdate* cursor_update);

  Machine* machine_;
  DeviceManager* device_manager_;
  DisplayInterface* display_;
  KeyboardInputInterface* keyboard_;
  SpiceAgentInterface* spice_agent_;
  SDL_Surface* screen_surface_ = nullptr;
  SDL_Cursor* cursor_ = nullptr;
  bool requested_update_window_ = false;
  uint16_t width_;
  uint16_t height_;
  uint16_t bpp_;
  bool grab_input_ = false;
  std::deque<const DisplayPartialBitmap*> partials_;
  std::deque<const DisplayCursorUpdate*> cursor_updates_;
  std::mutex mutex_;
};

#endif // _MVISOR_VIEWER_H
