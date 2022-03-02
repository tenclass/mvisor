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

#include <SDL2/SDL.h>
#include "machine.h"
#include "device_manager.h"
#include "device_interface.h"
#include <mutex>
#include <deque>

struct SimulateCursor {
  bool visible;
  int x;
  int y;
  int width;
  int height;
  int hotspot_x;
  int hotspot_y;
  SDL_Texture* texture = nullptr;
};

struct PendingResize {
  bool triggered = false;
  int width;
  int height;
  std::chrono::steady_clock::time_point time;
};

class Viewer {
 public:
  Viewer(Machine* machine);
  ~Viewer();
  int MainLoop();

 private:
  void LookupDevices();
  void DestroyWindow();
  void Render();
  void CreateWindow();
  void UpdateCaption();
  void RenderPartial(const DisplayPartialBitmap* partial);
  void RenderSurface(const DisplayPartialBitmap* partial);
  void RenderCursor(const DisplayCursorUpdate* cursor_update);
  void HandleEvent(const SDL_Event& event);
  PointerInputInterface* GetActivePointer();

  Machine* machine_;
  DeviceManager* device_manager_;
  DisplayInterface* display_;
  KeyboardInputInterface* keyboard_;
  SpiceAgentInterface* spice_agent_;
  std::vector<PointerInputInterface*> pointers_;

  SDL_Window* window_ = nullptr;
  SDL_Renderer* renderer_ = nullptr;
  SDL_Texture* screen_texture_ = nullptr;
  SDL_Palette* palette_ = nullptr;
  SDL_Cursor* cursor_ = nullptr;

  SimulateCursor server_cursor_;
  std::deque<const DisplayPartialBitmap*> partials_;
  std::deque<const DisplayCursorUpdate*> cursor_updates_;
  std::mutex mutex_;
  bool requested_update_window_ = false;

  uint16_t width_;
  uint16_t height_;
  uint16_t bpp_;

  PointerEvent pointer_state_ = { 0 };
  PendingResize pending_resize_;
};

#endif // _MVISOR_VIEWER_H
