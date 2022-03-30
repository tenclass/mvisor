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
#include <alsa/asoundlib.h>

#include <mutex>
#include <deque>

#include "machine.h"
#include "device_manager.h"
#include "device_interface.h"


struct PendingResize {
  bool triggered = false;
  int width;
  int height;
  std::chrono::steady_clock::time_point time;
};

struct PlaybackFormat {
  uint format;
  uint channels;
  uint frequency;
  uint interval_ms;
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
  void RenderCursor(const DisplayMouseCursor* cursor_update);
  void HandleEvent(const SDL_Event& event);
  PointerInputInterface* GetActivePointer();
  void SendPointerEvent();
  void OnPlayback(PlaybackState state, struct iovec& iov);

  Machine* machine_;
  DisplayInterface* display_;
  PlaybackInterface* playback_;
  KeyboardInputInterface* keyboard_;
  std::vector<PointerInputInterface*> pointers_;
  std::vector<DisplayResizeInterface*> resizers_;
  PlaybackFormat playback_format_;

  SDL_Window* window_ = nullptr;
  SDL_Renderer* renderer_ = nullptr;
  SDL_Texture* screen_texture_ = nullptr;
  SDL_Palette* palette_ = nullptr;
  SDL_Cursor* cursor_ = nullptr;

  std::mutex mutex_;
  bool requested_update_window_ = false;
  bool cursor_visible_ = false;
  uint64_t cursor_shape_id_ = 0;

  uint width_, height_, bpp_, stride_;

  PointerEvent pointer_state_ = { 0 };
  PendingResize pending_resize_;

  bool pcm_playback_error_ = false;
  snd_pcm_t* pcm_playback_ = nullptr;
};

#endif // _MVISOR_VIEWER_H
