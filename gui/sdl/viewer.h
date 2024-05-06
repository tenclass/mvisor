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
  int  width = 0;
  int  height = 0;
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
  void RenderSurface(const DisplayPartialBitmap* partial);
  void RenderCursor(const DisplayMouseCursor* cursor_update);
  void HandleEvent(const SDL_Event& event);
  void RegisterKeyboardShortcuts();
  PointerInputInterface* GetActivePointer();
  void SendPointerEvent();
  void SendResizerEvent();
  void OnPlayback(PlaybackState state, const std::string& data);
  void OnClipboardFromGuest(const ClipboardData& clipboard_data);
  void OnSerialPortEvent(SerialPortInterface* port, SerialPortEvent event, const std::string& data);
  void Schedule(VoidCallback callback);

  Machine* machine_;
  ClipboardInterface* clipboard_;
  std::string clipboard_data_;
  DisplayInterface* display_;
  PlaybackInterface* playback_;
  KeyboardInputInterface* keyboard_;
  std::vector<PointerInputInterface*> pointers_;
  std::vector<DisplayResizeInterface*> resizers_;
  PlaybackFormat playback_format_;

  SDL_Window* window_ = nullptr;
  SDL_Renderer* renderer_ = nullptr;
  SDL_Surface* screen_surface_ = nullptr;
  SDL_Palette* palette_ = nullptr;
  SDL_Cursor* cursor_ = nullptr;

  std::mutex mutex_;
  std::deque<VoidCallback> tasks_;
  bool cursor_visible_ = false;
  uint64_t cursor_shape_id_ = 0;

  int width_, height_, bpp_, stride_;

  PointerEvent pointer_state_;
  PendingResize pending_resize_;

  bool pcm_playback_error_ = false;
  snd_pcm_t* pcm_playback_ = nullptr;
  std::unordered_map<int, VoidCallback> keyboard_shortcuts_;
};

#endif // _MVISOR_VIEWER_H
