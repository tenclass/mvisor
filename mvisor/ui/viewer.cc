/* 
 * MVisor - SDL Viewer
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

#include "viewer.h"
#include <unistd.h>
#include <chrono>
#include "logger.h"
#include "keymap.h"
#include "spice/enums.h"

Viewer::Viewer(Machine* machine) : machine_(machine) {
  device_manager_ = machine_->device_manager();
}

Viewer::~Viewer() {
}

void Viewer::UpdateWindow() {
  uint16_t w, h, bpp;
  display_->GetDisplayMode(&w, &h, &bpp);

  width_ = w;
  height_ = h;
  bpp_ = bpp;

  if (screen_surface_) {
    SDL_FreeSurface(screen_surface_);
  }
  screen_surface_ = SDL_SetVideoMode(width_, height_, bpp_, SDL_SWSURFACE | SDL_DOUBLEBUF | SDL_RESIZABLE);
  
  if (bpp_ <= 8) {
    SDL_Color colors[256];
    const uint8_t* pallete = display_->GetPallete();
    for (int i = 0; i < 256; i++) {
      colors[i].r = *pallete++ << 2;
      colors[i].g = *pallete++ << 2;
      colors[i].b = *pallete++ << 2;
    }
    SDL_SetPalette(screen_surface_, SDL_LOGPAL | SDL_PHYSPAL, colors, 0, 256);
  }
  UpdateCaption();
}

void Viewer::UpdateCaption() {
  char title[100];
  if (grab_input_) {
    sprintf(title, "MVisor - Press [ESC] to release mouse");
  } else {
    sprintf(title, "MVisor - A mini x86 hypervisor - %dx%dx%d", width_, height_, bpp_);
  }
  SDL_WM_SetCaption(title, "MVisor");
}

void Viewer::RenderPartial(const DisplayPartialBitmap* partial) {
  SDL_LockSurface(screen_surface_);

  uint8_t* dst = (uint8_t*)screen_surface_->pixels;
  int dst_stride = screen_surface_->pitch;
  if (partial->flip) {
    dst += screen_surface_->pitch * (partial->y + partial->height - 1) + partial->x * (bpp_ >> 3);
    dst_stride = -dst_stride;
  } else {
    dst += screen_surface_->pitch * partial->y + partial->x * (bpp_ >> 3);
  }
  int lines = partial->height;
  size_t src_index = 0;
  while (lines > 0) {
    MV_ASSERT(src_index < partial->vector.size());
    uint8_t* src = partial->vector[src_index].data;
    auto copy_lines = partial->vector[src_index].size / partial->stride;
    while (copy_lines > 0) {
      memcpy(dst, src, partial->stride);
      src += partial->stride;
      dst += dst_stride;
      --copy_lines;
      --lines;
    }
    ++src_index;
  }
  SDL_UnlockSurface(screen_surface_);
}

void Viewer::RenderCursor(const DisplayCursorUpdate* cursor_update) {
  if (cursor_update->command == kDisplayCursorUpdateSet) {
    if (cursor_) {
      SDL_FreeCursor(cursor_);
      cursor_ = nullptr;
    }
    SDL_ShowCursor(SDL_ENABLE);
    auto set = cursor_update->set;
    uint32_t stride = SPICE_ALIGN(set.width, 8) >> 3;
    if (set.type == SPICE_CURSOR_TYPE_MONO) {
      uint8_t mask[4 * 100] = { 0 };
      cursor_ = SDL_CreateCursor(set.data + stride * set.height, mask,
        set.width, set.height, set.hotspot_x, set.hotspot_y);
      SDL_SetCursor(cursor_);
    } else {
      uint8_t* data = new uint8_t[stride * set.height];
      uint8_t* mask = new uint8_t[stride * set.height];
      int i = -1;
      for (int row = 0; row < set.height; ++row) {
        for (int col = 0; col < set.width; ++col) {
          if (col % 8) {
            data[i] <<= 1;
            mask[i] <<= 1;
          } else {
            ++i;
            data[i] = mask[i] = 0;
          }
          uint8_t* src = &set.data[set.width * 4 * row + col * 4];
          mask[i] |= src[3] ? 1 : 0;
          if (mask[i]) {
            data[i] |= (src[0] || src[1] || src[2]) ? 1 : 0;
          }
        }
      }
      cursor_ = SDL_CreateCursor(data, mask,
        set.width, set.height, set.hotspot_x, set.hotspot_y);
      SDL_SetCursor(cursor_);
      delete[] data;
      delete[] mask;
    }
  } else if (cursor_update->command == kDisplayCursorUpdateHide) {
    SDL_ShowCursor(SDL_DISABLE);
  }
}

void Viewer::Render() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (requested_update_window_) {
    requested_update_window_ = false;
    UpdateWindow();
    for (auto partial : partials_) {
      partial->release();
    }
    partials_.clear();
  }
  if (screen_surface_ && !partials_.empty()) {
    while (!partials_.empty()) {
      auto partial = partials_.front();
      partials_.pop_front();
      RenderPartial(partial);
      partial->release();
    }
    SDL_Flip(screen_surface_);
  }
  while (!cursor_updates_.empty()) {
    auto cursor_update = cursor_updates_.front();
    cursor_updates_.pop_front();
    RenderCursor(cursor_update);
    cursor_update->release();
  }
}

/* Reference about SDL-1.2:
 * https://www.libsdl.org/release/SDL-1.2.15/docs/html/index.html
 * FXIME: Why not SDL2? SDL2 create lots of threads after calling SDL_Init, it crashes after
 * a SIGUSR is sent to the vCPU.
 */
int Viewer::MainLoop() {
  SetThreadName("viewer");
  
  SDL_Init(SDL_INIT_VIDEO);

  SDL_Event event;

  keyboard_ = dynamic_cast<KeyboardInputInterface*>(device_manager_->LookupDeviceByName("Keyboard"));
  spice_agent_ = dynamic_cast<SpiceAgentInterface*>(device_manager_->LookupDeviceByName("SpiceAgent"));
  display_ = dynamic_cast<DisplayInterface*>(device_manager_->LookupDeviceByName("Qxl"));
  if (display_ == nullptr) {
    display_ = dynamic_cast<DisplayInterface*>(device_manager_->LookupDeviceByName("Vga"));
  }
  MV_ASSERT(display_);

  display_->RegisterDisplayChangeListener([this]() {
    requested_update_window_ = true;
  });
  display_->RegisterDisplayRenderer([this](const DisplayPartialBitmap* partial) {
    std::lock_guard<std::mutex> lock(mutex_);
    partials_.push_back(partial);
  }, [this](const DisplayCursorUpdate* update) {
    std::lock_guard<std::mutex> lock(mutex_);
    cursor_updates_.push_back(update);
  });

  uint32_t mouse_buttons = 0;
  auto frame_interval_us = std::chrono::microseconds(1000000 / 30);

  bool pending_resize = false;
  int  pending_resize_w, pending_resize_h;
  std::chrono::steady_clock::time_point pending_resize_time;

  // Loop until all vcpu exits
  while (machine_->IsValid()) {
    auto frame_start_time = std::chrono::steady_clock::now();
    Render();

    while (SDL_PollEvent(&event)) {
      std::lock_guard<std::mutex> lock(mutex_);
      uint8_t transcoded[10] = { 0 };
      switch (event.type)
      {
      case SDL_KEYDOWN:
      case SDL_KEYUP:
        /* Type ESCAPE to exit mouse grab mode */
        if (grab_input_ && event.key.keysym.sym == SDLK_ESCAPE) {
          grab_input_ = false;
          SDL_WM_GrabInput(SDL_GRAB_OFF);
          SDL_ShowCursor(SDL_ENABLE);
          UpdateCaption();
          break;
        }
        if (TranslateScancode(event.key.keysym.scancode, event.type == SDL_KEYDOWN, transcoded)) {
          keyboard_->QueueKeyboardEvent(transcoded);
        }
        break;
      case SDL_MOUSEBUTTONDOWN:
        /* If pointer device is not available, try to grab input and use PS/2 input */
        if (!grab_input_ && spice_agent_ && !spice_agent_->CanAcceptInput()) {
          grab_input_ = true;
          SDL_WM_GrabInput(SDL_GRAB_ON);
          SDL_ShowCursor(SDL_DISABLE);
          UpdateCaption();
        }
        /* fall through */
      case SDL_MOUSEBUTTONUP:
        if (event.button.state) {
          mouse_buttons |= (1 << event.button.button);
        } else {
          mouse_buttons &= ~(1 << event.button.button);
        }
      case SDL_MOUSEMOTION:
        if (spice_agent_ && spice_agent_->CanAcceptInput()) {
          int x, y;
          SDL_GetMouseState(&x, &y);
          spice_agent_->QueuePointerEvent(mouse_buttons, x, y);
        } else if (grab_input_) {
          /* swap the middle button and right button bit of input state */
          uint8_t ps2_state = ((mouse_buttons & 2) >> 1) | (mouse_buttons & 4) | ((mouse_buttons & 8) >> 2);
          /* multiply by 2 to prevent host mouse go out of window */
          keyboard_->QueueMouseEvent(ps2_state, event.motion.xrel * 2, event.motion.yrel * 2, 0);
        }
        break;
      case SDL_VIDEORESIZE:
        if (spice_agent_) {
          pending_resize = true;
          pending_resize_w = event.resize.w;
          pending_resize_h = event.resize.h;
          pending_resize_time = std::chrono::steady_clock::now();
        }
        break;
      case SDL_QUIT:
        machine_->Quit();
        break;
      }
    }

    /* Check viewer window resize */
    if (pending_resize && frame_start_time - pending_resize_time >= std::chrono::milliseconds(500)) {
      pending_resize = false;
      spice_agent_->Resize(pending_resize_w, pending_resize_h);
    }

    /* Keep display FPS */
    auto frame_cost_us = std::chrono::steady_clock::now() - frame_start_time;
    if (frame_cost_us < frame_interval_us) {
      std::this_thread::sleep_for(frame_interval_us - frame_cost_us);
    }
  }
  return 0;
}
