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
  SDL_Init(SDL_INIT_VIDEO);
}

Viewer::~Viewer() {
  DestroyWindow();
  SDL_Quit();
}

void Viewer::DestroyWindow() {
  if (screen_texture_) {
    if (palette_) {
      SDL_FreePalette(palette_);
      palette_ = nullptr;
    }
    if (cursor_) {
      SDL_FreeCursor(cursor_);
      cursor_ = nullptr;
    }
    if (server_cursor_.texture) {
      SDL_DestroyTexture(server_cursor_.texture);
      server_cursor_.texture = nullptr;
    }
    SDL_DestroyTexture(screen_texture_);
    SDL_DestroyWindow(window_);
    SDL_DestroyRenderer(renderer_);
  }
}

void Viewer::CreateWindow() {
  uint16_t w, h, bpp;
  display_->GetDisplayMode(&w, &h, &bpp);

  width_ = w;
  height_ = h;
  bpp_ = bpp;
  int x = SDL_WINDOWPOS_UNDEFINED, y = SDL_WINDOWPOS_UNDEFINED;
  window_ = SDL_CreateWindow("MVisor", x, y, width_, height_, SDL_WINDOW_RESIZABLE);
  renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
  MV_ASSERT(renderer_);
  
  switch (bpp_)
  {
  case 32:
    screen_texture_ = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_BGRA32,
      SDL_TEXTUREACCESS_STREAMING, width_, height_);
    break;
  case 24:
    screen_texture_ = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_BGR24,
      SDL_TEXTUREACCESS_STREAMING, width_, height_);
    break;
  case 16:
    screen_texture_ = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_BGR565,
      SDL_TEXTUREACCESS_STREAMING, width_, height_);
    break;
  case 8:
    screen_texture_ = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_BGRA32,
      SDL_TEXTUREACCESS_STREAMING, width_, height_);
    break;
  default:
    MV_PANIC("unsupported video bpp=%d", bpp_);
  }
  MV_ASSERT(screen_texture_);
  if (bpp_ == 8) {
    palette_ = SDL_AllocPalette(256);
    SDL_Color colors[256];
    const uint8_t* pallete = display_->GetPallete();
    for (int i = 0; i < 256; i++) {
      colors[i].r = *pallete++ << 2;
      colors[i].g = *pallete++ << 2;
      colors[i].b = *pallete++ << 2;
    }
    SDL_SetPaletteColors(palette_, colors, 0, 256);
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
  SDL_SetWindowTitle(window_, title);
}

void Viewer::RenderPartial(const DisplayPartialBitmap* partial) {
  uint8_t* dst;
  int dst_stride;
  SDL_LockTexture(screen_texture_, NULL, (void**)&dst, &dst_stride);

  if (partial->flip) {
    dst += dst_stride * (partial->y + partial->height - 1) + partial->x * (bpp_ >> 3);
    dst_stride = -dst_stride;
  } else {
    dst += dst_stride * partial->y + partial->x * (bpp_ >> 3);
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
  SDL_UnlockTexture(screen_texture_);
}

void Viewer::RenderSurface(const DisplayPartialBitmap* partial) {
  MV_ASSERT(partial->width == width_ && partial->height == height_);
  MV_ASSERT(partial->vector.size() == 1 && partial->vector[0].size == width_ * height_);
  auto surface_8bit = SDL_CreateRGBSurfaceWithFormatFrom(partial->vector[0].data,
    partial->width, partial->height, bpp_, partial->stride, SDL_PIXELFORMAT_INDEX8);
  SDL_SetSurfacePalette(surface_8bit, palette_);
  auto surface_32bit = SDL_ConvertSurfaceFormat(surface_8bit, SDL_PIXELFORMAT_BGRA32, 0);
  SDL_UpdateTexture(screen_texture_, nullptr, surface_32bit->pixels, surface_32bit->pitch);
  SDL_FreeSurface(surface_32bit);
  SDL_FreeSurface(surface_8bit);
}

void Viewer::RenderCursor(const DisplayCursorUpdate* cursor_update) {
  if (cursor_update->command == kDisplayCursorUpdateMove) {
    server_cursor_.x = cursor_update->move.x;
    server_cursor_.y = cursor_update->move.y;
  } else if (cursor_update->command == kDisplayCursorUpdateSet) {
    if (cursor_) {
      SDL_FreeCursor(cursor_);
      cursor_ = nullptr;
    }
    if (!grab_input_) {
      SDL_ShowCursor(SDL_ENABLE);
    }
    auto set = cursor_update->set;
    uint32_t stride = SPICE_ALIGN(set.width, 8) >> 3;
    if (set.type == SPICE_CURSOR_TYPE_MONO) {
      uint8_t mask[4 * 100] = { 0 };
      cursor_ = SDL_CreateCursor(set.data + stride * set.height, mask,
        set.width, set.height, set.hotspot_x, set.hotspot_y);
      SDL_SetCursor(cursor_);
    } else {
      auto surface = SDL_CreateRGBSurfaceWithFormatFrom(set.data, set.width, set.height,
        32, set.width * 4, SDL_PIXELFORMAT_BGRA32);
      cursor_ = SDL_CreateColorCursor(surface, set.hotspot_x, set.hotspot_y);
      SDL_SetCursor(cursor_);
      /* Store the cursor texture for server cursor */
      if (server_cursor_.texture) {
        SDL_DestroyTexture(server_cursor_.texture);
      }
      server_cursor_.texture = SDL_CreateTextureFromSurface(renderer_, surface);
      SDL_FreeSurface(surface);
    }
    /* Used when simulate cursor */
    server_cursor_.visible = set.visible;
    server_cursor_.x = set.x;
    server_cursor_.y = set.y;
    server_cursor_.width = set.width;
    server_cursor_.height = set.height;
    server_cursor_.hotspot_x = set.hotspot_x;
    server_cursor_.hotspot_y = set.hotspot_y;
  } else if (cursor_update->command == kDisplayCursorUpdateHide) {
    SDL_ShowCursor(SDL_DISABLE);
    server_cursor_.visible = false;
  }
}

void Viewer::Render() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (requested_update_window_) {
    requested_update_window_ = false;
    DestroyWindow();
    CreateWindow();
    for (auto partial : partials_) {
      partial->release();
    }
    partials_.clear();
  }
  bool redraw = false;
  while (!cursor_updates_.empty()) {
    auto cursor_update = cursor_updates_.front();
    cursor_updates_.pop_front();
    RenderCursor(cursor_update);
    cursor_update->release();
    if (grab_input_) {
      redraw = true;
    }
  }
  if (screen_texture_ && !partials_.empty()) {
    while (!partials_.empty()) {
      auto partial = partials_.front();
      partials_.pop_front();
      if (bpp_ == 8) {
        RenderSurface(partial);
      } else {
        RenderPartial(partial);
      }
      partial->release();
    }
    redraw = true;
  }

  if (redraw) {
    SDL_RenderCopy(renderer_, screen_texture_, nullptr, nullptr);
    if (grab_input_) {
      SDL_Rect rect = {
        .x = server_cursor_.x,
        .y = server_cursor_.y,
        .w = server_cursor_.width,
        .h = server_cursor_.height
      };
      SDL_RenderCopy(renderer_, server_cursor_.texture, nullptr, &rect);
    }
    SDL_RenderPresent(renderer_);
  }
}

void Viewer::LookupDevices() {
  keyboard_ = dynamic_cast<KeyboardInputInterface*>(device_manager_->LookupDeviceByName("Keyboard"));
  spice_agent_ = dynamic_cast<SpiceAgentInterface*>(device_manager_->LookupDeviceByName("SpiceAgent"));
  display_ = dynamic_cast<DisplayInterface*>(device_manager_->LookupDeviceByName("Qxl"));
  if (display_ == nullptr) {
    display_ = dynamic_cast<DisplayInterface*>(device_manager_->LookupDeviceByName("Vga"));
  }
  MV_ASSERT(keyboard_ && display_);

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
}

/* Reference about SDL-2:
 * https://wiki.libsdl.org/APIByCategory
 */
int Viewer::MainLoop() {
  SetThreadName("viewer");
  LookupDevices();

  auto frame_interval_us = std::chrono::microseconds(1000000 / 30);
  // Loop until all vcpu exits
  while (machine_->IsValid()) {
    auto frame_start_time = std::chrono::steady_clock::now();
    Render();

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      HandleEvent(event);
    }

    /* Check viewer window resize */
    if (pending_resize_.triggered && frame_start_time - pending_resize_.time >= std::chrono::milliseconds(300)) {
      pending_resize_.triggered = false;
      spice_agent_->Resize(pending_resize_.width, pending_resize_.height);
    }

    /* Keep display FPS */
    auto frame_cost_us = std::chrono::steady_clock::now() - frame_start_time;
    if (frame_cost_us < frame_interval_us) {
      std::this_thread::sleep_for(frame_interval_us - frame_cost_us);
    }
  }
  return 0;
}

void Viewer::HandleEvent(const SDL_Event& event) {
  std::lock_guard<std::mutex> lock(mutex_);
  uint8_t transcoded[10] = { 0 };
  switch (event.type)
  {
  case SDL_KEYDOWN:
  case SDL_KEYUP:
    /* Type ESCAPE to exit mouse grab mode */
    if (grab_input_ && event.key.keysym.sym == SDLK_ESCAPE) {
      grab_input_ = false;
      SDL_SetWindowGrab(window_, SDL_FALSE);
      SDL_ShowCursor(SDL_ENABLE);
      UpdateCaption();
      break;
    }
    if (TranslateScancode(event.key.keysym.scancode, event.type == SDL_KEYDOWN, transcoded)) {
      keyboard_->QueueKeyboardEvent(transcoded);
    }
    break;
  case SDL_MOUSEWHEEL: {
    int x, y;
    SDL_GetMouseState(&x, &y);
    if (event.wheel.y > 0) {
      spice_agent_->QueuePointerEvent(mouse_buttons_ | (1 << SPICE_MOUSE_BUTTON_UP), x, y);
      spice_agent_->QueuePointerEvent(mouse_buttons_, x, y);
    } else if (event.wheel.y < 0) {
      spice_agent_->QueuePointerEvent(mouse_buttons_ | (1 << SPICE_MOUSE_BUTTON_DOWN), x, y);
      spice_agent_->QueuePointerEvent(mouse_buttons_, x, y);
    }
    break;
  }
  case SDL_MOUSEBUTTONDOWN:
    /* If pointer device is not available, try to grab input and use PS/2 input */
    if (!grab_input_ && spice_agent_ && !spice_agent_->CanAcceptInput()) {
      grab_input_ = true;
      SDL_SetWindowGrab(window_, SDL_TRUE);
      SDL_ShowCursor(SDL_DISABLE);
      UpdateCaption();
    }
    /* fall through */
  case SDL_MOUSEBUTTONUP:
    if (event.button.state) {
      mouse_buttons_ |= (1 << event.button.button);
    } else {
      mouse_buttons_ &= ~(1 << event.button.button);
    }
    /* fall through */
  case SDL_MOUSEMOTION:
    if (spice_agent_ && spice_agent_->CanAcceptInput()) {
      int x, y;
      SDL_GetMouseState(&x, &y);
      spice_agent_->QueuePointerEvent(mouse_buttons_, x, y);
    } else if (grab_input_) {
      /* swap the middle button and right button bit of input state */
      uint8_t ps2_state = ((mouse_buttons_ & 2) >> 1) | (mouse_buttons_ & 4) | ((mouse_buttons_ & 8) >> 2);
      if (event.type == SDL_MOUSEMOTION) {
        /* multiply by 2 to prevent host mouse go out of window */
        keyboard_->QueueMouseEvent(ps2_state, event.motion.xrel * 2, event.motion.yrel * 2, 0);
      } else {
        keyboard_->QueueMouseEvent(ps2_state, 0, 0, 0);
      }
    }
    break;
  case SDL_WINDOWEVENT:
    switch (event.window.event)
    {
    case SDL_WINDOWEVENT_RESIZED:
      if (spice_agent_) {
        pending_resize_.triggered = true;
        pending_resize_.width = event.window.data1;
        pending_resize_.height = event.window.data2;
        pending_resize_.time = std::chrono::steady_clock::now();
      }
      break;
    }
    break;
  case SDL_QUIT:
    machine_->Quit();
    break;
  }
}
