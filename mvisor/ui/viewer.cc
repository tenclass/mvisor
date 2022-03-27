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
  SDL_Init(SDL_INIT_VIDEO);
  LookupDevices();
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
    SDL_DestroyTexture(screen_texture_);
    SDL_DestroyRenderer(renderer_);
    SDL_DestroyWindow(window_);
    screen_texture_ = nullptr;
    renderer_ = nullptr;
    window_ = nullptr;
  }
}

void Viewer::CreateWindow() {
  display_->GetDisplayMode(&width_, &height_, &bpp_, &stride_);
  MV_ASSERT(width_ && height_ && bpp_ && stride_);
  pointer_state_.screen_width = width_;
  pointer_state_.screen_height = height_;
  int x = SDL_WINDOWPOS_UNDEFINED, y = SDL_WINDOWPOS_UNDEFINED;
  window_ = SDL_CreateWindow("MVisor", x, y, width_, height_, SDL_WINDOW_RESIZABLE);
  renderer_ = SDL_CreateRenderer(window_, -1, 0);
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
  sprintf(title, "MVisor - A mini x86 hypervisor - %dx%dx%d", width_, height_, bpp_);
  SDL_SetWindowTitle(window_, title);
}

void Viewer::RenderPartial(const DisplayPartialBitmap* partial) {
  uint8_t* dst;
  int dst_stride;
  SDL_LockTexture(screen_texture_, NULL, (void**)&dst, &dst_stride);
  uint8_t* dst_end = dst + dst_stride * height_;

  if (partial->flip) {
    dst += dst_stride * (partial->y + partial->height - 1) + partial->x * (bpp_ >> 3);
    dst_stride = -dst_stride;
  } else {
    dst += dst_stride * partial->y + partial->x * (bpp_ >> 3);
  }
  int linesize = partial->width * (bpp_ >> 3);
  size_t lines = partial->height;
  size_t src_index = 0;
  while (lines > 0 && src_index < partial->vector.size()) {
    auto src = (uint8_t*)partial->vector[src_index].iov_base;
    auto copy_lines = partial->vector[src_index].iov_len / partial->stride;
    while (copy_lines > 0 && lines > 0) {
      MV_ASSERT(dst + linesize <= dst_end);
      memcpy(dst, src, linesize);
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
  MV_ASSERT(partial->vector.size() == 1 && partial->vector[0].iov_len == width_ * height_);
  auto surface_8bit = SDL_CreateRGBSurfaceWithFormatFrom(partial->vector[0].iov_base,
    partial->width, partial->height, bpp_, partial->stride, SDL_PIXELFORMAT_INDEX8);
  SDL_SetSurfacePalette(surface_8bit, palette_);
  auto surface_32bit = SDL_ConvertSurfaceFormat(surface_8bit, SDL_PIXELFORMAT_BGRA32, 0);
  SDL_UpdateTexture(screen_texture_, nullptr, surface_32bit->pixels, surface_32bit->pitch);
  SDL_FreeSurface(surface_32bit);
  SDL_FreeSurface(surface_8bit);
}

void Viewer::RenderCursor(const DisplayMouseCursor* cursor_update) {
  if (cursor_update->visible) {
    if (cursor_update->shape.id == cursor_shape_id_) {
      return;
    }
    if (cursor_) {
      SDL_FreeCursor(cursor_);
      cursor_ = nullptr;
    }
    SDL_ShowCursor(SDL_ENABLE);
    auto& shape = cursor_update->shape;
    uint32_t stride = SPICE_ALIGN(shape.width, 8) >> 3;

    size_t size = 0;
    for (auto& iov : shape.vector)
      size += iov.iov_len;
    auto shape_data = new uint8_t[size];
    auto ptr = shape_data;
    for (auto& iov : shape.vector) {
      memcpy(ptr, iov.iov_base, iov.iov_len);
      ptr += iov.iov_len;
    }
    if (shape.type == SPICE_CURSOR_TYPE_MONO) {
      uint8_t mask[4 * 100] = { 0 };
      cursor_ = SDL_CreateCursor(shape_data + stride * shape.height, mask,
        shape.width, shape.height, shape.hotspot_x, shape.hotspot_y);
      SDL_SetCursor(cursor_);
    } else {
      auto surface = SDL_CreateRGBSurfaceWithFormatFrom(shape_data, shape.width, shape.height,
        32, shape.width * 4, SDL_PIXELFORMAT_BGRA32);
      cursor_ = SDL_CreateColorCursor(surface, shape.hotspot_x, shape.hotspot_y);
      SDL_SetCursor(cursor_);
      SDL_FreeSurface(surface);
    }
    delete shape_data;
    cursor_shape_id_ = shape.id;
    cursor_visible_ = true;
  } else {
    if (cursor_visible_) {
      cursor_visible_ = false;
      SDL_ShowCursor(SDL_DISABLE);
    }
  }
}

/* Only use mutex with dequee, since we don't want to block the IoThread */
void Viewer::Render() {
  if (requested_update_window_) {
    requested_update_window_ = false;
    DestroyWindow();
    CreateWindow();
  }

  DisplayUpdate update;
  if (display_->AcquireUpdate(update)) {
    RenderCursor(&update.cursor);

    for (auto& partial : update.partials) {
      if (partial.x + partial.width <= width_ && partial.y + partial.height <= height_) {
        if (bpp_ == 8) {
          RenderSurface(&partial);
        } else {
          RenderPartial(&partial);
        }
      }
    }
    display_->ReleaseUpdate();
  }

  if (!update.partials.empty()) {
    SDL_RenderCopy(renderer_, screen_texture_, nullptr, nullptr);
    SDL_RenderPresent(renderer_);
  }
}


PointerInputInterface* Viewer::GetActivePointer() {
  if (machine_->IsPaused())
    return nullptr;
  for (auto pointer : pointers_) {
    if (pointer->InputAcceptable()) {
      return pointer;
    }
  }
  return nullptr;
}

void Viewer::LookupDevices() {
  keyboard_ = dynamic_cast<KeyboardInputInterface*>(machine_->LookupObjectByClass("Ps2"));
  display_ = dynamic_cast<DisplayInterface*>(machine_->device_manager()->LookupDeviceByClass("Qxl"));
  if (display_ == nullptr) {
    display_ = dynamic_cast<DisplayInterface*>(machine_->device_manager()->LookupDeviceByClass("Vga"));
    MV_ASSERT(display_);
  }
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<PointerInputInterface*>(o); })) {
    pointers_.push_back(dynamic_cast<PointerInputInterface*>(o));
  }
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<DisplayResizeInterface*>(o); })) {
    resizers_.push_back(dynamic_cast<DisplayResizeInterface*>(o));
  }
  MV_ASSERT(display_);

  display_->RegisterDisplayChangeListener([this]() {
    requested_update_window_ = true;
  });
}

/* Reference about SDL-2:
 * https://wiki.libsdl.org/APIByCategory
 */
int Viewer::MainLoop() {
  SetThreadName("mvisor-viewer");

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
      for (auto resizer : resizers_) {
        if (machine_->guest_os() == "Linux" && std::string("SpiceAgent") == dynamic_cast<Object*>(resizer)->classname()) {
          /* FIXME: spice agent resize is not working on Linux */
          continue;
        }
        if (resizer->Resize(pending_resize_.width, pending_resize_.height)) {
          break;
        }
      }
    }

    /* Keep display FPS */
    auto frame_cost_us = std::chrono::steady_clock::now() - frame_start_time;
    if (frame_cost_us < frame_interval_us) {
      std::this_thread::sleep_for(frame_interval_us - frame_cost_us);
    }
  }
  return 0;
}

void Viewer::SendPointerEvent() {
  auto pointer = GetActivePointer();
  if (pointer && window_) {
    int x, y, w, h;
    SDL_GetMouseState(&x, &y);
    SDL_GetWindowSize(window_, &w, &h);
    /* Check if window is scaled */
    x = x * width_ / w;
    y = y * height_ / h;
    pointer_state_.x = x;
    pointer_state_.y = y;
    pointer->QueuePointerEvent(pointer_state_);
  }
}

void Viewer::HandleEvent(const SDL_Event& event) {
  uint8_t transcoded[10] = { 0 };

  switch (event.type)
  {
  case SDL_KEYDOWN:
    if (event.key.keysym.sym == SDLK_PAUSE) {
      if (machine_->IsPaused()) {
        machine_->Resume();
      } else {
        machine_->Pause();
      }
    } else if (event.key.keysym.sym == SDLK_F2) {
      machine_->Pause();
      machine_->Save("/tmp/save");
      return;
    }
  case SDL_KEYUP:
    if (keyboard_ && TranslateScancode(event.key.keysym.scancode, event.type == SDL_KEYDOWN, transcoded)) {
      keyboard_->QueueKeyboardEvent(transcoded);
    }
    break;
  case SDL_MOUSEWHEEL:
    pointer_state_.z = event.wheel.y;
    SendPointerEvent();
    pointer_state_.z = 0;
    break;
  case SDL_MOUSEBUTTONDOWN:
  case SDL_MOUSEBUTTONUP:
    if (event.button.state) {
      pointer_state_.buttons |= (1 << event.button.button);
    } else {
      pointer_state_.buttons &= ~(1 << event.button.button);
    }
    SendPointerEvent();
    break;
  case SDL_MOUSEMOTION:
    /* if we dont check the pointer changes, it would have 1000 pointer events per second */
    if (event.motion.x != pointer_state_.x || event.motion.y != pointer_state_.y) {
      SendPointerEvent();
    }
    break;
  case SDL_WINDOWEVENT:
    switch (event.window.event)
    {
    case SDL_WINDOWEVENT_RESIZED:
      pending_resize_.triggered = true;
      pending_resize_.width = event.window.data1;
      pending_resize_.height = event.window.data2;
      pending_resize_.time = std::chrono::steady_clock::now();
      break;
    }
    break;
  case SDL_QUIT:
    machine_->Quit();
    break;
  }
}
