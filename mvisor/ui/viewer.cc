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
  if (screen_surface_) {
    if (palette_) {
      SDL_FreePalette(palette_);
      palette_ = nullptr;
    }
    if (cursor_) {
      SDL_FreeCursor(cursor_);
      cursor_ = nullptr;
    }
    SDL_FreeSurface(screen_surface_);
    SDL_DestroyRenderer(renderer_);
    SDL_DestroyWindow(window_);
    screen_surface_ = nullptr;
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
    screen_surface_ = SDL_CreateRGBSurfaceWithFormat(0, width_, height_, 32, SDL_PIXELFORMAT_BGRA32);
    break;
  case 24:
    screen_surface_ = SDL_CreateRGBSurfaceWithFormat(0, width_, height_, 24, SDL_PIXELFORMAT_BGR24);
    break;
  case 16:
    screen_surface_ = SDL_CreateRGBSurfaceWithFormat(0, width_, height_, 16, SDL_PIXELFORMAT_BGR565);
    break;
  case 8:
    screen_surface_ = SDL_CreateRGBSurfaceWithFormat(0, width_, height_, 32, SDL_PIXELFORMAT_BGRA32);
    break;
  default:
    MV_PANIC("unsupported video bpp=%d", bpp_);
  }
  MV_ASSERT(screen_surface_);
  SDL_SetSurfaceBlendMode(screen_surface_, SDL_BLENDMODE_NONE);

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
  auto dst = (uint8_t*)screen_surface_->pixels;
  int dst_stride = screen_surface_->pitch;
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
}

void Viewer::RenderSurface(const DisplayPartialBitmap* partial) {
  MV_ASSERT(partial->width == width_ && partial->height == height_);
  MV_ASSERT(partial->vector.size() == 1 && partial->vector[0].iov_len == width_ * height_);
  auto surface_8bit = SDL_CreateRGBSurfaceWithFormatFrom(partial->vector[0].iov_base,
    partial->width, partial->height, bpp_, partial->stride, SDL_PIXELFORMAT_INDEX8);
  SDL_SetSurfacePalette(surface_8bit, palette_);
  if (screen_surface_)
    SDL_FreeSurface(screen_surface_);
  screen_surface_ = SDL_ConvertSurfaceFormat(surface_8bit, SDL_PIXELFORMAT_BGRA32, 0);
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

    if (shape.type == SPICE_CURSOR_TYPE_MONO) {
      uint8_t mask[4 * 100] = { 0 };
      cursor_ = SDL_CreateCursor((const uint8_t*)shape.data.data() + stride * shape.height,
        mask, shape.width, shape.height, shape.hotspot_x, shape.hotspot_y);
      SDL_SetCursor(cursor_);
    } else {
      auto surface = SDL_CreateRGBSurfaceWithFormatFrom((void*)shape.data.data(),
        shape.width, shape.height, 32, shape.width * 4, SDL_PIXELFORMAT_BGRA32);
      cursor_ = SDL_CreateColorCursor(surface, shape.hotspot_x, shape.hotspot_y);
      SDL_SetCursor(cursor_);
      SDL_FreeSurface(surface);
    }
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
    auto screen_texture = SDL_CreateTextureFromSurface(renderer_, screen_surface_);
    SDL_RenderCopy(renderer_, screen_texture, nullptr, nullptr);
    SDL_RenderPresent(renderer_);
    SDL_DestroyTexture(screen_texture);
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

void Viewer::OnPlayback(PlaybackState state, struct iovec& iov) {
  if (pcm_playback_error_) {
    return;
  }
  switch (state)
  {
  case kPlaybackStart: {
    playback_->GetPlaybackFormat(&playback_format_.format, &playback_format_.channels,
      &playback_format_.frequency, &playback_format_.interval_ms);
    break;
  }
  case kPlaybackStop: {
    if (!pcm_playback_)
      break;
    snd_pcm_close(pcm_playback_);
    pcm_playback_ = nullptr;
    break;
  }
  case kPlaybackData: {
    if (!pcm_playback_) {
      if (!playback_format_.frequency || !playback_format_.channels) {
        break;
      }

      int err;
      MV_ASSERT(pcm_playback_ == nullptr);
      if ((err = snd_pcm_open(&pcm_playback_, "default", SND_PCM_STREAM_PLAYBACK, SND_PCM_NONBLOCK)) < 0) {
        MV_LOG("snd_pcm_open error: %s", snd_strerror(err));
        pcm_playback_error_ = true;
        break;
      }
      /* set the latency to 10 times of interval to buffer some more data */
      err = snd_pcm_set_params(pcm_playback_, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED,
        playback_format_.channels, playback_format_.frequency, 1, playback_format_.interval_ms * 1000 * 10);
      if (err < 0) {
        MV_PANIC("snd_pcm_set_params error: %s\n", snd_strerror(err));
      }
    }
    /* assume format is s16le */
    auto frames = snd_pcm_writei(pcm_playback_, iov.iov_base, iov.iov_len / playback_format_.channels / 2);
    if (frames < 0) {
      MV_LOG("snd_pcm_writei failed: %s\n", snd_strerror(frames));
    }
    if (frames < 0)
      frames = snd_pcm_recover(pcm_playback_, frames, 0);
    if (frames < 0) {
      MV_LOG("snd_pcm_writei failed: %s\n", snd_strerror(frames));
    }
    break;
  }
  }
}

void Viewer::LookupDevices() {
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<KeyboardInputInterface*>(o); })) {
    keyboard_ = dynamic_cast<KeyboardInputInterface*>(o);
  }
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<DisplayInterface*>(o); })) {
    display_ = dynamic_cast<DisplayInterface*>(o);
  }
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<PlaybackInterface*>(o); })) {
    playback_ = dynamic_cast<PlaybackInterface*>(o);
  }
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<PointerInputInterface*>(o); })) {
    pointers_.push_back(dynamic_cast<PointerInputInterface*>(o));
  }
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<DisplayResizeInterface*>(o); })) {
    resizers_.push_back(dynamic_cast<DisplayResizeInterface*>(o));
  }
  /* At least one display device is required for SDL viewer */
  MV_ASSERT(display_);

  display_->RegisterDisplayModeChangeListener([this]() {
    requested_update_window_ = true;
    SDL_Event event = { .type = SDL_USEREVENT };
    SDL_PushEvent(&event);
  });
  display_->RegisterDisplayUpdateListener([this]() {
    SDL_Event event = { .type = SDL_USEREVENT };
    SDL_PushEvent(&event);
  });
  if (playback_) {
    playback_->RegisterPlaybackListener([this](PlaybackState state, struct iovec iov) {
      OnPlayback(state, iov);
    });
  }
}

/* Reference about SDL-2:
 * https://wiki.libsdl.org/APIByCategory
 */
int Viewer::MainLoop() {
  SetThreadName("mvisor-viewer");

  // Loop until all vcpu exits
  SDL_Event event;
  while (machine_->IsValid()) {
    auto frame_start_time = std::chrono::steady_clock::now();
    auto ret = SDL_WaitEventTimeout(&event, 300);
    if (ret == 1) {
      HandleEvent(event);
    }

    /* Check viewer window resize */
    if (pending_resize_.triggered && frame_start_time - pending_resize_.time >= std::chrono::milliseconds(200)) {
      pending_resize_.triggered = false;
      for (auto resizer : resizers_) {
        if (resizer->Resize(pending_resize_.width, pending_resize_.height)) {
          if (machine_->debug()) {
            MV_LOG("%s resize to %dx%d", dynamic_cast<Object*>(resizer)->name(),
              pending_resize_.width, pending_resize_.height);
          }
          break;
        }
      }
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
  case SDL_USEREVENT:
    Render();
    break;
  case SDL_KEYDOWN:
    if (event.key.keysym.sym == SDLK_PAUSE) {
      if (machine_->IsPaused()) {
        MV_LOG("Resume");
        machine_->Resume();
      } else {
        MV_LOG("Pause");
        machine_->Pause();
      }
    } else if (event.key.keysym.sym == SDLK_F2) {
      MV_LOG("Save");
      machine_->Pause();
      machine_->Save("/tmp/save");
      return;
    } else if (event.key.keysym.sym == SDLK_F12) {
      if (machine_->IsPaused()) {
        MV_LOG("Reset");
        machine_->Resume();
        machine_->Reset();
      } else {
        MV_LOG("Shutdown");
        machine_->Shutdown();
      }
    }
  case SDL_KEYUP:
    if (keyboard_ && TranslateScancode(event.key.keysym.scancode, event.type == SDL_KEYDOWN, transcoded)) {
      auto mod = SDL_GetModState();
      uint8_t modifiers = ((mod & KMOD_NUM) ? 2: 0) | ((mod & KMOD_CAPS) ? 4 : 0);
      keyboard_->QueueKeyboardEvent(transcoded, modifiers);
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
