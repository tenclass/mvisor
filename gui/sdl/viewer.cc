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
#include "../keymap.h"
#include "spice/enums.h"
#include "spice/vd_agent.h"

Viewer::Viewer(Machine* machine) : machine_(machine) {
  bzero(&pointer_state_, sizeof(pointer_state_));
  LookupDevices();
  MV_ASSERT(SDL_Init(SDL_INIT_VIDEO) == 0);
}

Viewer::~Viewer() {
  DestroyWindow();

  if (display_) {
    display_->UnregisterDisplayModeChangeListener(display_mode_listener_);
    display_->UnregisterDisplayUpdateListener(display_update_listener_);
  }
  if (playback_) {
    playback_->UnregisterPlaybackListener(playback_listener_);
  }
  if (clipboard_) {
    clipboard_->UnregisterClipboardListener(clipboard_listener_);
  }

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

  screen_surface_ = SDL_CreateRGBSurfaceWithFormat(0, width_, height_, 32, SDL_PIXELFORMAT_BGRA32);
  MV_ASSERT(screen_surface_);
  SDL_SetSurfaceBlendMode(screen_surface_, SDL_BLENDMODE_NONE);

  if (bpp_ == 8) {
    palette_ = SDL_AllocPalette(256);
  }
  UpdateCaption();
}

void Viewer::UpdateCaption() {
  char title[100];
  sprintf(title, "MVisor - A mini x86 hypervisor - %dx%dx%d", width_, height_, bpp_);
  SDL_SetWindowTitle(window_, title);
}

void Viewer::RenderSurface(const DisplayPartialBitmap* partial) {
  uint format = SDL_PIXELFORMAT_UNKNOWN;
  switch (partial->bpp)
  {
  case 32:
    format = SDL_PIXELFORMAT_BGRA32;
    break;
  case 24:
    format = SDL_PIXELFORMAT_BGR24;
    break;
  case 16:
    format = SDL_PIXELFORMAT_RGB565;
    break;
  case 8:
    format = SDL_PIXELFORMAT_INDEX8;
    break;
  default:
    MV_PANIC("unsupported video bpp=%d", bpp_);
  }

  SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormatFrom(partial->data,
      partial->width, partial->height, partial->bpp, partial->stride, format);

  if (partial->bpp == 8) {
    SDL_Color colors[256];
    const uint8_t* p;
    int count;
    bool dac_8bit;
    display_->GetPalette(&p, &count, &dac_8bit);
    if (!dac_8bit) {
      for (int i = 0; i < count; i++) {
        colors[i].r = *p++ << 2;
        colors[i].g = *p++ << 2;
        colors[i].b = *p++ << 2;
      }
    }
    SDL_SetPaletteColors(palette_, colors, 0, count);
    SDL_SetSurfacePalette(surface, palette_);
  }

  SDL_SetSurfaceBlendMode(surface, SDL_BLENDMODE_NONE);
  auto dest_rect = SDL_Rect {
    .x = partial->x,
    .y = partial->y,
    .w = partial->width,
    .h = partial->height
  };

  SDL_BlitSurface(surface, NULL, screen_surface_, &dest_rect);
  SDL_FreeSurface(surface);
}

void Viewer::RenderCursor(const DisplayMouseCursor* cursor_update) {
  if (cursor_update->visible) {
    if (!cursor_visible_) {
      SDL_ShowCursor(SDL_ENABLE);
      cursor_visible_ = true;
    }
    if (cursor_update->shape.id == cursor_shape_id_) {
      return;
    }
    if (cursor_) {
      SDL_FreeCursor(cursor_);
      cursor_ = nullptr;
    }
    auto& shape = cursor_update->shape;

    if (shape.type == SPICE_CURSOR_TYPE_MONO) {
      uint32_t stride = SPICE_ALIGN(shape.width, 8) >> 3;
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
  } else {
    if (cursor_visible_) {
      cursor_visible_ = false;
      SDL_ShowCursor(SDL_DISABLE);
    }
  }
}

/* Only use mutex with dequee, since we don't want to block the IoThread */
void Viewer::Render(const DisplayUpdate& update) {
  RenderCursor(&update.cursor);

  for (auto& partial : update.partials) {
    if (partial.bpp == bpp_ && partial.x + partial.width <= width_ && partial.y + partial.height <= height_) {
      RenderSurface(&partial);
    }
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

KeyboardInputInterface* Viewer::GetActiveKeyboard() {
  if (machine_->IsPaused())
    return nullptr;
  for (auto keyboard : keyboards_) {
    if (keyboard->InputAcceptable()) {
      return keyboard;
    }
  }
  return nullptr;
}

void Viewer::OnPlayback(PlaybackState state, const std::string& data) {
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
        MV_WARN("snd_pcm_open error: %s", snd_strerror(err));
        pcm_playback_error_ = true;
        break;
      }
      /* set the latency to multiple times of interval to buffer some more data */
      err = snd_pcm_set_params(pcm_playback_, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED,
        playback_format_.channels, playback_format_.frequency, 1, playback_format_.interval_ms * 1000 * 50);
      if (err < 0) {
        MV_WARN("snd_pcm_set_params error: %s\n", snd_strerror(err));
        snd_pcm_close(pcm_playback_);
        pcm_playback_ = nullptr;
        pcm_playback_error_ = true;
        break;
      }
    }
    /* assume format is s16le */
    auto frames = snd_pcm_writei(pcm_playback_, data.data(), data.size() / playback_format_.channels / 2);
    if (frames < 0 && frames != -EAGAIN) {
      MV_WARN("snd_pcm_writei failed: %s, ret=%d\n", snd_strerror(frames), frames);
    }
    break;
  }
  }
}

void Viewer::LookupDevices() {
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<KeyboardInputInterface*>(o); })) {
    keyboards_.push_back(dynamic_cast<KeyboardInputInterface*>(o));
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
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<SerialPortInterface*>(o); })) {
    auto port = dynamic_cast<SerialPortInterface*>(o);
    auto port_name = std::string(port->port_name());
    if (port_name == "com.redhat.spice.0") {
      clipboard_ = dynamic_cast<ClipboardInterface*>(o);
      spice_agent_listener_ = port->RegisterSerialPortListener([this, port](SerialPortEvent event, uint8_t* data, size_t size) {
        Schedule([this, port, event, data = std::string((const char*)data, size)] () {
          if (event == kSerialPortStatusChanged) {
            /* Set screen size when VDAgent is ready */
            SendResizerEvent();
          }
        });
      });
    }
  }
  /* At least one display device is required for SDL viewer */
  MV_ASSERT(display_);

  display_mode_listener_ = display_->RegisterDisplayModeChangeListener([this]() {
    Schedule([this]() {
      DestroyWindow();
      CreateWindow();
    });
  });
  display_update_listener_ = display_->RegisterDisplayUpdateListener([this](auto& update) {
    Schedule([this, update=std::move(update)]() {
      Render(update);
    });
  });
  if (playback_) {
    playback_listener_ = playback_->RegisterPlaybackListener([this](PlaybackState state, struct iovec iov) {
      Schedule([this, state, data = std::string((const char*)iov.iov_base, iov.iov_len)] () {
        OnPlayback(state, data);
      });
    });
  }
  if(clipboard_) {
    clipboard_listener_ = clipboard_->RegisterClipboardListener([this](const ClipboardData clipboard_data) {
      /* std::move don't copy data, but replace data reference */
      Schedule([this, clipboard_data = std::move(clipboard_data)] () {
        OnClipboardFromGuest(clipboard_data);
      });
    });
  }
}

void Viewer::OnClipboardFromGuest(const ClipboardData& clipboard_data) {
  switch (clipboard_data.type)
  {
  case VD_AGENT_CLIPBOARD_NONE:
    clipboard_data_ = "";
    SDL_SetClipboardText("");
    break;
  case VD_AGENT_CLIPBOARD_UTF8_TEXT:
    clipboard_data_ = clipboard_data.data;
    SDL_SetClipboardText(clipboard_data_.c_str());
    break;
  default:
    MV_ERROR("Unhandled clipboard type=0x%x", clipboard_data.type);
    break;
  }
}

/* Use viewer UI thread to handle tasks */
void Viewer::Schedule(VoidCallback callback) {
  mutex_.lock();
  tasks_.emplace_back(std::move(callback));
  mutex_.unlock();
  SDL_Event event = { .type = SDL_USEREVENT };
  SDL_PushEvent(&event);
}

/* Reference about SDL-2:
 * https://wiki.libsdl.org/APIByCategory
 */
void Viewer::MainLoop() {
  SetThreadName("mvisor-viewer");
  SetupKeyboardShortcuts();

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
      SendResizerEvent();
    }
  }
}

void Viewer::SendResizerEvent() {
  if (!pending_resize_.width || !pending_resize_.height) {
    return;
  }
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

void Viewer::SetupKeyboardShortcuts() {
  keyboard_shortcuts_[SDLK_F2] = [this]() {
    MV_LOG("Save");
    machine_->Pause();
    machine_->Save("/tmp/save");
  };

  keyboard_shortcuts_[SDLK_F3] = [this]() {
    MV_LOG("Migration");
    MV_ASSERT(machine_->Save("127.0.0.1", 9979));
  };

  keyboard_shortcuts_[SDLK_F4] = [this]() {
    MV_LOG("Migration Post");
    MV_ASSERT(machine_->PostSave());
  };

  keyboard_shortcuts_[SDLK_F10] = [this]() {
      MV_LOG("Print Memory");
      machine_->memory_manager()->PrintMemoryScope();
  };

  keyboard_shortcuts_[SDLK_F11] = [this]() {
    if (machine_->IsPaused()) {
      MV_LOG("Resume");
      machine_->Resume();
    } else {
      MV_LOG("Pause");
      machine_->Pause();
    }
  };

  keyboard_shortcuts_[SDLK_F12] = [this]() {
    MV_LOG("Reset");
    machine_->Reset();
  };
}

void Viewer::HandleEvent(const SDL_Event& event) {
  uint8_t transcoded[10] = { 0 };
  auto keyboard = GetActiveKeyboard();

  switch (event.type)
  {
  case SDL_USEREVENT: {
    std::deque<VoidCallback> tasks_copy;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      tasks_copy.swap(tasks_);
    }
    for (auto& task : tasks_copy) {
      task();
    }
    break;
  }
  case SDL_KEYDOWN:
    // handle alt + Fx
    if (event.key.keysym.mod & KMOD_LALT) {
      auto it = keyboard_shortcuts_.find(event.key.keysym.sym);
      if (it != keyboard_shortcuts_.end()) {
        if (keyboard) {
          // release alt key
          auto qcode = ScancodeFromUsb(SDL_SCANCODE_LALT);
          QcodeToAtset1(qcode, 0, transcoded);
          keyboard->QueueKeyboardEvent(transcoded, 0);
        }
        // run the shortcut in a detached thread
        std::thread(it->second).detach();
        return;
      }
    }
    // fall through
  case SDL_KEYUP:
    if (keyboard) {
      auto qcode = ScancodeFromUsb(event.key.keysym.scancode);
      if (qcode == 0) {
        break;
      }
      if (QcodeToAtset1(qcode, event.type == SDL_KEYDOWN, transcoded) == 0) {
        break;
      }
      auto mod = SDL_GetModState();
      uint8_t modifiers = ((mod & KMOD_NUM) ? 2: 0) | ((mod & KMOD_CAPS) ? 4 : 0);
      keyboard->QueueKeyboardEvent(transcoded, modifiers);
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
    case SDL_WINDOWEVENT_FOCUS_GAINED:
      /* When viewer got focused, check clipboard if changed */
      if (SDL_HasClipboardText() && clipboard_data_ != SDL_GetClipboardText()) {
        clipboard_data_ = SDL_GetClipboardText();
        if (clipboard_) {
          clipboard_->ClipboardDataToGuest(VD_AGENT_CLIPBOARD_UTF8_TEXT, clipboard_data_);
        }
      }
      break;
    }
    break;
  case SDL_QUIT:
    machine_->Quit();
    break;
  }
}
