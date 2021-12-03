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
#include "logger.h"
#include "viewer_font.inc"
#include "keymap.h"

Viewer::Viewer(Machine* machine) : machine_(machine) {
  device_manager_ = machine_->device_manager();
}

Viewer::~Viewer() {
}

void Viewer::DrawCharacter(int x, int y, int character, int attribute, uint8_t* font) {
  uint8_t* buffer = (uint8_t*)screen_surface_->pixels;
  buffer += (y * 16) * screen_surface_->pitch;
  buffer += x * 8;
  
  // Draw the glyph
  uint8_t fore_color = attribute & 0xF;
  uint8_t back_color = (attribute >> 4) & 0xF;
  for (int yy = 0; yy < 16; yy++) {
    for (int xx = 0; xx < 8; xx++) {
      if (font[character * 16 + yy] & (0x80 >> xx)) {
        buffer[xx] = fore_color;
      } else {
        buffer[xx] = back_color;
      }
    }
    buffer += screen_surface_->pitch;
  }
}

void Viewer::DrawTextCursor() {
  uint32_t fore_color = 7;
  uint8_t cx, cy, sl_start, sl_end;
  display_->GetCursorLocation(&cx, &cy, &sl_start, &sl_end);

  uint8_t* buffer = (uint8_t*)screen_surface_->pixels;
  buffer += (cy * 16) * screen_surface_->pitch;
  buffer += cx * 8;

  uint8_t* end = buffer + screen_surface_->pitch * height_;

  buffer += screen_surface_->pitch * sl_start;

  for (int y = sl_start; y < sl_end; y++) {
    for (int x = 0; x < 8; x++) {
      if (buffer + x < end) {
        buffer[x] = fore_color;
      }
    }
    buffer += screen_surface_->pitch;
  }
}

void Viewer::DrawTextMode() {
  uint8_t* ptr = display_->GetVRamHostAddress();
  if (ptr == nullptr) {
    return;
  }
  // consider using BIOS fonts?
  uint8_t* font = (uint8_t*)__font8x16;

  for (int y = 0; y < 25; y++) {
    for (int x = 0; x < 80; x++) {
      int character = *ptr++;
      int attribute = *ptr++;
      DrawCharacter(x, y, character, attribute, font);
    }
  }
  
  DrawTextCursor();
}

void Viewer::DrawGraphicsMode() {
  uint8_t* vram = display_->GetVRamHostAddress();
  if (vram == nullptr) {
    return;
  }

  uint8_t* dst = (uint8_t*)screen_surface_->pixels;
  uint8_t* src = vram;
  int src_scanline = width_ * (bpp_ >> 3);
  for (int y = 0; y < screen_surface_->h; y++) {
    memcpy(dst, src, src_scanline);
    dst += screen_surface_->pitch;
    src += src_scanline;
  }
}

void Viewer::UpdateWindow() {
  DisplayMode mode;
  uint16_t w, h, bpp;
  display_->GetDisplayMode(&mode, &w, &h, &bpp);

  mode_ = mode;
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

void Viewer::AcquireDisplayFrame() {
  if (requested_update_window_) {
    requested_update_window_ = false;
    UpdateWindow();
  }

  if (!screen_surface_) {
    return;
  }

  SDL_LockSurface(screen_surface_);
  switch (mode_)
  {
  case kDisplayTextMode:
    DrawTextMode();
    break;
  case kDisplayVgaMode:
  case kDisplayVbeMode:
    DrawGraphicsMode();
    break;
  default:
    MV_LOG("Graphics mode without VBE is not supported yet!");
    break;
  }
  SDL_UnlockSurface(screen_surface_);

  SDL_Flip(screen_surface_);
}

/* Reference about SDL-1.2:
 * https://www.libsdl.org/release/SDL-1.2.15/docs/html/index.html
 * FXIME: Why not SDL2? SDL2 create lots of threads after calling SDL_Init, it crashes after
 * a SIGUSR is sent to the vCPU.
 */
int Viewer::MainLoop() {
  SDL_Event event;

  display_ = dynamic_cast<DisplayInterface*>(device_manager_->LookupDeviceByName("Qxl"));
  keyboard_ = dynamic_cast<KeyboardInputInterface*>(device_manager_->LookupDeviceByName("Keyboard"));
  spice_agent_ = dynamic_cast<SpiceAgentInterface*>(device_manager_->LookupDeviceByName("SpiceAgent"));
  display_->RegisterDisplayChangeListener([this]() {
    requested_update_window_ = true;
  });
  MV_ASSERT(display_);
  uint32_t mouse_buttons = 0;

  // Loop until all vcpu exits
  while (machine_->IsValid()) {
    AcquireDisplayFrame();

    while (SDL_PollEvent(&event)) {
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
        if (grab_input_) {
          /* swap the middle button and right button bit of input state */
          uint8_t ps2_state = ((mouse_buttons & 2) >> 1) | (mouse_buttons & 4) | ((mouse_buttons & 8) >> 2);
          /* multiply by 2 to prevent host mouse go out of window */
          keyboard_->QueueMouseEvent(ps2_state, event.motion.xrel * 2, event.motion.yrel * 2, 0);
        } else if (spice_agent_ && spice_agent_->CanAcceptInput()) {
          int x, y;
          SDL_GetMouseState(&x, &y);
          spice_agent_->QueuePointerEvent(mouse_buttons, x, y);
        }
        break;
      case SDL_VIDEORESIZE:
        if (spice_agent_) {
          spice_agent_->Resize(event.resize.w, event.resize.h);
        }
        break;
      case SDL_QUIT:
        machine_->Quit();
        break;
      }
    }
    SDL_Delay(1000 / 30);
  }
  return 0;
}
