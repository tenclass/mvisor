#include "viewer.h"
#include <unistd.h>
#include "logger.h"
#include "devices/keyboard.h"
#include "viewer_font.inc"
#include "keymap.h"

Viewer::Viewer(Machine* machine) : machine_(machine) {
  device_manager_ = machine_->device_manager();
}

Viewer::~Viewer() {
}

void Viewer::DrawCharacter(int x, int y, int character, int attribute, uint8_t* font) {
  auto pallete = vga_device_->pallete();
  // Translate from color index to RGB
  uint32_t i = (attribute & 0xF) * 3;
  uint32_t front_color = (pallete[i] << 18) | (pallete[i + 1] << 10) | (pallete[i + 2] << 2);
  i = ((attribute >> 4) & 0xF) * 3;
  uint32_t back_color =  (pallete[i] << 18) | (pallete[i + 1] << 10) | (pallete[i + 2] << 2);

  int line_pixels = draw_buffer_->pitch / 4;
  uint32_t* buffer = (uint32_t*)draw_buffer_->pixels;
  buffer += (y * 16) * line_pixels;
  buffer += x * 8;
  
  // Draw the glyph
  for (int yy = 0; yy < 16; yy++) {
    for (int xx = 0; xx < 8; xx++) {
      if (font[character * 16 + yy] & (0x80 >> xx)) {
        buffer[xx] = front_color;
      } else {
        buffer[xx] = back_color;
      }
    }
    buffer += line_pixels;
  }
}

void Viewer::DrawTextCursor() {
  auto pallete = vga_device_->pallete();
  // Translate from color index to RGB
  uint32_t i = (0xF) * 3;
  uint32_t front_color = (pallete[i] << 18) | (pallete[i + 1] << 10) | (pallete[i + 2] << 2);
  uint8_t cx, cy, sl_start, sl_end;
  vga_device_->GetCursorLocation(&cx, &cy, &sl_start, &sl_end);

  int line_pixels = draw_buffer_->pitch / 4;
  uint32_t* buffer = (uint32_t*)draw_buffer_->pixels;
  uint32_t* end = buffer + line_pixels * 400;

  buffer += (cy * 16) * line_pixels;
  buffer += cx * 8;
  buffer += line_pixels * sl_start;

  for (int y = sl_start; y < sl_end; y++) {
    for (int x = 0; x < 8; x++) {
      if (buffer + x < end) {
        buffer[x] = front_color;
      }
    }
    buffer += line_pixels;
  }
}

void Viewer::DrawTextMode() {
  uint8_t* ptr = vga_device_->GetVRamHostAddress();
  
  // consider using BIOS fonts?
  uint8_t* font = (uint8_t*)__font8x16;

  UpdateScreenSize(640, 400);

  for (int y = 0; y < 25; y++) {
    for (int x = 0; x < 80; x++) {
      int character = *ptr++;
      int attribute = *ptr++;
      DrawCharacter(x, y, character, attribute, font);
    }
  }
  
  DrawTextCursor();
}

void Viewer::DrawGraphicMode() {
  if (!vga_device_->IsVbeEnabled()) {
    MV_LOG("Graphics mode without VBE is not supported yet!");
    return;
  }

  auto pallete = vga_device_->pallete();
  uint8_t* ptr = vga_device_->GetVRamHostAddress();

  int height = vga_device_->height();
  int width = vga_device_->width();
  int bpp = vga_device_->bpp();

  UpdateScreenSize(width, height);
  uint32_t* buffer = (uint32_t*)draw_buffer_->pixels;

  switch (bpp)
  {
  case 8:
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        int i = *ptr++ * 3;
        uint32_t color = (pallete[i] << 18) | (pallete[i + 1] << 10) | (pallete[i + 2] << 2);
        buffer[y * width + x] = color;
      }
    }
    break;
  case 24:
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        uint32_t color = (ptr[2] << 16) | (ptr[1] << 8) | (ptr[0]);
        buffer[y * width + x] = color;
        ptr += 3;
      }
    }
    break;
  }
}

void Viewer::UpdateScreenSize(int w, int h) {
  if (draw_buffer_ && draw_buffer_->w == w && draw_buffer_->h == h) {
    return;
  }
  if (!w || !h) {
    return;
  }
  MV_LOG("update screen size %dx%d", w, h);
  width_ = w;
  height_ = h;
  if (draw_buffer_) {
    SDL_FreeSurface(draw_buffer_);
  }
  auto format = screen_surface_->format;
  draw_buffer_ = SDL_CreateRGBSurface(SDL_SWSURFACE, w, h, 32, format->Rmask,
    format->Gmask, format->Bmask, format->Amask);
}

int Viewer::MainLoop() {
  SDL_Event event;

  vga_device_ = dynamic_cast<VgaDevice*>(device_manager_->LookupDeviceByName("vga"));
  KeyboardDevice* kbd = dynamic_cast<KeyboardDevice*>(
    device_manager_->LookupDeviceByName("keyboard")
  );
  MV_ASSERT(vga_device_);
  MV_ASSERT(kbd);

  // Fixed screen size at the moment
  screen_surface_ = SDL_SetVideoMode(1024, 768, 32, SDL_HWSURFACE | SDL_DOUBLEBUF | SDL_RESIZABLE);
  SDL_WM_SetCaption("MVisor - A mini x86 hypervisor", "MVisor");

  // Loop until all vcpu exits
  while (machine_->IsValid()) {
    if (vga_device_->IsTextMode()) {
      DrawTextMode();
    } else {
      DrawGraphicMode();
    }
    SDL_SoftStretch(draw_buffer_, &draw_buffer_->clip_rect, screen_surface_, &screen_surface_->clip_rect);
    SDL_Flip(screen_surface_);

    while (SDL_PollEvent(&event)) {
      uint8_t transcoded[10] = { 0 };
      switch (event.type)
      {
      case SDL_KEYDOWN:
      case SDL_KEYUP:
        if (TranslateScancode(event.key.keysym.scancode, event.type == SDL_KEYDOWN, transcoded)) {
          for (int i = 0; transcoded[i]; i++) {
            kbd->QueueKeyboardEvent(transcoded[i]);
          }
        }
        break;
      case SDL_QUIT:
        machine_->Quit();
        break;
      }
    }
    SDL_Delay(1000 / 60);
  }
  return 0;
}
