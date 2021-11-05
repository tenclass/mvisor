#include "viewer.h"
#include <unistd.h>
#include "logger.h"
#include "devices/ps2_controller.h"
#include "viewer_font.inc"
#include "keymap.h"

Viewer::Viewer(Machine* machine) : machine_(machine) {
  device_manager_ = machine_->device_manager();
}

Viewer::~Viewer() {
}

void Viewer::DrawCharacter(int x, int y, int character, int attribute) {
  auto pallete = vga_device_->pallete();
  // Translate from color index to RGB
  uint32_t i = (attribute & 0xF) * 3;
  uint32_t front_color = (pallete[i] << 18) | (pallete[i + 1] << 10) | (pallete[i + 2] << 2);
  i = ((attribute >> 4) & 0xF) * 3;
  uint32_t back_color =  (pallete[i] << 18) | (pallete[i + 1] << 10) | (pallete[i + 2] << 2);

  int line_pixels = screen_surface_->pitch / 4;
  uint32_t* buffer = (uint32_t*)screen_surface_->pixels;
  buffer += (y * 16) * line_pixels;
  buffer += x * 8;
  
  // Draw the glyph
  for (int yy = 0; yy < 16; yy++) {
    for (int xx = 0; xx < 8; xx++) {
      if (__font8x16[character][yy] & (0x80 >> xx)) {
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

  int line_pixels = screen_surface_->pitch / 4;
  uint32_t* buffer = (uint32_t*)screen_surface_->pixels;
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
  uint64_t gpa = vga_device_->GetVRamAddress();
  uint8_t* base = (uint8_t*)device_manager_->TranslateGuestMemory(gpa);
  MV_ASSERT(base);

  uint8_t* ptr = base;
  for (int y = 0; y < 25; y++) {
    for (int x = 0; x < 80; x++) {
      int character = *ptr++;
      int attribute = *ptr++;
      DrawCharacter(x, y, character, attribute);
    }
  }
  
  DrawTextCursor();
}

void Viewer::DrawGraphicMode() {
  uint64_t gpa = vga_device_->GetVRamAddress();
  uint8_t* ptr = (uint8_t*)device_manager_->TranslateGuestMemory(gpa);
  MV_ASSERT(ptr);
}

int Viewer::MainLoop() {
  SDL_Event event;
  // Fixed screen size at the moment
  screen_surface_ = SDL_SetVideoMode(640, 400, 32, SDL_HWSURFACE | SDL_DOUBLEBUF);
  vga_device_ = dynamic_cast<VgaDevice*>(device_manager_->LookupDeviceByName("vga"));
  Ps2ControllerDevice* ps2_device = dynamic_cast<Ps2ControllerDevice*>(
    device_manager_->LookupDeviceByName("ps2")
  );
  MV_ASSERT(vga_device_);
  MV_ASSERT(ps2_device);

  SDL_WM_SetCaption("MVisor - A mini x86 hypervisor", "MVisor");

  // Loop until all vcpu exits
  while (machine_->IsValid()) {
    if (vga_device_->IsTextMode()) {
      DrawTextMode();
    } else {
      DrawGraphicMode();
    }
    SDL_Flip(screen_surface_);

    while (SDL_PollEvent(&event)) {
      uint8_t transcoded[10] = { 0 };
      switch (event.type)
      {
      case SDL_KEYDOWN:
      case SDL_KEYUP:
        if (TranslateScancode(event.key.keysym.scancode, event.type == SDL_KEYDOWN, transcoded)) {
          for (int i = 0; transcoded[i]; i++) {
            ps2_device->QueueKeyboardEvent(transcoded[i]);
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
