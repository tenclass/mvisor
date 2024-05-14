
#include <cstring>
#include <arpa/inet.h>

#include "../keymap.h"
#include "spice/vd_agent.h"
#include "spice/enums.h"
#include "connection.h"
#include "logger.h"

static char vnc_version[] = "RFB 003.008\n";

VncConnection::VncConnection(VncServer* server, int fd) : server_(server), fd_(fd) {
  LookupDevices();

  // Send VNC version
  send(fd_, vnc_version, strlen(vnc_version), 0);

  // z_stream initialization
  zstream_.zalloc = Z_NULL;
  zstream_.zfree = Z_NULL;
  zstream_.opaque = Z_NULL;
  MV_ASSERT(deflateInit2(&zstream_, Z_DEFAULT_COMPRESSION, Z_DEFLATED,
    MAX_WBITS, MAX_MEM_LEVEL, Z_DEFAULT_STRATEGY) == Z_OK);
}

VncConnection::~VncConnection() {
  safe_close(&fd_);

  update_cv_.notify_all();
  if (update_thread_.joinable()) {
    update_thread_.join();
  }

  if (display_) {
    display_->UnregisterDisplayModeChangeListener(display_mode_listener_);
    display_->UnregisterDisplayUpdateListener(display_update_listener_);
  }
  if (clipboard_) {
    clipboard_->UnregisterClipboardListener(clipboard_listener_);
  }

  if (frame_buffer_) {
    pixman_image_unref(frame_buffer_);
  }
  if (cursor_buffer_) {
    pixman_image_unref(cursor_buffer_);
  }

  deflateEnd(&zstream_);
}

void VncConnection::LookupDevices() {
  auto machine = server_->machine();
  for (auto o : machine->LookupObjects([](auto o) { return dynamic_cast<KeyboardInputInterface*>(o); })) {
    keyboard_ = dynamic_cast<KeyboardInputInterface*>(o);
  }
  for (auto o : machine->LookupObjects([](auto o) { return dynamic_cast<DisplayInterface*>(o); })) {
    display_ = dynamic_cast<DisplayInterface*>(o);
  }
  for (auto o : machine->LookupObjects([](auto o) { return dynamic_cast<PointerInputInterface*>(o); })) {
    pointers_.push_back(dynamic_cast<PointerInputInterface*>(o));
  }
  for (auto o : machine->LookupObjects([](auto o) { return dynamic_cast<SerialPortInterface*>(o); })) {
    auto port = dynamic_cast<SerialPortInterface*>(o);
    auto port_name = std::string(port->port_name());
    if (port_name == "com.redhat.spice.0") {
      clipboard_ = dynamic_cast<ClipboardInterface*>(o);
    }
  }

  if (display_) {
    display_mode_listener_ = display_->RegisterDisplayModeChangeListener([this]() {
      server_->Schedule([this]() {
        ResizeFrameBuffer();
      });
    });

    display_update_listener_ = display_->RegisterDisplayUpdateListener([this](const DisplayUpdate& update) {
      server_->Schedule([this, update=std::move(update)]() {
        if (state_ == kVncRunning) {
          Render(update);
        }
      });
    });
  }

  if(clipboard_) {
    clipboard_listener_ = clipboard_->RegisterClipboardListener([this](const ClipboardData clipboard_data) {
      /* std::move don't copy data, but replace data reference */
      server_->Schedule([this, clipboard_data = std::move(clipboard_data)] () {
        OnClipboardFromGuest(clipboard_data);
      });
    });
  }
}

uint8_t VncConnection::ReadUInt8() {
  uint8_t value;
  recv(fd_, &value, sizeof(value), 0);
  return value;
}

uint16_t VncConnection::ReadUInt16() {
  uint16_t value;
  recv(fd_, &value, sizeof(value), 0);
  return ntohs(value);
}

uint32_t VncConnection::ReadUInt32() {
  uint32_t value;
  recv(fd_, &value, sizeof(value), 0);
  return ntohl(value);
}

void VncConnection::ReadData(void* data, size_t size) {
  recv(fd_, data, size, 0);
}

void VncConnection::ReadSkip(size_t size) {
  MV_ASSERT(size < 1024);
  uint8_t buf[size];
  recv(fd_, buf, size, 0);
}

bool VncConnection::OnReceive() {
  if (state_ == kVncRunning) {
    // Handle VNC message
    uint8_t message_type;
    ssize_t n = recv(fd_, &message_type, sizeof(message_type), 0);
    if (n <= 0) {
      return false;
    }

    // const char* message_type_names[] = {
    //   "SetPixelFormat",
    //   "",
    //   "SetEncodings",
    //   "FramebufferUpdateRequest",
    //   "KeyEvent",
    //   "PointerEvent",
    //   "ClientCutText",
    // };
    // MV_LOG("VNC message type %d %s", message_type, message_type_names[message_type]);

    switch (message_type)
    {
    case 0: // SetPixelFormat
      return OnSetPixelFormat();
    case 2: // SetEncodings
      return OnSetEncodings();
    case 3: // FramebufferUpdateRequest
      return OnFrameBufferUpdateRequest();
    case 4: // KeyEvent
      return OnKeyEvent();
    case 5: // PointerEvent
      return OnPointerEvent();
    case 6: // ClientCutText
      return OnClientCutText();
    default:
      MV_WARN("Unknown VNC message type %d", message_type);
    }
    return false;
  }

  // Handle VNC handshake
  char buf[1024];
  ssize_t n = recv(fd_, buf, sizeof(buf) - 1, 0);
  if (n <= 0) {
    return false;
  }
  buf[n] = '\0';

  switch (state_)
  {
  case kVncVersion: {
    if (n < 12 || strncmp(buf, "RFB ", 4) != 0) {
      MV_WARN("VNC version mismatch %s", buf);
      return false;
    }
    state_ = kVncSecurity;
    char security_types[] = {1, 1};
    send(fd_, security_types, sizeof(security_types), 0);
    break;
  }
  case kVncSecurity: {
    state_ = kVNcInit;
    uint32_t status = 0;
    send(fd_, &status, sizeof(status), 0);
    break;
  }
  case kVNcInit: {
    shared_ = buf[0] == 1;
    state_ = kVncRunning;
    if (display_ == nullptr) {
      return false;
    }
    SendServerInit();
    break;
  }
  case kVncClosed:
    return false;
  default:
    MV_WARN("Unknown VNC state %d", state_);
    return false;
  }
  return true;
}

struct ServerInitMessage {
  uint16_t framebuffer_width;
  uint16_t framebuffer_height;
  uint8_t  pixel_format[16];
  uint32_t name_length;
  char     name[256];
} __attribute__((packed));

void VncConnection::SendServerInit() {
  int w, h, bpp;
  display_->GetDisplayMode(&w, &h, &bpp, nullptr);
  frame_buffer_width_ = w;
  frame_buffer_height_ = h;

  ServerInitMessage msg = {
    .framebuffer_width = htons(w),
    .framebuffer_height = htons(h),
  };
  bzero(&pixel_format_, sizeof(pixel_format_));
  pixel_format_.bits_per_pixel = 32;
  pixel_format_.depth = 24;
  pixel_format_.true_color = 1;
  pixel_format_.red_max = htons(255);
  pixel_format_.green_max = htons(255);
  pixel_format_.blue_max = htons(255);
  pixel_format_.red_shift = 16;
  pixel_format_.green_shift = 8;
  pixel_format_.blue_shift = 0;
  memcpy(msg.pixel_format, &pixel_format_, sizeof(pixel_format_));

  auto name = server_->machine()->vm_name();
  auto name_length = std::min(name.size(), sizeof(msg.name));
  memcpy(msg.name, name.data(), name_length);
  msg.name_length = htonl(name_length);

  send(fd_, &msg, offsetof(ServerInitMessage, name) + name_length, 0);

  update_thread_ = std::thread(&VncConnection::UpdateLoop, this);
  // If machine is paused, resume it
  if (server_->machine()->IsPaused()) {
    server_->machine()->Resume();
  }
}

bool VncConnection::OnSetPixelFormat() {
  ReadSkip(3); // padding

  uint8_t pixel_format[16];
  ReadData(pixel_format, sizeof(pixel_format));

  std::lock_guard<std::recursive_mutex> lock(update_mutex_);
  pixel_format_.bits_per_pixel = pixel_format[0];
  pixel_format_.depth = pixel_format[1];
  pixel_format_.big_endian = pixel_format[2];
  pixel_format_.true_color = pixel_format[3];
  pixel_format_.red_max = ntohs(*(uint16_t*)&pixel_format[4]);
  pixel_format_.green_max = ntohs(*(uint16_t*)&pixel_format[6]);
  pixel_format_.blue_max = ntohs(*(uint16_t*)&pixel_format[8]);
  pixel_format_.red_shift = pixel_format[10];
  pixel_format_.green_shift = pixel_format[11];
  pixel_format_.blue_shift = pixel_format[12];

  if (frame_buffer_ != nullptr) {
    pixman_image_unref(frame_buffer_);
    frame_buffer_ = nullptr;
  }
  return true;
}


bool VncConnection::OnSetEncodings() {
  ReadSkip(1); // padding

  std::lock_guard<std::recursive_mutex> lock(update_mutex_);
  client_encodings_.clear();
  auto n_encodings = ReadUInt16();
  for (int i = 0; i < n_encodings; i++) {
    auto encoding = (int32_t)ReadUInt32();
    client_encodings_.push_back(encoding);
  }

  // Encoding RAW must be supported
  if (!IsEncodingSupported(0)) {
    MV_WARN("VNC encoding RAW not supported");
    return false;
  }
  return true;
}

bool VncConnection::IsEncodingSupported(int32_t encoding) {
  std::lock_guard<std::recursive_mutex> lock(update_mutex_);
  return std::find(client_encodings_.begin(), client_encodings_.end(), encoding) != client_encodings_.end();
}


struct FrameBufferUpdateRequest {
  uint8_t incremental;
  uint16_t x;
  uint16_t y;
  uint16_t w;
  uint16_t h;
} __attribute__((packed));

bool VncConnection::OnFrameBufferUpdateRequest() {
  FrameBufferUpdateRequest req;
  ReadData(&req, sizeof(req));
  req.x = ntohs(req.x);
  req.y = ntohs(req.y);
  req.w = ntohs(req.w);
  req.h = ntohs(req.h);

  if (req.incremental == 0) {
    display_->Refresh();
  }
  frame_buffer_update_requested_ = true;
  update_cv_.notify_one();
  return true;
}

bool VncConnection::OnClientCutText() {
  ReadSkip(3); // padding

  auto length = ReadUInt32();
  std::string text;
  text.resize(length);
  ReadData((char*)text.data(), length);

  if (clipboard_) {
    clipboard_->ClipboardDataToGuest(VD_AGENT_CLIPBOARD_UTF8_TEXT, text);
  }
  return true;
}

void VncConnection::OnClipboardFromGuest(const ClipboardData& clipboard_data) {
  if (clipboard_data.type == VD_AGENT_CLIPBOARD_UTF8_TEXT) {
    uint8_t buf[8] = {0};
    buf[0] = 3; // ServerCutText
    *(uint32_t*)&buf[4] = htonl(clipboard_data.data.size());
    send(fd_, buf, sizeof(buf), 0);
    send(fd_, clipboard_data.data.data(), clipboard_data.data.size(), 0);
  }
}

bool VncConnection::OnKeyEvent() {
  uint8_t down = ReadUInt8();
  ReadSkip(2); // padding

  uint32_t key = ReadUInt32();
  if (keyboard_) {
    auto qcode = ScancodeFromX11(key);
    if (qcode == 0) {
      MV_WARN("Unknown key %d", key);
      return true;
    }
    uint8_t transcoded[10] = { 0 };
    if (QcodeToAtset1(qcode, down, transcoded) == 0) {
      return true;
    }

    if (down && qcode == Q_KEY_CODE_CAPS_LOCK) {
      modifiers_ ^= 4;
    }
    if (keyboard_->InputAcceptable()) {
      keyboard_->QueueKeyboardEvent(transcoded, modifiers_);
    }
  }
  return true;
}


bool VncConnection::OnPointerEvent() {
  // buttons bits: 0=left, 1=middle, 2=right, 3=scroll up, 4=scroll down, 5=scroll left, 6=scroll right
  auto buttons = ReadUInt8();
  auto x = ReadUInt16();
  auto y = ReadUInt16();

  uint screen_width, screen_height;
  display_->GetDisplayMode((int*)&screen_width, (int*)&screen_height, nullptr, nullptr);

  PointerEvent event = {
    .buttons = uint(((buttons & 1) ? 2 : 0) | ((buttons & 2) ? 4 : 0) | ((buttons & 4) ? 8 : 0)),
    .x = x,
    .y = y,
    .z = (buttons & 8) ? 1 : ((buttons & 0x10) ? -1 : 0),
    .screen_width = screen_width,
    .screen_height = screen_height,
  };
  
  auto pointer = GetActivePointer();
  if (pointer) {
    pointer->QueuePointerEvent(event);
  }
  return true;
}

PointerInputInterface* VncConnection::GetActivePointer() {
  if (server_->machine()->IsPaused())
    return nullptr;
  for (auto pointer : pointers_) {
    if (pointer->InputAcceptable()) {
      return pointer;
    }
  }
  return nullptr;
}

void VncConnection::ResizeFrameBuffer() {
  std::unique_lock<std::recursive_mutex> lock(update_mutex_);
  int w, h, bpp;
  display_->GetDisplayMode(&w, &h, &bpp, nullptr);
  frame_buffer_width_ = w;
  frame_buffer_height_ = h;

  if (frame_buffer_) {
    pixman_image_unref(frame_buffer_);
    frame_buffer_ = nullptr;
  }

  dirty_rects_.clear();
  if (state_ == kVncRunning) {
    SendDesktopSize();
  }
}

void VncConnection::SendDesktopSize() {
  if (IsEncodingSupported(-223)) {
    uint8_t buf[16] = {0};
    *(uint16_t*)&buf[2] = htons(1);  // number of rectangles
    *(uint16_t*)&buf[8] = htons(frame_buffer_width_);
    *(uint16_t*)&buf[10] = htons(frame_buffer_height_);
    *(uint32_t*)&buf[12] = htonl(-223); // DesktopSize pseudo-encoding
    send(fd_, buf, sizeof(buf), 0);
  }
}

pixman_format_code_t VncConnection::GetPixelFormat(int bpp) {
  switch (bpp) {
  case 8:
    return PIXMAN_a2b2g2r2;
  case 16:
    return PIXMAN_b5g6r5;
  case 24:
    return PIXMAN_b8g8r8;
  default:
    return PIXMAN_a8b8g8r8;
  }
}

void VncConnection::CreateFrameBuffer() {
  if (frame_buffer_ == nullptr) {
    auto bytes_per_pixel = pixel_format_.bits_per_pixel / 8;
    auto code = GetPixelFormat(pixel_format_.bits_per_pixel);
    auto stride_bytes = frame_buffer_width_ * bytes_per_pixel;
    frame_buffer_ = pixman_image_create_bits(code, frame_buffer_width_, frame_buffer_height_, nullptr, stride_bytes);
    MV_ASSERT(frame_buffer_);
  }
}

void VncConnection::RenderSurface(const DisplayPartialBitmap* partial) {
  auto bytes_per_pixel = pixel_format_.bits_per_pixel / 8;
  if (frame_buffer_ == nullptr) {
    CreateFrameBuffer();
  }

  auto src = partial->data;
  auto src_stride = partial->stride;
  auto dst_x = partial->x;
  auto dst_y = partial->y;
  auto src_w = partial->width;
  auto src_h = partial->height;

  if (partial->bpp == 8) {
    // palette mode
    auto palette = partial->palette;
    auto stride = pixman_image_get_stride(frame_buffer_);
    auto data = (uint8_t*)pixman_image_get_data(frame_buffer_);
    auto dst = data + dst_y * stride + dst_x * 4;
    for (int y = 0; y < src_h; y++) {
      for (int x = 0; x < src_w; x++) {
        auto index = src[y * src_stride + x];
        auto color = palette + index * 3;
        if (pixel_format_.bits_per_pixel == 8) {
          dst[bytes_per_pixel * x] = ((color[2] >> 4) & 3) | ((color[1] >> 4) & 3) << 2 | ((color[0] >> 4) & 3) << 4;
        } else {
          dst[bytes_per_pixel * x + 0] = color[2] << 2;
          dst[bytes_per_pixel * x + 1] = color[1] << 2;
          dst[bytes_per_pixel * x + 2] = color[0] << 2;
          dst[bytes_per_pixel * x + 3] = 0;
        }
      }
      dst += stride;
    }
  } else if (partial->bpp == 24) {
    auto surface = pixman_image_create_bits_no_clear(PIXMAN_b8g8r8, src_w, src_h, (uint32_t*)src, src_stride);
    MV_ASSERT(surface);
    pixman_image_composite(PIXMAN_OP_SRC, surface, nullptr, frame_buffer_, 0, 0, 0, 0, dst_x, dst_y, src_w, src_h);
    pixman_image_unref(surface);
  } else if (partial->bpp == 32) {
    auto surface = pixman_image_create_bits_no_clear(PIXMAN_a8b8g8r8, src_w, src_h, (uint32_t*)src, src_stride);
    MV_ASSERT(surface);
    pixman_image_composite(PIXMAN_OP_SRC, surface, nullptr, frame_buffer_, 0, 0, 0, 0, dst_x, dst_y, src_w, src_h);
    pixman_image_unref(surface);
  } else {
    MV_WARN("Unsupported bpp %d", partial->bpp);
    return;
  }

  AddDirtyRect(dst_y, dst_x, dst_y + src_h, dst_x + src_w);
}

void VncConnection::RenderCursor(const DisplayMouseCursor* cursor_update) {
  if (cursor_update->visible == 0) {
    if (cursor_shape_id_ != 0) {
      // Create a blank cursor if the cursor is hidden
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);
      if (cursor_buffer_) {
        pixman_image_unref(cursor_buffer_);
      }
      cursor_buffer_ = pixman_image_create_bits(PIXMAN_a8b8g8r8, 4, 4, nullptr, 2 * 4);
      MV_ASSERT(cursor_buffer_);
      cursor_hotspot_x_ = 0;
      cursor_hotspot_y_ = 0;
      cursor_shape_id_ = 0;
      cursor_update_requested_ = true;
      return;
    }
  } else {
    if (cursor_shape_id_ == cursor_update->shape.id) {
      return;
    }

    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    if (cursor_buffer_) {
      pixman_image_unref(cursor_buffer_);
      cursor_buffer_ = nullptr;
    }
    auto& shape = cursor_update->shape;

    cursor_shape_id_ = shape.id;
    cursor_hotspot_x_ = shape.hotspot_x;
    cursor_hotspot_y_ = shape.hotspot_y;

    if (shape.type == SPICE_CURSOR_TYPE_MONO) {
      // convert to Alpha cursor
      cursor_buffer_ = pixman_image_create_bits_no_clear(PIXMAN_a8b8g8r8, shape.width, shape.height, nullptr, shape.width * 4);
      MV_ASSERT(cursor_buffer_);
      // convert monochrome cursor to alpha cursor
      auto stride = pixman_image_get_stride(cursor_buffer_);
      auto data = (uint8_t*)pixman_image_get_data(cursor_buffer_);
      auto src_stride = (shape.width + 7) / 8;
      auto src = (const uint8_t*)shape.data.data() + src_stride * shape.height; // mask
      for (int y = 0; y < shape.height; y++) {
        auto dst = data + y * stride;
        for (int x = 0; x < shape.width; x++) {
          auto bit = src[y * src_stride + x / 8] & (1 << (7 - x % 8));
          dst[4 * x + 0] = 0;
          dst[4 * x + 1] = 0;
          dst[4 * x + 2] = 0;
          dst[4 * x + 3] = bit ? 0xFF : 0x00;
        }
      }
    } else {
      cursor_buffer_ = pixman_image_create_bits_no_clear(PIXMAN_a8b8g8r8, shape.width, shape.height, nullptr, shape.width * 4);
      MV_ASSERT(cursor_buffer_);
      auto data = (uint8_t*)pixman_image_get_data(cursor_buffer_);
      memcpy(data, shape.data.data(), shape.width * shape.height * 4);
      auto stride = pixman_image_get_stride(cursor_buffer_);
      auto src_stride = shape.width * 4;
      for (int y = 0; y < shape.height; y++) {
        auto dst = data + y * stride;
        auto src = shape.data.data() + y * src_stride;
        for (int x = 0; x < shape.width; x++) {
          dst[4 * x + 0] = src[4 * x + 2];
          dst[4 * x + 1] = src[4 * x + 1];
          dst[4 * x + 2] = src[4 * x + 0];
          dst[4 * x + 3] = src[4 * x + 3];
        }
      }
    }
    cursor_shape_id_ = shape.id;
    cursor_update_requested_ = true;
  }
}

void VncConnection::Render(const DisplayUpdate& update) {
  for (auto& partial : update.partials) {
    RenderSurface(&partial);
  }
  RenderCursor(&update.cursor);

  if ((frame_buffer_update_requested_ && !dirty_rects_.empty()) || cursor_update_requested_) {
    update_cv_.notify_one();
  }
}

void VncConnection::SendCursorUpdate() {
  std::unique_lock<std::recursive_mutex> lock(update_mutex_);
  if (!cursor_buffer_ || !IsEncodingSupported(-314)) {
    return;
  }

  int height = pixman_image_get_height(cursor_buffer_);
  int width = pixman_image_get_width(cursor_buffer_);
  uint8_t header[20] = {0};
  *(uint16_t*)&header[2] = htons(1);  // number of rectangles
  *(uint16_t*)&header[4] = htons(cursor_hotspot_x_);
  *(uint16_t*)&header[6] = htons(cursor_hotspot_y_);
  *(uint16_t*)&header[8] = htons(width);
  *(uint16_t*)&header[10] = htons(height);
  *(uint32_t*)&header[12] = htonl(-314); // Alpha cursor pseudo-encoding
  *(uint32_t*)&header[16] = htonl(0); // RAW encoding of cursor
  send(fd_, header, sizeof(header), 0);

  auto stride = pixman_image_get_stride(cursor_buffer_);
  auto data = (uint8_t*)pixman_image_get_data(cursor_buffer_);
  send(fd_, data, height * stride, 0);
}

void VncConnection::SendFrameBufferUpdate(int x, int y, int width, int height) {
  std::unique_lock<std::recursive_mutex> lock(update_mutex_);
  if (frame_buffer_ == nullptr) {
    // Frame buffer is destroyed if client set pixel format
    return;
  }

  MV_ASSERT(width > 0 && height > 0);
  MV_ASSERT(x + width <= frame_buffer_width_ && y + height <= frame_buffer_height_);
  auto bytes_per_pixel = pixel_format_.bits_per_pixel / 8;

  uint8_t header[16] = {0};
  *(uint16_t*)&header[2] = htons(1);  // number of rectangles
  *(uint16_t*)&header[4] = htons(x);
  *(uint16_t*)&header[6] = htons(y);
  *(uint16_t*)&header[8] = htons(width);
  *(uint16_t*)&header[10] = htons(height);
  
  if (IsEncodingSupported(6)) {
    *(uint32_t*)&header[12] = htonl(6); // zlib encoding
    std::string compressed;
    compressed.resize(width * height * 4 + 100);
    auto dst = (uint8_t*)compressed.data();
    memcpy(dst, header, 16);
    uint32_t* compressed_size_ptr = (uint32_t*)&dst[16];
    zstream_.next_out = (Bytef*)&dst[20];
    zstream_.avail_out = compressed.size() - 20;

    // fill raw encoding data
    auto stride = pixman_image_get_stride(frame_buffer_);
    auto data = (uint8_t*)pixman_image_get_data(frame_buffer_);
    auto src = data + y * stride + x * bytes_per_pixel;
    for (int i = 0; i < height; i++) {
      zstream_.next_in = src;
      zstream_.avail_in = width * bytes_per_pixel;
      MV_ASSERT(deflate(&zstream_, Z_SYNC_FLUSH) == Z_OK);
      src += stride;
    }

    // fix compressed size
    uint32_t compressed_size = compressed.size() - 20 - zstream_.avail_out;
    *compressed_size_ptr = htonl(compressed_size);
    send(fd_, compressed.data(), 20 + compressed_size, 0);
  } else {
    *(uint32_t*)&header[12] = htonl(0); // Raw encoding
    send(fd_, header, sizeof(header), 0);

    auto stride = pixman_image_get_stride(frame_buffer_);
    auto data = (uint8_t*)pixman_image_get_data(frame_buffer_);
    auto dst = data + y * stride + x * bytes_per_pixel;
    
    for (int i = 0; i < height; i++) {
      send(fd_, dst, width * bytes_per_pixel, 0);
      dst += stride;
    }
  }
}

void VncConnection::UpdateLoop() {
  SetThreadName("mvisor-vnc-connection");

  while (fd_ != -1) {
    std::unique_lock<std::mutex> lock(server_->mutex());
    update_cv_.wait(lock, [this]() {
      return fd_ == -1 || (frame_buffer_update_requested_ && !dirty_rects_.empty()) || cursor_update_requested_;
    });
    if (fd_ == -1) {
      break;
    }
    if (state_ != kVncRunning) {
      continue;
    }
    
    // Send large data to client will block the thread 
    // until the data is sent, so we should unlock the mutex
    if (frame_buffer_update_requested_ && !dirty_rects_.empty()) {
      VncRect rect = dirty_rects_.back();
      dirty_rects_.pop_back();
      if (dirty_rects_.empty()) {
        frame_buffer_update_requested_ = false;
      }
      lock.unlock();
      SendFrameBufferUpdate(rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top);
    } else if (cursor_update_requested_) {
      cursor_update_requested_ = false;
      lock.unlock();
      SendCursorUpdate();
    }
  }
}

/* Align width to 16 and height to 2 for SSE or AVX */
void VncConnection::AddDirtyRect(int top, int left, int bottom, int right) {
  /* make sure position and size of the created slice is multiple of 2 */
  const int width_alignment = 16;
  if (left % width_alignment) {
    left -= left % width_alignment;
  }
  if (right % width_alignment) {
    right += width_alignment - (right % width_alignment);
  }

  const int height_alignment = 2;
  if (top % height_alignment) {
    top -= top % height_alignment;
  }
  if (bottom % height_alignment) {
    bottom += height_alignment - (bottom % height_alignment);
  }
  /* avoid overflow */
  if (right > frame_buffer_width_) {
    right = frame_buffer_width_;
  }
  if (bottom > frame_buffer_height_) {
    bottom = frame_buffer_height_;
  }
  if (right - left < 2 || bottom - top < 2) {
    return;
  }

  AddDirtyRectInternal(top, left, bottom, right);
}

/* Use dirty rectangle algorithm to remove overlapped parts */
void VncConnection::AddDirtyRectInternal(int top, int left, int bottom, int right) {
  if (left >= right || top >= bottom) {
    return;
  }

  size_t current_size = dirty_rects_.size();
  for (size_t i = 0; i < current_size; i++) {
    auto r = dirty_rects_[i];
    if (left < r.right && top < r.bottom && r.left < right && r.top < bottom) {
      if (top < r.top)
        AddDirtyRectInternal(top, left, r.top, right);
      if (bottom > r.bottom)
        AddDirtyRectInternal(r.bottom, left, bottom, right);
      if (left < r.left)
        AddDirtyRectInternal(std::max(top, r.top), left, std::min(bottom, r.bottom), r.left);
      if (right > r.right)
        AddDirtyRectInternal(std::max(top, r.top), r.right, std::min(bottom, r.bottom), right);
      return;
    }
  }

  dirty_rects_.emplace_back(VncRect {
    top, left, bottom, right
  });
}
