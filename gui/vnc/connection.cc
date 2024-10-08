#include "version.h"
#include <cstring>
#include <arpa/inet.h>

#ifdef HAS_OPENSSL
#include <openssl/evp.h>
#include <openssl/err.h>
#endif

#include "../keymap.h"
#include "spice/vd_agent.h"
#include "spice/enums.h"
#include "connection.h"
#include "logger.h"

static char vnc_version[] = "RFB 003.008\n";

VncConnection::VncConnection(VncServer* server, int fd) : server_(server), fd_(fd) {
  // Send VNC version
  send(fd_, vnc_version, strlen(vnc_version), 0);
}

VncConnection::~VncConnection() {
  state_ = kVncClosed;
  safe_close(&fd_);

  if (update_thread_.joinable()) {
    update_cv_.notify_all();
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

void VncConnection::Close() {
  std::unique_lock<std::mutex> lock(update_mutex_);
  state_ = kVncClosed;
  safe_close(&fd_);
  update_cv_.notify_all();
}

void VncConnection::LookupDevices() {
  auto machine = server_->machine();
  for (auto o : machine->LookupObjects([](auto o) { return dynamic_cast<KeyboardInputInterface*>(o); })) {
    keyboards_.insert(keyboards_.begin(), dynamic_cast<KeyboardInputInterface*>(o));
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
        std::lock_guard<std::mutex> lock(update_mutex_);
        OnDisplayModeChange();
      });
    });

    display_update_listener_ = display_->RegisterDisplayUpdateListener([this](const DisplayUpdate& update) {
      server_->Schedule([this, update=std::move(update)]() {
        if (state_ == kVncRunning) {
          std::lock_guard<std::mutex> lock(update_mutex_);
          OnDisplayUpdate(update);
        }
      });
    });
  }

  if(clipboard_) {
    clipboard_listener_ = clipboard_->RegisterClipboardListener([this](const ClipboardData clipboard_data) {
      /* std::move don't copy data, but replace data reference */
      server_->Schedule([this, clipboard_data = std::move(clipboard_data)] () {
        std::lock_guard<std::mutex> lock(update_mutex_);
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

bool VncConnection::CheckClientAuth(const char* buffer, ssize_t length) {
#ifdef HAS_OPENSSL
  if (length < 16) {
    return false;
  }

  uint8_t encrypted[16], key[8];
  auto password = server_->password();
  for (size_t i = 0; i < sizeof(key); i++) {
    uint8_t r = i < password.size() ? password[i] : 0;
    r = (r & 0xf0) >> 4 | (r & 0x0f) << 4;
    r = (r & 0xcc) >> 2 | (r & 0x33) << 2;
    r = (r & 0xaa) >> 1 | (r & 0x55) << 1;
    key[i] = r;
  }

  EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
  MV_ASSERT(ctx);
  if (EVP_CipherInit_ex(ctx, EVP_des_ecb(), nullptr, key, nullptr, 1) == 0) {
    MV_WARN("EVP_CipherInit_ex failed %s", ERR_error_string(ERR_get_error(), nullptr));
    EVP_CIPHER_CTX_free(ctx);
    return false;
  }

  // Encrypt chanllenge with password
  int outlen = 0;
  if (EVP_CipherUpdate(ctx, encrypted, &outlen, challenge_, sizeof(challenge_)) == 0) {
    EVP_CIPHER_CTX_free(ctx);
    return false;
  }
  if (EVP_CipherFinal_ex(ctx, encrypted + outlen, &outlen) == 0) {
    EVP_CIPHER_CTX_free(ctx);
    return false;
  }
  EVP_CIPHER_CTX_free(ctx);

  if (memcmp(encrypted, buffer, sizeof(encrypted)) != 0) {
    return false;
  }
  return true;
#else
  MV_UNUSED(buffer);
  MV_UNUSED(length);
  return false;
#endif
}

bool VncConnection::OnReceive() {
  if (state_ == kVncRunning) {
    // Handle VNC message
    uint8_t message_type;
    ssize_t n = recv(fd_, &message_type, sizeof(message_type), 0);
    if (n <= 0) {
      return false;
    }

    std::lock_guard<std::mutex> lock(update_mutex_);
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
  } else if (state_ == kVncClosed) {
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
    char security_types[] = {1, (char)server_->security_type()};
    send(fd_, security_types, sizeof(security_types), 0);
    break;
  }
  case kVncSecurity: {
    if (n < 1 || buf[0] != server_->security_type()) {
      MV_WARN("VNC security type not supported %d", buf[0]);
      return false;
    }
    if (server_->security_type() == kVncSecurityVncAuth) {
      // VNC authentication
      for (int i = 0; i < 16; i++) {
        challenge_[i] = rand() % 256;
      }
      send(fd_, challenge_, sizeof(challenge_), 0);
      state_ = kVncAuth;
    } else {
      state_ = kVncInit;
      uint32_t status = 0;
      send(fd_, &status, sizeof(status), 0);
    }
    break;
  }
  case kVncAuth: {
    if (!CheckClientAuth(buf, n)) {
      MV_WARN("VNC auth failed");
      const char failed_string[] = "Authentication failed";
      *(uint32_t*)&buf[0] = htonl(1);
      *(uint32_t*)&buf[4] = htonl(sizeof(failed_string));
      send(fd_, buf, 8, 0);
      send(fd_, failed_string, sizeof(failed_string), 0);
      return false;
    }
    state_ = kVncInit;
    uint32_t status = 0;
    send(fd_, &status, sizeof(status), 0);
    break;
  }
  case kVncInit: {
    shared_ = buf[0] == 1;
    if (!shared_) {
      server_->SetExclusiveConnnction(this);
    }

    LookupDevices();
    if (display_ == nullptr) {
      return false;
    }
    state_ = kVncRunning;
    SendServerInit();
    break;
  }
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
  int bpp;
  display_->GetDisplayMode(&frame_buffer_width_, &frame_buffer_height_, &bpp, nullptr);
  ResetFrameBuffer();

  // z_stream initialization
  zstream_.zalloc = Z_NULL;
  zstream_.zfree = Z_NULL;
  zstream_.opaque = Z_NULL;
  MV_ASSERT(deflateInit2(&zstream_, Z_DEFAULT_COMPRESSION, Z_DEFLATED,
    MAX_WBITS, MAX_MEM_LEVEL, Z_DEFAULT_STRATEGY) == Z_OK);

  ServerInitMessage msg = {
    .framebuffer_width = htons(frame_buffer_width_),
    .framebuffer_height = htons(frame_buffer_height_),
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
  display_->Refresh();
}

bool VncConnection::OnSetPixelFormat() {
  ReadSkip(3); // padding

  uint8_t pixel_format[16];
  ReadData(pixel_format, sizeof(pixel_format));
  if (memcmp(&pixel_format_, pixel_format, sizeof(pixel_format)) == 0) {
    return true;
  }

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

  ResetFrameBuffer();
  display_->Refresh();
  return true;
}


bool VncConnection::OnSetEncodings() {
  ReadSkip(1); // padding

  client_encodings_.clear();
  auto n_encodings = ReadUInt16();
  for (int i = 0; i < n_encodings; i++) {
    auto encoding = (int32_t)ReadUInt32();
    client_encodings_.push_back(encoding);
  }

  // Set preferred encoding
  for (auto encoding : client_encodings_) {
    if (encoding == 6 || encoding == 0) {
      preferred_encoding_ = encoding;
      break;
    }
  }

  // Encoding RAW must be supported
  if (!IsEncodingSupported(0)) {
    MV_WARN("VNC encoding RAW not supported");
    return false;
  }
  return true;
}

bool VncConnection::IsEncodingSupported(int32_t encoding) {
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
    AddDirtyRect(req.y, req.x, req.y + req.h, req.x + req.w);
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
  auto keyboard = GetActiveKeyboard();
  if (keyboard) {
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
    if (keyboard->InputAcceptable()) {
      keyboard->QueueKeyboardEvent(transcoded, modifiers_);
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

KeyboardInputInterface* VncConnection::GetActiveKeyboard() {
  if (server_->machine()->IsPaused())
    return nullptr;
  for (auto keyboard : keyboards_) {
    if (keyboard->InputAcceptable()) {
      return keyboard;
    }
  }
  return nullptr;
}

void VncConnection::OnDisplayModeChange() {
  int w, h, bpp;
  display_->GetDisplayMode(&w, &h, &bpp, nullptr);
  frame_buffer_width_ = w;
  frame_buffer_height_ = h;

  ResetFrameBuffer();
  frame_buffer_resize_requested_ = true;
  update_cv_.notify_one();
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

void VncConnection::ResetFrameBuffer() {
  if (frame_buffer_) {
    pixman_image_unref(frame_buffer_);
    frame_buffer_ = nullptr;
    dirty_rects_.clear();
  }

  auto bytes_per_pixel = pixel_format_.bits_per_pixel / 8;
  auto code = GetPixelFormat(pixel_format_.bits_per_pixel);
  auto stride_bytes = frame_buffer_width_ * bytes_per_pixel;
  frame_buffer_ = pixman_image_create_bits(code, frame_buffer_width_, frame_buffer_height_, nullptr, stride_bytes);
  MV_ASSERT(frame_buffer_);
  /* After reset frame buffer, a full screen update is required */
  AddDirtyRect(0, 0, frame_buffer_height_, frame_buffer_width_);
}

void VncConnection::RenderSurface(const DisplayPartialBitmap* partial) {
  auto bytes_per_pixel = pixel_format_.bits_per_pixel / 8;
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
      if (cursor_buffer_) {
        pixman_image_unref(cursor_buffer_);
      }
      cursor_buffer_ = pixman_image_create_bits(PIXMAN_a8b8g8r8, 4, 4, nullptr, 4 * 4);
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

void VncConnection::OnDisplayUpdate(const DisplayUpdate& update) {
  for (auto& partial : update.partials) {
    RenderSurface(&partial);
  }
  RenderCursor(&update.cursor);

  if (frame_buffer_update_requested_ && (!dirty_rects_.empty() || cursor_update_requested_)) {
    update_cv_.notify_one();
  }
}

void VncConnection::BuildDesktopSize(std::vector<std::string>& updates) {
  if (!IsEncodingSupported(-223)) {
    MV_WARN("DesktopSize pseudo-encoding not supported");
    return;
  }
  std::string buf(12, 0);
  auto ptr = (uint8_t*)buf.data();
  *(uint16_t*)&ptr[4] = htons(frame_buffer_width_);
  *(uint16_t*)&ptr[6] = htons(frame_buffer_height_);
  *(uint32_t*)&ptr[8] = htonl(-223); // DesktopSize pseudo-encoding
  updates.emplace_back(std::move(buf));
}

void VncConnection::BuildCursorUpdate(std::vector<std::string>& updates) {
  if (!cursor_buffer_) {
    return;
  }

  auto data = (uint8_t*)pixman_image_get_data(cursor_buffer_);
  int height = pixman_image_get_height(cursor_buffer_);
  int width = pixman_image_get_width(cursor_buffer_);
  int stride = pixman_image_get_stride(cursor_buffer_);
  int bytes_per_pixel = 4; // As we have already converted cursor to A8B8G8R8

  if (IsEncodingSupported(-314)) { // Alpha cursor pseudo-encoding
    std::string buf(16, 0);
    auto ptr = (uint8_t*)buf.data();
    *(uint16_t*)&ptr[0] = htons(cursor_hotspot_x_);
    *(uint16_t*)&ptr[2] = htons(cursor_hotspot_y_);
    *(uint16_t*)&ptr[4] = htons(width);
    *(uint16_t*)&ptr[6] = htons(height);
    *(uint32_t*)&ptr[8] = htonl(-314);
    *(uint32_t*)&ptr[12] = htonl(0); // RAW encoding of cursor

    for (int y = 0; y < height; y++) {
      buf.append((char*)data + y * stride, width * bytes_per_pixel);
    }
    updates.emplace_back(std::move(buf));
    return;
  }

  if (IsEncodingSupported(-239)) { // Cursor pseudo-encoding
    std::string buf(12, 0);
    auto ptr = (uint8_t*)buf.data();
    *(uint16_t*)&ptr[0] = htons(cursor_hotspot_x_);
    *(uint16_t*)&ptr[2] = htons(cursor_hotspot_y_);
    *(uint16_t*)&ptr[4] = htons(width);
    *(uint16_t*)&ptr[6] = htons(height);
    *(uint32_t*)&ptr[8] = htonl(-239);

    for (int y = 0; y < height; y++) {
      buf.append((char*)data + y * stride, width * bytes_per_pixel);
    }

    // mask data is floor((width + 7) / 8) * height bytes
    auto mask_stride = (width + 7) / 8;
    std::string mask(mask_stride * height, 0);
    for (int y = 0; y < height; y++) {
      auto src = data + y * stride;
      auto dst = (uint8_t*)mask.data() + y * mask_stride;
      for (int x = 0; x < width; x++) {
        if (src[bytes_per_pixel * x + 3] == 0xFF) {
          dst[x / 8] |= 1 << (7 - x % 8);
        }
      }
    }
    buf.append(mask);
    updates.emplace_back(std::move(buf));
    return;
  }
}

void VncConnection::BuildFrameBufferUpdate(std::vector<std::string>& updates, int x, int y, int width, int height) {
  if (frame_buffer_ == nullptr) {
    // Frame buffer is destroyed if client set pixel format
    return;
  }

  MV_ASSERT(width > 0 && height > 0);
  MV_ASSERT(x + width <= frame_buffer_width_ && y + height <= frame_buffer_height_);
  auto bytes_per_pixel = pixel_format_.bits_per_pixel / 8;

  std::string buf;
  buf.resize(width * height * bytes_per_pixel * 2);
  auto ptr = (uint8_t*)buf.data();
  *(uint16_t*)&ptr[0] = htons(x);
  *(uint16_t*)&ptr[2] = htons(y);
  *(uint16_t*)&ptr[4] = htons(width);
  *(uint16_t*)&ptr[6] = htons(height);
  
  if (IsEncodingSupported(6)) {
    *(uint32_t*)&ptr[8] = htonl(6); // zlib encoding

    uint32_t* compressed_size_ptr = (uint32_t*)&ptr[12];
    zstream_.next_out = (Bytef*)&ptr[16];
    zstream_.avail_out = buf.size() - 16;

    // fill raw encoding data
    auto stride = pixman_image_get_stride(frame_buffer_);
    auto data = (uint8_t*)pixman_image_get_data(frame_buffer_);
    auto src = data + y * stride + x * bytes_per_pixel;
    for (int i = 0; i < height; i++) {
      zstream_.next_in = src;
      zstream_.avail_in = width * bytes_per_pixel;
      MV_ASSERT(deflate(&zstream_, Z_NO_FLUSH) == Z_OK);
      src += stride;
    }
    
    // Z_SYNC_FLUSH to flush the data
    MV_ASSERT(deflate(&zstream_, Z_SYNC_FLUSH) == Z_OK);

    // fix compressed size
    uint32_t compressed_size = buf.size() - 16 - zstream_.avail_out;
    *compressed_size_ptr = htonl(compressed_size);
    buf.resize(16 + compressed_size);
  } else {
    *(uint32_t*)&ptr[8] = htonl(0); // Raw encoding

    auto data = (uint8_t*)pixman_image_get_data(frame_buffer_);
    auto src_stride = pixman_image_get_stride(frame_buffer_);
    auto dst_stride = width * bytes_per_pixel;
    auto src = data + y * src_stride + x * bytes_per_pixel;
    auto dst = ptr + 12;
    
    for (int i = 0; i < height; i++) {
      memcpy(dst, src, dst_stride);
      src += src_stride;
      dst += dst_stride;
    }
    buf.resize(12 + height * dst_stride);
  }
  updates.emplace_back(std::move(buf));
}

void VncConnection::UpdateLoop() {
  SetThreadName("mvisor-vnc-conn");

  while (state_ == kVncRunning) {
    std::unique_lock<std::mutex> lock(update_mutex_);
    update_cv_.wait(lock, [this]() {
      return state_ != kVncRunning || (frame_buffer_update_requested_ && (
             !dirty_rects_.empty() || cursor_update_requested_ || frame_buffer_resize_requested_
      ));
    });
    if (state_ != kVncRunning) {
      break;
    }
    
    std::vector<std::string> updates;
    if (cursor_update_requested_) {
      cursor_update_requested_ = false;
      BuildCursorUpdate(updates);
    }
    if (frame_buffer_resize_requested_) {
      frame_buffer_resize_requested_ = false;
      BuildDesktopSize(updates);
    } else {
      while (!dirty_rects_.empty()) {
        auto rect = dirty_rects_.back();
        dirty_rects_.pop_back();
        BuildFrameBufferUpdate(updates, rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top);
      }
    }

    if (!updates.empty()) {
      frame_buffer_update_requested_ = false;

      // Send updates
      uint8_t header[4] = {0};
      *(uint16_t*)&header[2] = htons(updates.size());
      send(fd_, header, sizeof(header), 0);
      for (auto& update : updates) {
        send(fd_, update.data(), update.size(), 0);
      }
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
