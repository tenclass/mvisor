#ifndef _MVISOR_VNC_CONNECTION_H
#define _MVISOR_VNC_CONNECTION_H

#include <zlib.h>
#include <pixman.h>
#include "server.h"

enum VncConnectionState {
  kVncVersion,
  kVncSecurity,
  kVncAuth,
  kVNcInit,
  kVncRunning,
  kVncClosed,
};

struct PixelFormat {
  uint8_t bits_per_pixel;
  uint8_t depth;
  uint8_t big_endian;
  uint8_t true_color;
  uint16_t red_max;
  uint16_t green_max;
  uint16_t blue_max;
  uint8_t red_shift;
  uint8_t green_shift;
  uint8_t blue_shift;
  uint8_t padding[3];
} __attribute__((packed));

struct VncRect {
    int top;
    int left;
    int bottom;
    int right;
};

class VncConnection {
 private:
  VncConnectionState  state_ = kVncVersion;
  VncServer*          server_;
  int                 fd_;
  uint8_t             challenge_[16];
  bool                shared_ = false;
  PixelFormat         pixel_format_;
  std::vector<int32_t>client_encodings_;
  int                 preferred_encoding_ = 0;
  int                 frame_buffer_width_ = 0;
  int                 frame_buffer_height_ = 0;
  pixman_image_t*     frame_buffer_ = nullptr;
  pixman_image_t*     cursor_buffer_ = nullptr;
  uint64_t            cursor_shape_id_ = 0;
  int                 cursor_hotspot_x_ = 0;
  int                 cursor_hotspot_y_ = 0;
  bool                frame_buffer_update_requested_ = false;
  bool                frame_buffer_resize_requested_ = false;
  bool                cursor_update_requested_ = false;
  std::vector<VncRect>dirty_rects_;
  uint8_t             modifiers_ = 0;

  std::thread               update_thread_;
  std::recursive_mutex      update_mutex_;
  std::condition_variable   update_cv_;
  z_stream                  zstream_ = {};

  ClipboardInterface*       clipboard_ = nullptr;
  std::string               clipboard_data_;
  DisplayInterface*         display_ = nullptr;
  KeyboardInputInterface*   keyboard_ = nullptr;
  std::vector<PointerInputInterface*>   pointers_;

  std::list<DisplayModeChangeListener>::iterator  display_mode_listener_;
  std::list<DisplayUpdateListener>::iterator      display_update_listener_;
  std::list<ClipboardListener>::iterator          clipboard_listener_;

  bool CheckClientAuth(const char* buffer, ssize_t length);
  void CreateFrameBuffer();
  void DestroyFrameBuffer();
  void SendServerInit();
  void LookupDevices();
  void ResizeFrameBuffer();
  bool SendDesktopSize();
  bool SendCursorUpdate();
  bool SendFrameBufferUpdate(int x, int y, int width, int height);
  void AddDirtyRectInternal(int top, int left, int bottom, int right);
  void AddDirtyRect(int top, int left, int bottom, int right);
  void UpdateLoop();
  pixman_format_code_t GetPixelFormat(int bpp);

  uint8_t ReadUInt8();
  uint16_t ReadUInt16();
  uint32_t ReadUInt32();
  void ReadSkip(size_t size);
  void ReadData(void* data, size_t size);
  bool OnSetPixelFormat();
  bool OnSetEncodings();
  bool OnFrameBufferUpdateRequest();
  bool OnKeyEvent();
  bool OnPointerEvent();
  bool OnClientCutText();
  bool IsEncodingSupported(int32_t encoding);

  void OnClipboardFromGuest(const ClipboardData& clipboard_data);
  void Render(const DisplayUpdate& update);
  void RenderSurface(const DisplayPartialBitmap* partial);
  void RenderCursor(const DisplayMouseCursor* cursor_update);
  PointerInputInterface* GetActivePointer();

 public:
  VncConnection(VncServer* server, int fd);
  ~VncConnection();
  bool OnReceive();

  int fd() { return fd_; }
};

#endif
