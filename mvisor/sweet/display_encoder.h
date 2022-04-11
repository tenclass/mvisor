/* 
 * MVisor - Sweet Renderer
 * Copyright (C) 2022 Terrence <terrence@tenclass.com>
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


#ifndef _MVISOR_SWEET_DISPLAY_ENCODER_H
#define _MVISOR_SWEET_DISPLAY_ENCODER_H

#include <functional>
#include <thread>
#include <mutex>
#include <vector>
#include <condition_variable>

#include <x264.h>

#include "device_interface.h"
#include "pb/sweet.pb.h"

using namespace SweetProtocol;

typedef std::function<void (void* data, size_t length)> OutputCallback;

struct EncodeSlice {
  uint            x;
  uint            y;
  uint            width;
  uint            height;
  x264_picture_t  yuv;
};

class SweetDisplayEncoder {
 public:
  SweetDisplayEncoder(uint width, uint height, uint bpp, uint stride, DisplayStreamConfig* config);
  ~SweetDisplayEncoder();

  void Render(std::vector<DisplayPartialBitmap>& partials);
  void Start(OutputCallback output_cb);
  void Stop();
  void ForceKeyframe();
 private:
  void SetDefaultConfig();
  void EncodeProcess();
  void InitializeX264();
  void RenderPartial(DisplayPartialBitmap* partial);
  void CreateEncodeSlice(uint top, uint left, uint bottom, uint right);
  void ConvertSlices();
  void DrawSlices(std::vector<EncodeSlice*>& slices);
  void Encode();

  bool                        destroyed_ = false;
  bool                        started_ = false;
  bool                        force_keyframe_ = false;
  uint                        screen_width_, screen_height_, screen_bpp_, screen_stride_;
  uint8_t*                    screen_bitmap_ = nullptr;
  DisplayStreamConfig*        config_;
  OutputCallback              output_callback_;

  std::thread                 encode_thread_;
  mutable std::mutex          encode_mutex_;
  std::condition_variable     encode_cv_;
  std::vector<EncodeSlice*>   encode_slices_;

  x264_t*                     x264_;
  x264_picture_t              input_yuv_;
  x264_picture_t              output_yuv_;
  x264_nal_t*                 output_nal_;
  int                         output_nal_size_;
  int                         output_nal_sequence_;
};

#endif // _MVISOR_SWEET_DISPLAY_ENCODER_H
