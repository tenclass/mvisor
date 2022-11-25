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

#include "display_encoder.h"

#include <libyuv.h>

#include "logger.h"


SweetDisplayEncoder::SweetDisplayEncoder(int width, int height, DisplayStreamConfig* config) :
  screen_width_(width), screen_height_(height), config_(config)
{
  /* set screen bpp to ARGB */
  screen_bpp_ = 32;
  /* aligned screen stride by 128 */
  screen_stride_ = screen_width_ * 4;
  if (screen_stride_ % 128)
    screen_stride_ += 128 - (screen_stride_ % 128);

  /* for VGA color convertion */
  screen_bitmap_ = new uint8_t[screen_stride_ * screen_height_];
  InitializeX264();

  /* start encode thread */
  encode_thread_ = std::thread(&SweetDisplayEncoder::EncodeProcess, this);
}

SweetDisplayEncoder::~SweetDisplayEncoder() {
  /* terminate encode thread */
  destroyed_ = true;
  encode_cv_.notify_all();
  if (encode_thread_.joinable()) {
    encode_thread_.join();
  }

  for (auto slice : encode_slices_) {
    delete slice;
  }

  if (x264_) {
    x264_encoder_close(x264_);
  }
  x264_picture_clean(&input_yuv_);
  delete screen_bitmap_;
}

void SweetDisplayEncoder::InitializeX264() {
  auto& param = x264_param_;
  char tune[30];

  // MV_LOG("%ux%ux%u codec=%s profile=%s speed=%s bitrate=%u qmin=%u fps=%u threads=%u flags=0x%x",
  //   screen_width_, screen_height_, screen_bpp_,
  //   config_->codec().c_str(), config_->profile().c_str(), config_->preset().c_str(), config_->bitrate(),
  //   config_->qmin(), config_->fps(), config_->threads(), config_->flags());

  /* check fast decode flag, zerolatency is required */
  if (config_->flags() & 1) {
    strcpy(tune, "zerolatency,fastdecode");
  } else {
    strcpy(tune, "zerolatency");
  }
  if (x264_param_default_preset(&param, config_->preset().c_str(), tune) < 0) {
    MV_PANIC("failed to set default preset %s", config_->preset().c_str());
  }

  if (config_->profile() == "high444") {
    param.i_csp = X264_CSP_I444;
  } else if (config_->profile() == "high422") {
    param.i_csp = X264_CSP_I422;
  } else {
    param.i_csp = X264_CSP_I420;
  }

  param.i_width = screen_width_;
  param.i_height = screen_height_;
  
  /* rate control method is not configurable now */
  param.rc.i_rc_method = X264_RC_CRF;
  param.rc.f_rf_constant = config_->qmin();
  param.rc.i_qp_max = config_->qmin() + 9;
  param.rc.i_vbv_max_bitrate = config_->bitrate() / 1000;
  param.rc.i_vbv_buffer_size = config_->bitrate() * 2 / 1000;

  param.i_fps_num = config_->fps();
  param.i_fps_den = 1;
  param.b_vfr_input = 0;
  param.b_repeat_headers = 1;
  param.b_annexb = 1;
  param.i_log_level = X264_LOG_ERROR;
  param.i_threads = config_->threads();
  param.i_keyint_min = 7200;
  param.i_keyint_max = 7200;
  param.i_scenecut_threshold = 0;

  /* colorspace */
  param.vui.i_colorprim = 1; // BT.709
  param.vui.i_transfer = 13; // sRGB
  param.vui.i_colmatrix = 1; // BT.709

  /* check CABAC and num_ref_frames */
  if (config_->flags() & 2) {
    param.b_cabac = 1;
  }
  if (config_->flags() & 4) {
    param.i_frame_reference = 3;
  }

  if (x264_param_apply_profile(&param, config_->profile().c_str()) < 0) {
    MV_PANIC("failed to set profile %s", config_->profile().c_str());
  }

  if (x264_picture_alloc(&input_yuv_, param.i_csp, param.i_width, param.i_height) < 0) {
    MV_PANIC("failed to allocate yuv picture %dx%d", param.i_width, param.i_height);
  }
}

void SweetDisplayEncoder::Start(OutputCallback callback) {
  std::lock_guard<std::mutex> lock(encode_mutex_);
  force_keyframe_ = true;
  output_callback_ = callback;
}

/* Supports 8 / 24 bit VGA mode */
bool SweetDisplayEncoder::ConvertPartial(DisplayPartialBitmap* partial) {
  /* Only support converting the whole screen */
  if (partial->x || partial->y || partial->width != screen_width_ || partial->height != screen_height_) {
    MV_ERROR("failed to convert x=%u y=%u %ux%u to %ux%u", partial->x, partial->y,
      partial->width, partial->height, screen_width_, screen_height_);
    return false;
  }

  switch(partial->bpp) {
    case 8:
      /* Convert from 8 bit to ARGB */
      for (int y = 0; y < partial->height; y++) {
        auto from = partial->data + partial->stride * y;
        auto to = screen_bitmap_ + screen_stride_ * y;
        for (int x = 0; x < partial->width; x++) {
          auto palette = &partial->palette[from[x] * 3];
          to[0] = palette[2] << 2;
          to[1] = palette[1] << 2;
          to[2] = palette[0] << 2;
          to[3] = 0;
          to += 4;
        }
      }
      break;
    case 16:
      libyuv::RGB565ToARGB(partial->data, partial->stride, screen_bitmap_, screen_stride_, partial->width, partial->height);
      break;
    case 24:
      libyuv::RGB24ToARGB(partial->data, partial->stride, screen_bitmap_, screen_stride_, partial->width, partial->height);
      break;
    default:
      MV_ERROR("cannot convert bpp=%u", partial->bpp);
      return false;
  }
  return true;
}

void SweetDisplayEncoder::Render(std::vector<DisplayPartialBitmap>& partials) {
  std::lock_guard<std::mutex> lock(encode_mutex_);
  for (auto& partial : partials) {
    // MV_LOG("partial %d,%d %dx%dx%d stride=%d", partial.x, partial.y,
    //   partial.width, partial.height, partial.bpp, partial.stride);

    if (partial.bpp == 32) {
      CreateEncodeSlice(partial.data, partial.stride, partial.x, partial.y, partial.width, partial.height);
    } else {
      if (ConvertPartial(&partial)) {
        uint8_t* src = screen_bitmap_ + screen_stride_ * partial.y + partial.x * 4;
        CreateEncodeSlice(src, screen_stride_, partial.x, partial.y, partial.width, partial.height);
      }
    }
  }

  if (!encode_slices_.empty()) {
    encode_cv_.notify_all();
  }
}

void SweetDisplayEncoder::CreateEncodeSlice(uint8_t* src, int stride, int x, int y, int w, int h) {
  if (!w || !h) {
    return;
  }
  if ((x & 1) || (y & 1) || (w & 1) || (h & 1)) {
    MV_ERROR("invalid slice (%d,%d) %dx%d", x, y, w, h);
    return;
  }
  if (x + w > screen_width_ || y + h > screen_height_) {
    MV_ERROR("overflow (%d,%d) screen=%dx%d", x + w, y + h, screen_width_, screen_height_);
    return;
  }

  EncodeSlice* slice = new EncodeSlice;
  slice->x = x;
  slice->y = y;
  slice->width = w;
  slice->height = h;
  
  /* convert to YUV */
  if (x264_picture_alloc(&slice->yuv, input_yuv_.img.i_csp, w, h) < 0) {
    MV_PANIC("failed to allocate yuv slice %dx%d", w, h);
    return;
  }

  auto dst = &slice->yuv.img;
  /* libyuv here must be modified to use BT.709 */
  switch (dst->i_csp)
  {
  case X264_CSP_I420:
    libyuv::ARGBToI420(src, stride,
      dst->plane[0], dst->i_stride[0],
      dst->plane[1], dst->i_stride[1],
      dst->plane[2], dst->i_stride[2],
      slice->width, slice->height);
    break;
  case X264_CSP_I444:
    libyuv::ARGBToI444(src, stride,
      dst->plane[0], dst->i_stride[0],
      dst->plane[1], dst->i_stride[1],
      dst->plane[2], dst->i_stride[2],
      slice->width, slice->height);
    break;
  default:
    MV_PANIC("unsupported csp=0x%x", dst->i_csp);
  }
  
  encode_slices_.push_back(slice);
}

void SweetDisplayEncoder::EncodeProcess() {
  SetThreadName("sweet-encoder");

  // if threads > 1, the encoder takes "sweet-encoder" as new thread name
  x264_ = x264_encoder_open(&x264_param_);
  MV_ASSERT(x264_);

  uint average_packet_size = config_->bitrate() / config_->fps() / 8;
  auto idle_interval = std::chrono::milliseconds(500);
  auto frame_interval = std::chrono::microseconds(1000000 / config_->fps());
  auto next_encode_time = std::chrono::steady_clock::now() + frame_interval * 0.0;

  while (!destroyed_) {
    std::unique_lock<std::mutex> lock(encode_mutex_);
    encode_cv_.wait_for(lock, idle_interval, [this]() {
      return destroyed_ || !encode_slices_.empty();
    });

    if (destroyed_)
      break;

    auto start_time = std::chrono::steady_clock::now();

    if (encode_slices_.empty()) {
      lock.unlock();
    } else {
      auto copied(encode_slices_);
      encode_slices_.clear();
      lock.unlock();

      DrawSlices(copied);
    }
  
    Encode();

    lock.lock();
    if (output_callback_) {
      output_callback_(output_nal_->p_payload, output_nal_size_);
    }

    /* Calculate next frame time point. Control bitrate by limiting fps */
    double overhead = double(output_nal_size_) / average_packet_size;
    if (overhead > 1.5 && overhead < config_->fps()) {
      next_encode_time = start_time + frame_interval * (overhead - 1.0);
    } else {
      next_encode_time = start_time + frame_interval;
    }
    if (std::chrono::steady_clock::now() < next_encode_time) {
      std::this_thread::sleep_until(next_encode_time);
    }
  }
}

void SweetDisplayEncoder::DrawSlices(std::vector<EncodeSlice*>& slices) {
  auto& dst = input_yuv_.img;
  uint log2_chroma_w = 0, log2_chroma_h = 0;

  switch (dst.i_csp)
  {
  case X264_CSP_I420:
    log2_chroma_w = 1;
    log2_chroma_h = 1;
    break;
  case X264_CSP_I444:
    log2_chroma_w = 0;
    log2_chroma_h = 0;
    break;
  default:
    MV_PANIC("unsupported csp=0x%x", dst.i_csp);
  }

  for (auto slice: slices) {
    auto& src = slice->yuv.img;
    uint8_t* to, *from;

    // copy Y bits
    from = src.plane[0];
    to = dst.plane[0] + dst.i_stride[0] * slice->y + slice->x;
    for (uint j = 0; j < slice->height; j++) {
      memcpy(to, from, slice->width);
      to += dst.i_stride[0];
      from += src.i_stride[0];
    }

    // copy U bits
    to = dst.plane[1] + dst.i_stride[1] * (slice->y >> log2_chroma_h) + (slice->x >> log2_chroma_w);
    from = src.plane[1];
    for (uint j = 0; j < (slice->height >> log2_chroma_h); j++) {
      memcpy(to, from, slice->width >> log2_chroma_w);
      to += dst.i_stride[1];
      from += src.i_stride[1];
    }

    // copy V bits
    to = dst.plane[2] + dst.i_stride[2] * (slice->y >> log2_chroma_h) + (slice->x >> log2_chroma_w);
    from = src.plane[2];
    for (uint j = 0; j < (slice->height >> log2_chroma_h); j++) {
      memcpy(to, from, slice->width >> log2_chroma_w);
      to += dst.i_stride[2];
      from += src.i_stride[2];
    }
  
    // cleanup
    x264_picture_clean(&slice->yuv);
    delete slice;
  }
}

void SweetDisplayEncoder::Encode() {
  ++input_yuv_.i_pts;
  if (force_keyframe_) {
    input_yuv_.i_type = X264_TYPE_KEYFRAME;
    force_keyframe_ = false;
  } else {
    input_yuv_.i_type = X264_TYPE_AUTO;
  }

  output_nal_size_ = x264_encoder_encode(x264_, &output_nal_, &output_nal_sequence_, &input_yuv_, &output_yuv_);
  if (output_nal_size_ < 0)
    return;
}

void SweetDisplayEncoder::ForceKeyframe() {
  force_keyframe_ = true;
  encode_cv_.notify_all();
}
