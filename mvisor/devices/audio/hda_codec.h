/* 
 * MVisor HDA Codec
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

#ifndef _MVISOR_DEVICES_AUDIO_HDA_CODEC_H
#define _MVISOR_DEVICES_AUDIO_HDA_CODEC_H

#include <cstdint>
#include <functional>
#include <vector>

typedef std::function<void(uint32_t)> ResponseCallback;
typedef std::function<size_t (uint8_t*, size_t)> TransferCallback;

struct HdaCodecBuffer {
  uint8_t*  data;
  size_t    length;
  bool      interrupt_on_completion;
  size_t    read_counter;
};

class HdaCodecInterface {
 public:
  virtual void StartCommand(uint8_t node_id, uint32_t data, ResponseCallback callback) = 0;
  virtual void StartStream(uint32_t stream_id, bool output, TransferCallback callback) = 0;
  virtual void StopStream(uint32_t stream_id, bool output) = 0;
};

#endif // _MVISOR_DEVICES_AUDIO_HDA_CODEC_H
