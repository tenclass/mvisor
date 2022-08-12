/* 
 * MVisor
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

#include "utilities.h"

#include <cstdint>
#include <cerrno>
#include <zstd.h>
#include <zstd_errors.h>

#include "logger.h"


ssize_t zstd_decompress(const void* src,  size_t src_size, void* dest, size_t dest_size) {
  size_t zstd_ret = 0;
  ssize_t ret = 0;
  ZSTD_outBuffer output = {
    .dst = dest,
    .size = dest_size,
    .pos = 0
  };
  ZSTD_inBuffer input = {
    .src = src,
    .size = src_size,
    .pos = 0
  };
  ZSTD_DCtx* dctx = ZSTD_createDCtx();
  MV_ASSERT(dctx);

  while (output.pos < output.size) {
    size_t last_in_pos = input.pos;
    size_t last_out_pos = output.pos;
    zstd_ret = ZSTD_decompressStream(dctx, &output, &input);

    if (ZSTD_isError(zstd_ret)) {
      MV_LOG("ZSTD decompress error inpos=%d outpos=%d: %s", last_in_pos, last_out_pos,
        ZSTD_getErrorString(ZSTD_getErrorCode(zstd_ret)));
      DumpHex((uint8_t*)src + last_in_pos, 64);
      ret = -EIO;
      break;
    }

    /* prevent infinitely waiting for input data */
    if (last_in_pos >= input.pos && last_out_pos >= output.pos) {
      ret = -EIO;
    }
  }
  
  /* make sure no more data pending output */
  if (zstd_ret > 0) {
    ret = -EIO;
  }
  ZSTD_freeDCtx(dctx);
  return ret;
}
