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
#include "logger.h"


#pragma GCC push_options
#pragma GCC target("avx2")
#include <immintrin.h>

/* input buffer must be 256 bytes alignment */
__attribute__((used))
static bool avx2_test_zero(const void* buffer, size_t length) {
  uint64_t start = (uint64_t)buffer;
  MV_ASSERT(!((start | length) & 0xFF));

  auto *p = (__m256i*)buffer;
  auto *e = (__m256i*)(start + length);
  while (p < e) {
    __builtin_prefetch(p);
    __m256i t = p[0] | p[1] | p[2] | p[3] | p[4] | p[5] | p[6] | p[7];
    if (__builtin_expect(!_mm256_testz_si256(t, t), 0))
      return false;
    p += 8;
  }
  return true;
}

#pragma GCC pop_options

#pragma GCC push_options
#pragma GCC target("avx512f")
#include <immintrin.h>

/* input buffer must be 256 bytes alignment */
__attribute__((used))
static bool avx512_test_zero(const void* buffer, size_t length) {
  uint64_t start = (uint64_t)buffer;
  MV_ASSERT(!((start | length) & 0xFF));

  auto *p = (__m512i*)buffer;
  auto *e = (__m512i*)(start + length);
  while (p < e) {
    __builtin_prefetch(p);
    __m512i t = p[0] | p[1] | p[2] | p[3];
    if (__builtin_expect(!!_mm512_test_epi64_mask(t, t), 0))
      return false;
    p += 4;
  }
  return true;
}

#pragma GCC pop_options

bool test_zero(const void* buffer, size_t length) {
  return avx2_test_zero(buffer, length);
}
