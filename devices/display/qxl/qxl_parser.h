/* 
 * MVisor QXL Parser
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
#ifndef _MVISOR_DEVICES_QXL_PARSER_H
#define _MVISOR_DEVICES_QXL_PARSER_H

#include "qxl_dev.h"
#include "canvas/draw.h"

#include <functional>
#include <vector>

class Qxl;
class QxlRender;

struct QxlParser {
 public:
  uint32_t      surface_id;
  uint8_t       effect;
  uint8_t       type;
  uint8_t       self_bitmap;
  SpiceRect     self_bitmap_area;
  SpiceImage*   self_bitmap_image;
  SpiceRect     bbox;
  SpiceClip     clip;
  union {
    SpiceFill         fill;
    SpiceOpaque       opaque;
    SpiceCopy         copy;
    SpiceTransparent  transparent;
    SpiceAlphaBlend   alphablend;
    SpiceBlend        blend;
    struct {
      SpicePoint      src_pos;
    } copy_bits;
    SpiceRop3         rop3;
    SpiceStroke       stroke;
    SpiceText         text;
    SpiceBlackness    blackness;
    SpiceInvers       invers;
    SpiceWhiteness    whiteness;
    SpiceComposite    composite;
  } u;

 public:
  QxlParser(QxlRender* render);
  ~QxlParser();

  bool Parse(QXLDrawable* drawable);

 private:
  QxlRender*                render_;
  Qxl*                      qxl_;
  std::vector<void*>        gc_;
  std::vector<SpiceChunks*> chunks_;

  bool ParseDrawOpaque(QXLOpaque* q);
  bool ParseDrawBlend(QXLBlend* q);
  bool ParseDrawBlackness(QXLBlackness* q);
  bool ParseDrawWhiteness(QXLWhiteness* q);
  bool ParseDrawInvers(QXLInvers* q);
  bool ParseDrawRop3(QXLRop3* q);
  bool ParseDrawCopy(QXLCopy* q);
  bool ParseDrawStroke(QXLStroke* q);
  bool ParseDrawText(QXLText* q);
  bool ParseDrawTransparent(QXLTransparent* q);
  bool ParseDrawAlphaBlend(QXLAlphaBlend* q);
  bool ParseCopyBits(QXLCopyBits* q);
  bool ParseDrawFill(QXLFill* q);

  SpiceClipRects* GetClipRects(uint64_t address);
  SpiceImage* GetImage(uint64_t address);
  void GetQMask(SpiceQMask* s, QXLQMask* q);
  void GetBrush(SpiceBrush* s, QXLBrush* q);
  SpiceString* GetString(uint64_t address);
  SpicePath* GetPath(uint64_t address);
};


#endif
