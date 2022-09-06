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

#include <string>

#include "qxl_parser.h"
#include "qxl_render.h"
#include "qxl.h"
#include "logger.h"

static inline void parse_rect(SpiceRect* s, QXLRect* q) {
  s->top = q->top;
  s->bottom = q->bottom;
  s->left = q->left;
  s->right = q->right;
}

static inline void parse_point(SpicePoint* s, QXLPoint* q) {
  s->x = q->x;
  s->y = q->y;
}

static inline bool is_valid_rect(SpiceRect* s) {
  return s->top >= 0 && s->left >= 0 && s->top <= s->bottom && s->left <= s->right;
}

QxlParser::QxlParser(QxlRender* render) {
  render_ = render;
  qxl_ = render->qxl();
}

QxlParser::~QxlParser() {
  for (auto ptr : gc_) {
    free(ptr);
  }
  for (auto chunks : chunks_) {
    spice_chunks_destroy(chunks);
  }
}

bool QxlParser::Parse(QXLDrawable* q) {
  surface_id = q->surface_id;
  type = q->type;
  effect = q->effect;

  parse_rect(&bbox, &q->bbox);
  if (!is_valid_rect(&bbox)) {
    return false;
  }

  self_bitmap = q->self_bitmap;
  parse_rect(&self_bitmap_area, &q->self_bitmap_area);

  clip.type = q->clip.type;
  if (clip.type == SPICE_CLIP_TYPE_RECTS) {
    clip.rects = GetClipRects(q->clip.data);
  } else {
    clip.rects = nullptr;
  }

  bool ret;
  switch (type)
  {
  case QXL_DRAW_FILL:
    ret = ParseDrawFill(&q->u.fill);
    break;
  case QXL_DRAW_OPAQUE:
    ret = ParseDrawOpaque(&q->u.opaque);
    break;
  case QXL_DRAW_COPY:
    ret = ParseDrawCopy(&q->u.copy);
    break;
  case QXL_COPY_BITS:
    ret = ParseCopyBits(&q->u.copy_bits);
    break;
  case QXL_DRAW_BLEND:
    ret = ParseDrawBlend(&q->u.blend);
    break;
  case QXL_DRAW_BLACKNESS:
    ret = ParseDrawBlackness(&q->u.blackness);
    break;
  case QXL_DRAW_WHITENESS:
    ret = ParseDrawWhiteness(&q->u.whiteness);
    break;
  case QXL_DRAW_INVERS:
    ret = ParseDrawInvers(&q->u.invers);
    break;
  case QXL_DRAW_ROP3:
    ret = ParseDrawRop3(&q->u.rop3);
    break;
  case QXL_DRAW_STROKE:
    ret = ParseDrawStroke(&q->u.stroke);
    break;
  case QXL_DRAW_TEXT:
    ret = ParseDrawText(&q->u.text);
    break;
  case QXL_DRAW_TRANSPARENT:
    ret = ParseDrawTransparent(&q->u.transparent);
    break;
  case QXL_DRAW_ALPHA_BLEND:
    ret = ParseDrawAlphaBlend(&q->u.alpha_blend);
    break;
  default:
    MV_ERROR("unhandled drawable type=%d", type);
    ret = false;
  }

  return ret;
}

SpiceClipRects* QxlParser::GetClipRects(uint64_t address) {
  auto q = (QXLClipRects*)qxl_->GetMemSlotAddress(address);
  auto r = (SpiceClipRects*)spice_malloc_n_m(q->num_rects, sizeof(SpiceRect), sizeof(SpiceClipRects));
  gc_.push_back(r);

  r->num_rects = q->num_rects;

  std::string data;
  qxl_->GetMemSlotLinearizedChunkData(&q->chunk, data);
  auto q_rects = (QXLRect*)data.data();

  for (uint i = 0; i < r->num_rects; i++) {
    parse_rect(&r->rects[i], &q_rects[i]);
  }
  
  return r;
}

SpiceImage* QxlParser::GetImage(uint64_t address) {
  if (address == 0) {
    return nullptr;
  }

  auto q = (QXLImage*)qxl_->GetMemSlotAddress(address);
  auto r = (SpiceImage*)spice_new0(SpiceImage, 1);
  gc_.push_back(r);

  r->descriptor.id = q->descriptor.id;
  r->descriptor.type = q->descriptor.type;
  r->descriptor.flags = 0;
  if (q->descriptor.flags & QXL_IMAGE_HIGH_BITS_SET)
    r->descriptor.flags |= SPICE_IMAGE_FLAGS_HIGH_BITS_SET;
  if (q->descriptor.flags & QXL_IMAGE_CACHE)
    r->descriptor.flags |= SPICE_IMAGE_FLAGS_CACHE_ME;
  
  r->descriptor.width = q->descriptor.width;
  r->descriptor.height = q->descriptor.height;

  /* Try to find image from cache */
  if (render_->images().Contains(r->descriptor.id)) {
    r->descriptor.type = SPICE_IMAGE_TYPE_FROM_CACHE;
    r->descriptor.flags = 0;
    return r;
  }

  if (r->descriptor.type == SPICE_IMAGE_TYPE_BITMAP) {
    r->u.bitmap.format = q->bitmap.format;
    r->u.bitmap.x = q->bitmap.x;
    r->u.bitmap.y = q->bitmap.y;
    r->u.bitmap.stride = q->bitmap.stride;
    r->u.bitmap.flags = 0;
    
    if (q->bitmap.flags & QXL_BITMAP_TOP_DOWN) {
      r->u.bitmap.flags |= SPICE_BITMAP_FLAGS_TOP_DOWN;
    }
    
    if (q->bitmap.palette) {
      auto qp = (QXLPalette*)qxl_->GetMemSlotAddress(q->bitmap.palette);
      auto sp = (SpicePalette*)spice_malloc_n_m(qp->num_ents, sizeof(uint32_t), sizeof(SpicePalette));
      gc_.push_back(sp);

      sp->unique = qp->unique;
      sp->num_ents = qp->num_ents;
      for (uint i = 0; i < sp->num_ents; i++) {
        sp->ents[i] = qp->ents[i];
      }
      r->u.bitmap.palette = sp;
      r->u.bitmap.palette_id = sp->unique;
    }
    
    MV_ASSERT(!(q->bitmap.flags & QXL_BITMAP_DIRECT));

    std::vector<iovec> v;
    size_t data_size = qxl_->GetMemSlotChunkData(q->bitmap.data, v);
    auto chunks = (SpiceChunks*)spice_chunks_new(v.size());
    chunks_.push_back(chunks);

    chunks->data_size = data_size;
    for (size_t i = 0; i < chunks->num_chunks; i++) {
      chunks->chunk[i].data = (uint8_t*)v[i].iov_base;
      chunks->chunk[i].len = v[i].iov_len;
    }
    r->u.bitmap.data = chunks;
  } else {
    MV_PANIC("unhandled descriptor type=%d", r->descriptor.type);
  }
  return r;
}

void QxlParser::GetQMask(SpiceQMask* s, QXLQMask* q) {
  s->bitmap = GetImage(q->bitmap);
  if (s->bitmap) {
    s->flags = q->flags;
    parse_point(&s->pos, &q->pos);
  } else {
    s->flags = 0;
    s->pos.x = 0;
    s->pos.y = 0;
  }
}

void QxlParser::GetBrush(SpiceBrush* s, QXLBrush* q) {
  s->type = q->type;
  if (q->type == SPICE_BRUSH_TYPE_SOLID) {
    s->u.color = q->u.color;
  } else if (q->type == SPICE_BRUSH_TYPE_PATTERN) {
    s->u.pattern.pat = GetImage(q->u.pattern.pat);
    parse_point(&s->u.pattern.pos, &q->u.pattern.pos);
  }
}

SpiceString* QxlParser::GetString(uint64_t address) {
  auto q = (QXLString*)qxl_->GetMemSlotAddress(address);
  std::string data;
  qxl_->GetMemSlotLinearizedChunkData(&q->chunk, data);
  MV_ASSERT(data.size() == q->data_size);

  uint bpp = 0;
  if (q->flags & SPICE_STRING_FLAGS_RASTER_A1) {
    bpp = 1;
  } else if (q->flags & SPICE_STRING_FLAGS_RASTER_A4) {
    bpp = 4;
  } else if (q->flags & SPICE_STRING_FLAGS_RASTER_A8) {
    bpp = 8;
  }
  MV_ASSERT(bpp != 0);

  std::vector<SpiceRasterGlyph*> glyphs;
  auto start = (QXLRasterGlyph*)data.data();
  auto end = (QXLRasterGlyph*)(data.data() + data.size());
  while (start < end) {
    size_t glyph_size = start->height * ((start->width * bpp + 7U) / 8U);
    auto spice_glyph = (SpiceRasterGlyph*)spice_malloc_n_m(glyph_size, 1, sizeof(SpiceRasterGlyph));
    gc_.push_back(spice_glyph);

    spice_glyph->width = start->width;
    spice_glyph->height = start->height;
    parse_point(&spice_glyph->render_pos, &start->render_pos);
    parse_point(&spice_glyph->glyph_origin, &start->glyph_origin);
    memcpy(spice_glyph->data, start->data, glyph_size);

    glyphs.push_back(spice_glyph);
    start = (QXLRasterGlyph*)&start->data[glyph_size];
  }

  auto s = (SpiceString*)spice_malloc_n_m(glyphs.size(), sizeof(SpiceRasterGlyph*), sizeof(SpiceString));
  gc_.push_back(s);

  s->length = q->length;
  s->flags = q->flags;
  for (size_t i = 0; i < glyphs.size(); i++) {
    s->glyphs[i] = glyphs[i];
  }
  return s;
}

SpicePath* QxlParser::GetPath(uint64_t address) {
  auto q = (QXLPath*)qxl_->GetMemSlotAddress(address);
  std::string data;
  qxl_->GetMemSlotLinearizedChunkData(&q->chunk, data);
  MV_ASSERT(data.size() == q->data_size);
  
  std::vector<SpicePathSeg*> path_segs;
  auto start = (QXLPathSeg*)data.data();
  auto end = (QXLPathSeg*)(data.data() + data.size());
  while (start < end) {
    auto spice_seg = (SpicePathSeg*)spice_malloc_n_m(start->count, sizeof(SpicePointFix), sizeof(SpicePathSeg));
    gc_.push_back(spice_seg);

    spice_seg->flags = start->flags;
    spice_seg->count = start->count;
    for (uint i = 0; i < spice_seg->count; i++) {
      spice_seg->points[i].x = start->points[i].x;
      spice_seg->points[i].y = start->points[i].y;
    }

    path_segs.push_back(spice_seg);
    start = (QXLPathSeg*)&start->points[start->count];
  }

  auto s = (SpicePath*)spice_malloc_n_m(path_segs.size(), sizeof(SpicePathSeg*), sizeof(SpicePath));
  gc_.push_back(s);

  s->num_segments = path_segs.size();
  for (size_t i = 0; i < s->num_segments; i++) {
    s->segments[i] = path_segs[i];
  }
  return s;
}

bool QxlParser::ParseDrawCopy(QXLCopy* q) {
  SpiceCopy* s = &u.copy;
  s->src_bitmap = GetImage(q->src_bitmap);
  if (!s->src_bitmap) {
    return false;
  }
  parse_rect(&s->src_area, &q->src_area);

  s->rop_descriptor = q->rop_descriptor;
  s->scale_mode = q->scale_mode;

  GetQMask(&s->mask, &q->mask);
  return true;
}

bool QxlParser::ParseDrawOpaque(QXLOpaque* q) {
  SpiceOpaque* s = &u.opaque;
  s->src_bitmap = GetImage(q->src_bitmap);
  if (!s->src_bitmap)
    return false;
  parse_rect(&s->src_area, &q->src_area);

  GetBrush(&s->brush, &q->brush);

  s->rop_descriptor = q->rop_descriptor;
  s->scale_mode = q->scale_mode;

  GetQMask(&s->mask, &q->mask);
  return true;
}

bool QxlParser::ParseDrawFill(QXLFill* q) {
  SpiceFill* s = &u.fill;
  GetBrush(&s->brush, &q->brush);
  s->rop_descriptor = q->rop_descriptor;
  GetQMask(&s->mask, &q->mask);
  return true;
}

bool QxlParser::ParseDrawStroke(QXLStroke* q) {
  SpiceStroke* s = &u.stroke;
  s->path = GetPath(q->path);
  if (!s->path) {
    return false;
  }

  s->back_mode = q->back_mode;
  s->fore_mode = q->fore_mode;
  GetBrush(&s->brush, &q->brush);

  s->attr.flags = q->attr.flags;
  if (s->attr.flags & SPICE_LINE_FLAGS_STYLED) {
    s->attr.style_nseg = q->attr.style_nseg;
    auto qs = (uint8_t*)qxl_->GetMemSlotAddress(q->attr.style);
    s->attr.style = (SPICE_FIXED28_4*)spice_malloc_n(q->attr.style_nseg, sizeof(SPICE_FIXED28_4));
    gc_.push_back(s->attr.style);

    memcpy(s->attr.style, qs, s->attr.style_nseg * sizeof(QXLFIXED));
  } else {
    s->attr.style_nseg = 0;
    s->attr.style = nullptr;
  }
  return true;
}

bool QxlParser::ParseDrawText(QXLText* q) {
  SpiceText* s = &u.text;
  s->back_mode = q->back_mode;
  s->fore_mode = q->fore_mode;
  parse_rect(&s->back_area, &q->back_area);
  GetBrush(&s->back_brush, &q->back_brush);
  GetBrush(&s->fore_brush, &q->fore_brush);
  s->str = GetString(q->str);
  return true;
}

bool QxlParser::ParseDrawTransparent(QXLTransparent* q) {
  SpiceTransparent* s = &u.transparent;
  s->src_bitmap = GetImage(q->src_bitmap);
  if (!s->src_bitmap)
    return false;
  parse_rect(&s->src_area, &q->src_area);

  s->src_color = q->src_color;
  s->true_color = q->true_color;
  return true;
}

bool QxlParser::ParseDrawAlphaBlend(QXLAlphaBlend* q) {
  SpiceAlphaBlend* s = &u.alphablend;
  s->src_bitmap = GetImage(q->src_bitmap);
  if (!s->src_bitmap)
    return false;
  parse_rect(&s->src_area, &q->src_area);

  s->alpha_flags = q->alpha_flags;
  s->alpha = q->alpha;
  return true;
}

bool QxlParser::ParseCopyBits(QXLCopyBits* q) {
  parse_point(&u.copy_bits.src_pos, &q->src_pos);
  return true;
}

bool QxlParser::ParseDrawBlend(QXLBlend* q) {
  SpiceBlend* s = &u.blend;
  s->src_bitmap = GetImage(q->src_bitmap);
  if (!s->src_bitmap)
    return false;
  parse_rect(&s->src_area, &q->src_area);
  
  s->rop_descriptor = q->rop_descriptor;
  s->scale_mode = q->scale_mode;

  GetQMask(&s->mask, &q->mask);
  return true;
}

bool QxlParser::ParseDrawBlackness(QXLBlackness* q) {
  SpiceBlackness* s = &u.blackness;
  GetQMask(&s->mask, &q->mask);
  return true;
}

bool QxlParser::ParseDrawWhiteness(QXLWhiteness* q) {
  SpiceWhiteness* s = &u.whiteness;
  GetQMask(&s->mask, &q->mask);
  return true;
}

bool QxlParser::ParseDrawInvers(QXLInvers* q) {
  SpiceInvers* s = &u.invers;
  GetQMask(&s->mask, &q->mask);
  return true;
}

bool QxlParser::ParseDrawRop3(QXLRop3* q) {
  SpiceRop3* s = &u.rop3;
  s->src_bitmap = GetImage(q->src_bitmap);
  if (!s->src_bitmap)
    return false;
  parse_rect(&s->src_area, &q->src_area);

  s->rop3 = q->rop3;
  s->scale_mode = q->scale_mode;
  GetBrush(&s->brush, &q->brush);
  GetQMask(&s->mask, &q->mask);
  return true;
}
