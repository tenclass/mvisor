/* 
 * MVisor QXL Render
 * Copyright (C) 2022 Terrence <terrence@tenclass.com>
 * Copy and modified from QEMU
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

#include <pixman.h>

#include "qxl_render.h"
#include "qxl.h"
#include "qxl_parser.h"
#include "logger.h"


static pixman_image_t *image_get(SpiceImageCache *cache, uint64_t id)
{
  auto ops = SPICE_CONTAINEROF(cache, QxlRenderOps, image_cache);
  auto& images = ops->render->images();

  void* data;
  if (!images.Get(id, data)) {
    return nullptr;
  }
  return pixman_image_ref((pixman_image_t*)data);
}

static void image_put(SpiceImageCache *image_cache, uint64_t id, pixman_image_t *image)
{
  auto ops = SPICE_CONTAINEROF(image_cache, QxlRenderOps, image_cache);
  auto& images = ops->render->images();
  if (images.Contains(id)) {
    return;
  }

  void* data = pixman_image_ref(image);
  images.Put(id, data);
}

static SpiceCanvas *surfaces_get(SpiceImageSurfaces *image_surfaces, uint32_t surface_id)
{
  auto ops = SPICE_CONTAINEROF(image_surfaces, QxlRenderOps, image_surfaces);
  auto& surfaces = ops->render->surfaces();

  auto it = surfaces.find(surface_id);
  return it != surfaces.end() ? it->second.canvas : nullptr;
}


static SpiceImageSurfacesOps image_surfaces_ops = {
  .get = surfaces_get
};

static SpiceImageCacheOps image_cache_ops = {
  .put = image_put,
  .get = image_get
};


QxlRender::QxlRender(Qxl* qxl) : qxl_(qxl) {
  render_ops_.image_cache.ops = &image_cache_ops;
  render_ops_.image_surfaces.ops = &image_surfaces_ops;
  render_ops_.render = this;
  debug_ = qxl->debug();

  images_.Initialize(1024, [this](auto key, void* data) {
    MV_UNUSED(key);
    pixman_image_unref((pixman_image_t*)data);
  });
}

QxlRender::~QxlRender() {
  images_.Clear();

  for (auto it = surfaces_.begin(); it != surfaces_.end(); it++) {
    auto& surface = it->second;
    surface.canvas->ops->destroy(surface.canvas);
    if (surface.qxl_surface_cmd) {
      qxl_->ReleaseGuestResource(&surface.qxl_surface_cmd->release_info);
    }
  }
}


void QxlRender::ParseSurfaceCommand(uint64_t slot_address) {
  auto surface_cmd = (QXLSurfaceCmd*)qxl_->GetMemSlotAddress(slot_address);
  switch (surface_cmd->type)
  {
  case QXL_SURFACE_CMD_CREATE:
    CreateSurface(surface_cmd->surface_id, slot_address);
    break;
  case QXL_SURFACE_CMD_DESTROY:
    DestroySurface(surface_cmd->surface_id);
    qxl_->ReleaseGuestResource(&surface_cmd->release_info);
    break;
  default:
    MV_PANIC("invalid cmd=%d", surface_cmd->type);
    break;
  }
}


void QxlRender::ParseDrawCommand(uint64_t slot_address) {
  // struct timespec ts1, ts2;
  // clock_gettime(CLOCK_REALTIME, &ts1);
  auto qxl_drawable = (QXLDrawable*)qxl_->GetMemSlotAddress(slot_address);

  QxlParser drawable(this);
  if (!drawable.Parse(qxl_drawable)) {
    qxl_->ReleaseGuestResource(&qxl_drawable->release_info);
    MV_ERROR("failed to parse drawable type=%d", drawable.type);
    return;
  }

  auto& bbox = drawable.bbox;
  auto& surface = GetSurface(drawable.surface_id);
  auto canvas = surface.canvas;

  switch (drawable.type)
  {
  case QXL_DRAW_FILL:
    canvas->ops->draw_fill(canvas, &drawable.bbox, &drawable.clip, &drawable.u.fill);
    break;
  case QXL_DRAW_OPAQUE:
    canvas->ops->draw_opaque(canvas, &drawable.bbox, &drawable.clip, &drawable.u.opaque);
    break;
  case QXL_DRAW_COPY:
    canvas->ops->draw_copy(canvas, &drawable.bbox, &drawable.clip, &drawable.u.copy);
    break;
  case QXL_COPY_BITS:
    canvas->ops->copy_bits(canvas, &drawable.bbox, &drawable.clip, &drawable.u.copy_bits.src_pos);
    break;
  case QXL_DRAW_BLEND:
    canvas->ops->draw_blend(canvas, &drawable.bbox, &drawable.clip, &drawable.u.blend);
    break;
  case QXL_DRAW_BLACKNESS:
    canvas->ops->draw_blackness(canvas, &drawable.bbox, &drawable.clip, &drawable.u.blackness);
    break;
  case QXL_DRAW_WHITENESS:
    canvas->ops->draw_whiteness(canvas, &drawable.bbox, &drawable.clip, &drawable.u.whiteness);
    break;
  case QXL_DRAW_INVERS:
    canvas->ops->draw_invers(canvas, &drawable.bbox, &drawable.clip, &drawable.u.invers);
    break;
  case QXL_DRAW_ROP3:
    canvas->ops->draw_rop3(canvas, &drawable.bbox, &drawable.clip, &drawable.u.rop3);
    break;
  case QXL_DRAW_STROKE:
    canvas->ops->draw_stroke(canvas, &drawable.bbox, &drawable.clip, &drawable.u.stroke);
    break;
  case QXL_DRAW_TEXT:
    canvas->ops->draw_text(canvas, &drawable.bbox, &drawable.clip, &drawable.u.text);
    break;
  case QXL_DRAW_TRANSPARENT:
    canvas->ops->draw_transparent(canvas, &drawable.bbox, &drawable.clip, &drawable.u.transparent);
    break;
  case QXL_DRAW_ALPHA_BLEND:
    canvas->ops->draw_alpha_blend(canvas, &drawable.bbox, &drawable.clip, &drawable.u.alphablend);
    break;
  default:
    MV_ERROR("unsupported type %d", drawable.type);
    break;
  }
  qxl_->ReleaseGuestResource(&qxl_drawable->release_info);

  AddDirtyRect(bbox.top, bbox.left, bbox.bottom, bbox.right);
  // clock_gettime(CLOCK_REALTIME, &ts2);
  // printf("diff draw %lu ns\n", ts2.tv_nsec - ts1.tv_nsec);
}

void QxlRender::UpdateArea(const QXLRect& rect, bool update_surface) {
  if (update_surface == 0) {
    AddDirtyRect(rect.top, rect.left, rect.bottom, rect.right);
  }
}

void QxlRender::GetUpdatePartials(std::vector<DisplayPartialBitmap>& partials) {
  if (dirty_rects_.empty())
    return;

  auto& surface = GetSurface(0);

  for (auto& rect : dirty_rects_) {
    DisplayPartialBitmap partial;
    partial.stride = surface.stride;
    partial.bpp = surface.bpp;
    partial.width = rect.right - rect.left;
    partial.height = rect.bottom - rect.top;
    partial.x = rect.left;
    partial.y = rect.top;
    partial.palette = nullptr;
    partial.data = surface.data + partial.stride * partial.y + partial.x * (partial.bpp / 8);
    partials.emplace_back(std::move(partial));
  }
  dirty_rects_.clear();
}

void QxlRender::Redraw() {
  dirty_rects_.clear();

  auto& surface = GetSurface(0);
  AddDirtyRect(0, 0, surface.height, surface.width);
}


void QxlRender::CreatePrimarySurface(const QXLSurfaceCreate& create, bool clear) {
  MV_ASSERT(surfaces_.find(0) == surfaces_.end());
  Surface surface;
  surface.id = 0;
  surface.slot_address = 0;
  surface.qxl_surface_cmd = nullptr;
  surface.bpp = qxl_->GetBitsPerPixelByFormat(create.format);
  surface.width = create.width;
  surface.height = create.height;
  surface.stride = create.stride;
  surface.data = (uint8_t*)qxl_->GetMemSlotAddress(create.mem);
  if (surface.stride < 0) {
    surface.data -= surface.stride * (surface.height - 1);
  }
  surface.canvas = canvas_create_for_data(surface.width, surface.height, create.format, surface.data, surface.stride,
    &render_ops_.image_cache, &render_ops_.image_surfaces, nullptr, nullptr, nullptr);
  surfaces_[surface.id] = surface;

  if (clear) {
    surface.canvas->ops->clear(surface.canvas);
  }

  Redraw();
  if (debug_) {
    MV_LOG("create primary %dx%dx%d stride=%d", surface.width, surface.height, surface.bpp, surface.stride);
  }
}

void QxlRender::DestroyPrimarySurface() {
  DestroySurface(0);
}

void QxlRender::CreateSurface(int surface_id, uint64_t slot_address) {
  MV_ASSERT(surfaces_.find(surface_id) == surfaces_.end());
  MV_ASSERT(surface_id > 0);
  MV_ASSERT(slot_address > 0);

  Surface surface;
  surface.id = surface_id;
  surface.slot_address = slot_address;
  surface.qxl_surface_cmd = (QXLSurfaceCmd*)qxl_->GetMemSlotAddress(slot_address);
  auto& create = surface.qxl_surface_cmd->u.surface_create;
  surface.bpp = qxl_->GetBitsPerPixelByFormat(create.format);
  surface.width = create.width;
  surface.height = create.height;
  surface.stride = create.stride;
  surface.data = (uint8_t*)qxl_->GetMemSlotAddress(create.data);
  if (surface.stride < 0) {
    surface.data -= surface.stride * (surface.height - 1);
  }
  surface.canvas = canvas_create_for_data(surface.width, surface.height, create.format, surface.data, surface.stride,
    &render_ops_.image_cache, &render_ops_.image_surfaces, nullptr, nullptr, nullptr);
  surfaces_[surface.id] = surface;

  if (debug_) {
    MV_LOG("create surface id=%d %dx%dx%d", surface_id, surface.width, surface.height, surface.bpp);
  }
}

const Surface& QxlRender::GetSurface(int surface_id) {
  auto it = surfaces_.find(surface_id);
  MV_ASSERT(it != surfaces_.end());
  return it->second;
}

void QxlRender::DestroySurface(int surface_id) {
  auto it = surfaces_.find(surface_id);
  if (it == surfaces_.end()) {
    MV_ERROR("failed to destroy surface %d", surface_id);
    return;
  }

  auto& surface = it->second;
  surface.canvas->ops->destroy(surface.canvas);
  surfaces_.erase(it);
  if (surface.qxl_surface_cmd) {
    qxl_->ReleaseGuestResource(&surface.qxl_surface_cmd->release_info);
  }

  if (debug_) {
    MV_LOG("destroy surface id=%d", surface.id);
  }
}

/* Align width to 16 and height to 2 for SSE or AVX */
void QxlRender::AddDirtyRect(int32_t top, int32_t left, int32_t bottom, int32_t right) {
  auto &surface = GetSurface(0);
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
  if (right > surface.width) {
    right = surface.width;
  }
  if (bottom > surface.height) {
    bottom = surface.height;
  }
  if (right - left < 2 || bottom - top < 2) {
    return;
  }

  AddDirtyRectInternal(top, left, bottom, right);
}

/* Use dirty rectangle algorithm to remove overlapped parts */
void QxlRender::AddDirtyRectInternal(int32_t top, int32_t left, int32_t bottom, int32_t right) {
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

  dirty_rects_.emplace_back(QXLRect {
    top, left, bottom, right
  });
}
