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

#ifndef _MVISOR_DEVICES_QXL_RENDER_H
#define _MVISOR_DEVICES_QXL_RENDER_H

#include <cstdint>

#include <array>
#include <map>

#include "lru_cache.h"
#include "qxl_dev.h"
#include "canvas/sw_canvas.h"
#include "device_interface.h"

class Qxl;
class QxlRender;

struct QxlRenderOps {
  SpiceImageCache     image_cache;
  SpiceImageSurfaces  image_surfaces;
  QxlRender*          render = nullptr;
};

struct Surface {
  uint            id;
  uint64_t        slot_address;
  QXLSurfaceCmd*  qxl_surface_cmd;
  int             bpp;
  int             width;
  int             height;
  int             stride;
  uint8_t*        data;
  SpiceCanvas*    canvas;
};


class QxlRender {
 private:
  Qxl*                          qxl_;
  std::map<uint32_t, Surface>   surfaces_;
  SimpleLRUCache<uint64_t, void*> images_;
  QxlRenderOps                  render_ops_;
  bool                          debug_;
  std::vector<QXLRect>          dirty_rects_;

 public:
  inline std::map<uint32_t, Surface>&       surfaces() { return surfaces_; }
  inline SimpleLRUCache<uint64_t, void*>&   images() { return images_; }
  inline Qxl*                               qxl() { return qxl_; }

  QxlRender(Qxl* qxl);
  ~QxlRender();

  void CreatePrimarySurface(const QXLSurfaceCreate& create, bool clear);
  void DestroyPrimarySurface();

  void ParseSurfaceCommand(uint64_t slot_address);
  void ParseDrawCommand(uint64_t slot_address);

  void GetUpdatePartials(std::vector<DisplayPartialBitmap>& partials);
  void UpdateArea(const QXLRect& rect, bool update_surface);
  void Redraw();
  const Surface& GetSurface(int surface_id);

 private:
  void CreateSurface(int surface_id, uint64_t slot_address);
  void DestroySurface(int surface_id);
  void AddDirtyRect(int32_t top, int32_t left, int32_t bottom, int32_t right);
  void AddDirtyRectInternal(int32_t top, int32_t left, int32_t bottom, int32_t right);
};

#endif // _MVISOR_DEVICES_QXL_RENDER_H
