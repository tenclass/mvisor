/* 
 * MVisor QXL
 * Copyright (C) 2021 Terrence <terrence@tenclass.com>
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

#ifndef _MVISOR_DEVICES_QXL_H
#define _MVISOR_DEVICES_QXL_H

#include "../vga.h"

#include <list>
#include <vector>
#include <mutex>

#include "qxl_dev.h"
#include "machine.h"


#define NUM_MEMSLOTS 8
#define MEMSLOT_GENERATION_BITS 8
#define MEMSLOT_SLOT_BITS 8

struct PrimarySurface {
  bool            active;
};


class QxlRender;
class Qxl : public Vga, public DisplayResizeInterface {
 private:
  uint32_t  qxl_rom_size_ = 0;
  void*     qxl_rom_base_ = nullptr;
  uint32_t  qxl_vram32_size_ = 0;
  uint8_t*  qxl_vram32_base_ = nullptr;

  QXLRom*   qxl_rom_ = nullptr;
  QXLModes* qxl_modes_ = nullptr;
  QXLRam*   qxl_ram_ = nullptr;

  struct guest_slots {
    QXLMemSlot    slot;
    uint64_t      offset;
    bool          active;
    uint8_t*      hva; 
  } guest_slots_[NUM_MEMSLOTS];

  PrimarySurface                primary_surface_;
  std::vector<QXLReleaseInfo*>  free_resources_;
  DisplayMouseCursor            current_cursor_;
  const StateChangeListener*    state_change_listener_ = nullptr;

  QxlRender*                    render_ = nullptr;
  std::recursive_mutex          render_mutex_;
  bool                          flushing_commands_ = false;

 public:
  Qxl();
  virtual ~Qxl();
  virtual void Connect();
  virtual void Disconnect();
  virtual void Reset();
  virtual bool SaveState(MigrationWriter* writer);
  virtual bool LoadState(MigrationReader* reader);
  virtual void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);

  // Interfaces
  virtual bool AcquireUpdate(DisplayUpdate& update, bool redraw);
  virtual void ReleaseUpdate();
  virtual bool Resize(int width, int height);

  // QxlRender make use of these methods
  void* GetMemSlotAddress(uint64_t data);
  void ReleaseGuestResource(QXLReleaseInfo* info);
  int GetBitsPerPixelByFormat(uint32_t format);
  size_t GetMemSlotChunkData(uint64_t data, std::vector<iovec>& vector);
  size_t GetMemSlotChunkData(QXLDataChunk* chunk, std::vector<iovec>& vector);
  void GetMemSlotLinearizedChunkData(QXLDataChunk* chunk, std::string& data);

 private:
  virtual bool ActivatePciBar(uint8_t index);
  virtual bool DeactivatePciBar(uint8_t index);
  virtual void UpdateDisplayMode();

  void IntializeQxlRom();
  void IntializeQxlRam();
  void SetInterrupt(uint32_t interrupt);
  void UpdateIrqLevel();
  uint64_t TranslateAsyncCommand(uint64_t command, bool* async);
  void ParseControlCommand(uint64_t command, uint32_t argument);
  void FlushCommandsAndResources();
  void AddMemSlot(int slot_id, QXLMemSlot& slot);
  void DeleteMemSlot(int slot_id);
  void FreeGuestResources();
  void FetchCommands();
  bool FetchGraphicsCommand(QXLCommand& command);
  bool FetchCursorCommand(QXLCommand& command);
  void ParseSurfaceCommand(uint64_t slot_address);
  void ParseCursorCommand(uint64_t slot_address);
};



#endif // _MVISOR_DEVICES_QXL_H
