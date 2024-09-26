/* 
 * MVisor - Debugger
 * Copyright (C) 2024 Andy <550896603@qq.com>
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

#ifndef _MVISOR_DEBUGGER_H
#define _MVISOR_DEBUGGER_H

#include <gtk/gtk.h>
#include <string>
#include "machine.h"
#include "windows/kernel.h"

enum ProcessViewColumn{
  PROCESS_VIEW_COLUMN_INDEX,
  PROCESS_VIEW_COLUMN_PID,
  PROCESS_VIEW_COLUMN_PPID,
  PROCESS_VIEW_COLUMN_CR3,
  PROCESS_VIEW_COLUMN_NAME,
  NUM_COLUMNS
};

class MDebugger {
 private:
  MDebugger();
  ~MDebugger();

  bool running_ = false;

  static Machine* machine_;
  static GtkLabel* status_;
  static gchar* search_text_;
  static GtkEntry* memory_address_input_;
  static GtkTextView* memory_text_;
  static GtkListStore* process_store_;
  static GtkComboBoxText* process_list_;
  static std::vector<GuestProcess> processes_;

  static std::string NumberToHexString(uint64_t number);
  static std::string BytesToHexString(const uint8_t* data, size_t size);
  static std::vector<uint8_t> HexStringToBytes(const std::string& hex);

  static bool GetGuestProcesses(std::vector<GuestProcess>& processes);
  static bool GetHvaFromUserInput(void** hva, size_t* size);
  static void* GuestVAToHostAddress(uint64_t gva, uint64_t cr3);

  static void Activate(GtkApplication* app, gpointer user_data);
  static void ProcessViewSearchChanged(GtkSearchEntry* search_entry, gpointer user_data);
  static gboolean ProcessViewSearchFilter(GtkTreeModel* model, GtkTreeIter* iter, gpointer data);

  static void MemoryRead(GtkWidget* widget, gpointer data);
  static void MemoryWrite(GtkWidget* widget, gpointer data);
  static void ProcessViewRefresh(GtkWidget* widget, gpointer data);

public:
  static MDebugger& getInstance() {
    static MDebugger instance;
    return instance;
  }

  MDebugger(const MDebugger&) = delete;
  MDebugger& operator=(const MDebugger&) = delete;

  void Run(Machine* machine);
};

#endif
