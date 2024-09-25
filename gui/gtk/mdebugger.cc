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

#include <sys/ioctl.h>
#include <sstream>
#include <iomanip>
#include "mdebugger.h"
#include "logger.h"
#include "device_manager.h"

std::string MDebugger::glade_ = "<interface><requires lib=\"gtk+\" version=\"3.24\" /><object class=\"GtkWindow\" id=\"main_window\"><property name=\"can-focus\">False</property><property name=\"title\" translatable=\"yes\">MVisor-Debugger</property><property name=\"gravity\">north</property><child><object class=\"GtkGrid\"><property name=\"visible\">True</property><property name=\"can-focus\">False</property><child><object class=\"GtkNotebook\"><property name=\"visible\">True</property><property name=\"can-focus\">True</property><property name=\"show-border\">False</property><child><object class=\"GtkGrid\"><property name=\"visible\">True</property><property name=\"can-focus\">False</property><property name=\"margin-start\">8</property><property name=\"margin-end\">8</property><property name=\"margin-top\">8</property><property name=\"margin-bottom\">8</property><property name=\"row-homogeneous\">True</property><property name=\"column-homogeneous\">True</property><child><object class=\"GtkButton\" id=\"process_view_refresh\"><property name=\"label\" translatable=\"yes\">Refresh</property><property name=\"visible\">True</property><property name=\"can-focus\">True</property><property name=\"receives-default\">True</property><property name=\"valign\">start</property><property name=\"image-position\">right</property></object><packing><property name=\"left-attach\">9</property><property name=\"top-attach\">0</property></packing></child><child><object class=\"GtkSearchEntry\" id=\"process_search_entry\"><property name=\"visible\">True</property><property name=\"can-focus\">True</property><property name=\"halign\">start</property><property name=\"valign\">start</property><property name=\"width-chars\">40</property><property name=\"primary-icon-name\">edit-find-symbolic</property><property name=\"primary-icon-activatable\">False</property><property name=\"primary-icon-sensitive\">False</property><property name=\"placeholder-text\" translatable=\"yes\">Search</property></object><packing><property name=\"left-attach\">0</property><property name=\"top-attach\">0</property><property name=\"width\">6</property></packing></child><child><object class=\"GtkScrolledWindow\"><property name=\"visible\">True</property><property name=\"can-focus\">True</property><property name=\"shadow-type\">in</property><property name=\"min-content-height\">500</property><property name=\"max-content-height\">500</property><property name=\"propagate-natural-height\">True</property><child><object class=\"GtkTreeView\" id=\"process_view\"><property name=\"visible\">True</property><property name=\"can-focus\">True</property><property name=\"has-focus\">True</property><property name=\"receives-default\">True</property><property name=\"headers-clickable\">False</property><property name=\"enable-search\">False</property><property name=\"enable-grid-lines\">both</property><child internal-child=\"selection\"><object class=\"GtkTreeSelection\" /></child></object></child></object><packing><property name=\"left-attach\">0</property><property name=\"top-attach\">1</property><property name=\"width\">10</property><property name=\"height\">9</property></packing></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child></object><packing><property name=\"tab-expand\">True</property></packing></child><child type=\"tab\"><object class=\"GtkLabel\"><property name=\"visible\">True</property><property name=\"can-focus\">False</property><property name=\"label\" translatable=\"yes\">Process Viewer</property></object><packing><property name=\"tab-fill\">False</property></packing></child><child><object class=\"GtkGrid\"><property name=\"visible\">True</property><property name=\"can-focus\">False</property><property name=\"margin-start\">8</property><property name=\"margin-end\">8</property><property name=\"margin-top\">8</property><property name=\"margin-bottom\">8</property><property name=\"row-homogeneous\">True</property><property name=\"column-homogeneous\">True</property><child><object class=\"GtkComboBoxText\" id=\"process_list\"><property name=\"visible\">True</property><property name=\"can-focus\">False</property><property name=\"valign\">start</property></object><packing><property name=\"left-attach\">1</property><property name=\"top-attach\">0</property><property name=\"width\">3</property></packing></child><child><object class=\"GtkLabel\"><property name=\"visible\">True</property><property name=\"can-focus\">False</property><property name=\"halign\">end</property><property name=\"valign\">start</property><property name=\"margin-end\">7</property><property name=\"margin-top\">4</property><property name=\"label\" translatable=\"yes\">Process: </property><attributes><attribute name=\"size\" value=\"13000\" /></attributes></object><packing><property name=\"left-attach\">0</property><property name=\"top-attach\">0</property></packing></child><child><object class=\"GtkLabel\"><property name=\"visible\">True</property><property name=\"can-focus\">False</property><property name=\"halign\">end</property><property name=\"valign\">start</property><property name=\"margin-end\">7</property><property name=\"margin-top\">4</property><property name=\"label\" translatable=\"yes\">Address: </property><attributes><attribute name=\"size\" value=\"13000\" /></attributes></object><packing><property name=\"left-attach\">0</property><property name=\"top-attach\">1</property></packing></child><child><object class=\"GtkLabel\"><property name=\"visible\">True</property><property name=\"can-focus\">False</property><property name=\"halign\">end</property><property name=\"valign\">start</property><property name=\"margin-end\">7</property><property name=\"margin-top\">4</property><property name=\"label\" translatable=\"yes\">Memory: </property><attributes><attribute name=\"size\" value=\"13000\" /></attributes></object><packing><property name=\"left-attach\">0</property><property name=\"top-attach\">2</property></packing></child><child><object class=\"GtkEntry\" id=\"memory_address_input\"><property name=\"height-request\">30</property><property name=\"visible\">True</property><property name=\"can-focus\">True</property><property name=\"has-focus\">True</property><property name=\"valign\">start</property><property name=\"hexpand\">True</property><property name=\"max-length\">32</property><property name=\"placeholder-text\" translatable=\"yes\">0x</property><property name=\"input-purpose\">number</property></object><packing><property name=\"left-attach\">1</property><property name=\"top-attach\">1</property><property name=\"width\">2</property></packing></child><child><object class=\"GtkButton\" id=\"process_list_refresh\"><property name=\"label\" translatable=\"yes\">Refresh</property><property name=\"visible\">True</property><property name=\"can-focus\">True</property><property name=\"receives-default\">True</property><property name=\"valign\">start</property><property name=\"image-position\">right</property></object><packing><property name=\"left-attach\">9</property><property name=\"top-attach\">0</property></packing></child><child><object class=\"GtkButton\" id=\"memory_write\"><property name=\"label\" translatable=\"yes\">Write</property><property name=\"visible\">True</property><property name=\"can-focus\">True</property><property name=\"receives-default\">True</property><property name=\"valign\">start</property><property name=\"margin-top\">16</property><property name=\"image-position\">right</property></object><packing><property name=\"left-attach\">9</property><property name=\"top-attach\">9</property></packing></child><child><object class=\"GtkButton\" id=\"memory_read\"><property name=\"label\" translatable=\"yes\">Read</property><property name=\"visible\">True</property><property name=\"can-focus\">True</property><property name=\"receives-default\">True</property><property name=\"valign\">start</property><property name=\"margin-start\">6</property><property name=\"image-position\">right</property></object><packing><property name=\"left-attach\">3</property><property name=\"top-attach\">1</property></packing></child><child><object class=\"GtkScrolledWindow\"><property name=\"visible\">True</property><property name=\"can-focus\">True</property><property name=\"shadow-type\">in</property><property name=\"min-content-height\">500</property><property name=\"max-content-height\">500</property><property name=\"propagate-natural-height\">True</property><child><object class=\"GtkTextView\" id=\"memory_text\"><property name=\"visible\">True</property><property name=\"can-focus\">True</property><property name=\"pixels-above-lines\">8</property><property name=\"pixels-below-lines\">8</property><property name=\"pixels-inside-wrap\">8</property><property name=\"wrap-mode\">char</property><property name=\"left-margin\">8</property><property name=\"right-margin\">8</property><property name=\"top-margin\">8</property><property name=\"bottom-margin\">8</property></object></child></object><packing><property name=\"left-attach\">1</property><property name=\"top-attach\">2</property><property name=\"width\">8</property><property name=\"height\">7</property></packing></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child><child><placeholder /></child></object><packing><property name=\"position\">1</property><property name=\"tab-expand\">True</property></packing></child><child type=\"tab\"><object class=\"GtkLabel\"><property name=\"visible\">True</property><property name=\"can-focus\">False</property><property name=\"label\" translatable=\"yes\">Memory Hacker</property></object><packing><property name=\"position\">1</property><property name=\"tab-fill\">False</property></packing></child></object><packing><property name=\"left-attach\">0</property><property name=\"top-attach\">0</property></packing></child><child><object class=\"GtkLabel\" id=\"status_label\"><property name=\"visible\">True</property><property name=\"can-focus\">False</property><property name=\"halign\">start</property><property name=\"valign\">end</property><property name=\"margin-start\">8</property><property name=\"margin-top\">8</property><property name=\"hexpand\">True</property><property name=\"vexpand\">True</property><property name=\"single-line-mode\">True</property></object><packing><property name=\"left-attach\">0</property><property name=\"top-attach\">1</property></packing></child></object></child></object></interface>";
Machine* MDebugger::machine_ = nullptr;
gchar* MDebugger::search_text_ = nullptr;
GtkListStore* MDebugger::process_store_ = nullptr;
GtkComboBoxText* MDebugger::process_list_ = nullptr;
GtkEntry* MDebugger::memory_address_input_ = nullptr;
GtkTextView* MDebugger::memory_text_ = nullptr;
GtkLabel* MDebugger::status_ = nullptr;
std::vector<GuestProcess> MDebugger::processes_;

MDebugger::MDebugger() {
}

MDebugger::~MDebugger() {
}

gboolean MDebugger::ProcessViewSearchFilter(GtkTreeModel* model, GtkTreeIter* iter, gpointer data) {
  MV_UNUSED(data);

  gchar* name;
  gchar* pid;
  gchar* ppid;
  gchar* cr3;
  gboolean visible = false;

  gtk_tree_model_get(model, iter, PROCESS_VIEW_COLUMN_PID, &pid, -1);
  gtk_tree_model_get(model, iter, PROCESS_VIEW_COLUMN_PPID, &ppid, -1);
  gtk_tree_model_get(model, iter, PROCESS_VIEW_COLUMN_NAME, &name, -1);
  gtk_tree_model_get(model, iter, PROCESS_VIEW_COLUMN_CR3, &cr3, -1);

  if (search_text_ && strlen(search_text_) > 0) {
    if (strstr(name, search_text_) != NULL) {
      visible = true;
    } else if (strstr(pid, search_text_) != NULL) {
      visible = true;
    } else if (strstr(ppid, search_text_) != NULL) {
      visible = true;
    } else if (strstr(cr3, search_text_) != NULL) {
      visible = true;
    }
  } else {
    visible = true;
  }

  g_free(name);
  g_free(pid);
  g_free(ppid);
  g_free(cr3);
  return visible;
}

void MDebugger::ProcessViewSearchChanged(GtkSearchEntry* search_entry, gpointer user_data) {
  if (search_text_) {
    g_free(search_text_);
  }

  auto text = gtk_entry_get_text(GTK_ENTRY(search_entry));
  search_text_ = g_strdup(text);

  auto filter = GTK_TREE_MODEL_FILTER(user_data);
  gtk_tree_model_filter_refilter(filter);
}

void MDebugger::Activate(GtkApplication* app, gpointer user_data) {
  MV_UNUSED(user_data);

  auto builder = gtk_builder_new();
  gtk_builder_add_from_string(builder, glade_.c_str(), glade_.size(), NULL);

  status_ = GTK_LABEL(gtk_builder_get_object(builder, "status_label"));
  process_list_ = GTK_COMBO_BOX_TEXT(gtk_builder_get_object(builder, "process_list"));
  memory_address_input_ = GTK_ENTRY(gtk_builder_get_object(builder, "memory_address_input"));
  memory_text_ = GTK_TEXT_VIEW(gtk_builder_get_object(builder, "memory_text"));
  process_store_ = gtk_list_store_new(NUM_COLUMNS, G_TYPE_UINT64, G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING);

  auto window = GTK_WIDGET(gtk_builder_get_object(builder, "main_window"));
  auto process_view = GTK_TREE_VIEW(gtk_builder_get_object(builder, "process_view"));
  auto renderer = gtk_cell_renderer_text_new();

  auto column = gtk_tree_view_column_new_with_attributes("Index", renderer, "text", PROCESS_VIEW_COLUMN_INDEX, NULL);
  gtk_tree_view_column_set_expand(column, TRUE);
  gtk_tree_view_append_column(process_view, column);

  column = gtk_tree_view_column_new_with_attributes("PID", renderer, "text", PROCESS_VIEW_COLUMN_PID, NULL);
  gtk_tree_view_column_set_expand(column, TRUE);
  gtk_tree_view_append_column(process_view, column);

  column = gtk_tree_view_column_new_with_attributes("PPID", renderer, "text", PROCESS_VIEW_COLUMN_PPID, NULL);
  gtk_tree_view_column_set_expand(column, TRUE);
  gtk_tree_view_append_column(process_view, column);

  column = gtk_tree_view_column_new_with_attributes("CR3", renderer, "text", PROCESS_VIEW_COLUMN_CR3, NULL);
  gtk_tree_view_column_set_expand(column, TRUE);
  gtk_tree_view_append_column(process_view, column);

  column = gtk_tree_view_column_new_with_attributes("Name", renderer, "text", PROCESS_VIEW_COLUMN_NAME, NULL);
  gtk_tree_view_column_set_expand(column, TRUE);
  gtk_tree_view_append_column(process_view, column);

  auto filter = GTK_TREE_MODEL_FILTER(gtk_tree_model_filter_new(GTK_TREE_MODEL(process_store_), NULL));
  gtk_tree_model_filter_set_visible_func(filter, MDebugger::ProcessViewSearchFilter, NULL, NULL);

  auto process_search_entry = GTK_SEARCH_ENTRY(gtk_builder_get_object(builder, "process_search_entry"));
  g_signal_connect(process_search_entry, "search-changed", G_CALLBACK(MDebugger::ProcessViewSearchChanged), filter);

  gtk_tree_view_set_model(process_view, GTK_TREE_MODEL(filter));

  auto process_view_refresh = GTK_BUTTON(gtk_builder_get_object(builder, "process_view_refresh"));
  g_signal_connect(process_view_refresh, "clicked", G_CALLBACK(MDebugger::ProcessViewRefresh), NULL);

  auto process_list_refresh = GTK_BUTTON(gtk_builder_get_object(builder, "process_list_refresh"));
  g_signal_connect(process_list_refresh, "clicked", G_CALLBACK(MDebugger::ProcessViewRefresh), NULL);

  auto memory_read = GTK_BUTTON(gtk_builder_get_object(builder, "memory_read"));
  g_signal_connect(memory_read, "clicked", G_CALLBACK(MDebugger::MemoryRead), NULL);

  auto memory_write = GTK_BUTTON(gtk_builder_get_object(builder, "memory_write"));
  g_signal_connect(memory_write, "clicked", G_CALLBACK(MDebugger::MemoryWrite), NULL);

  gtk_application_add_window(app, GTK_WINDOW(window));
  gtk_widget_show_all(window);
  g_object_unref(builder);
}

std::string MDebugger::NumberToHexString(uint64_t number) {
  std::stringstream ss;
  ss << std::hex << number;
  return ss.str();
}

std::string MDebugger::BytesToHexString(const uint8_t* data, size_t length) {
  std::ostringstream oss;
  for (size_t i = 0; i < length; ++i) {
    oss << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(data[i]);
    if (i % 16 == 15) {
      oss << "\n";
    } else {
      oss << "\t";
    }
  }
  return oss.str();
}

std::vector<uint8_t> MDebugger::HexStringToBytes(const std::string& hex) {
  std::vector<uint8_t> bytes;

  if (hex.empty()) {
    gtk_label_set_label(status_, "invalid length of memory view.");
    return bytes;
  }

  // Remove any spaces and newlines from the input string
  std::string clean_hex;
  clean_hex.reserve(hex.size());
  for (char c : hex) {
    if (!std::isspace(c)) {
      clean_hex.push_back(c);
    }
  }

  // Check if the length of the cleaned string is even
  if (clean_hex.length() % 2 != 0) {
    gtk_label_set_label(status_, "invalid length of memory view.");
    return bytes;
  }

  bytes.reserve(clean_hex.length() / 2);
  for (size_t i = 0; i < clean_hex.length(); i += 2) {
    auto byte_string = clean_hex.substr(i, 2);
    auto byte = static_cast<uint8_t>(std::stoul(byte_string, nullptr, 16));
    bytes.push_back(byte);
  }

  return bytes;
}

void* MDebugger::GuestVAToHostAddress(uint64_t gva, uint64_t cr3) {
  void* hva;
  uint64_t gpa;

  auto pml4_gpa = cr3 & 0xFFFFFFFFFF000;
  auto pml4 = (uint64_t*)machine_->device_manager()->TranslateGuestMemory(pml4_gpa);

  auto pml4e_index = (gva >> 39) & 0x1FF;
  auto pml4e = pml4[pml4e_index];
  MV_ASSERT((pml4e & 0x80) == 0);

  if (!(pml4e & 0x1)) {
    MV_ERROR("page fault happened");
    return nullptr;
  }

  auto pdpt_gpa = pml4e & 0xFFFFFFFFFF000;
  auto pdpt = (uint64_t*)machine_->device_manager()->TranslateGuestMemory(pdpt_gpa);

  auto pdpte_index = (gva >> 30) & 0x1FF;
  auto pdpte = pdpt[pdpte_index];

  if (!(pdpte & 0x1)) {
    MV_ERROR("page fault happened");
    return nullptr;
  }

  if ((pdpte & 0x80) == 0) {
    auto pd_gpa = pdpte & 0xFFFFFFFFFF000;
    auto pd = (uint64_t*)machine_->device_manager()->TranslateGuestMemory(pd_gpa);

    auto pde_index = (gva >> 21) & 0x1FF;
    auto pde = pd[pde_index];

    if (!(pde & 0x1)) {
      MV_ERROR("page fault happened");
      return nullptr;
    }

    if ((pde & 0x80) == 0) {
      auto pte_gpa = pde & 0xFFFFFFFFFF000;
      auto pt = (uint64_t*)machine_->device_manager()->TranslateGuestMemory(pte_gpa);

      auto pte_index = (gva >> 12) & 0x1FF;
      auto pte = pt[pte_index];

      if (!(pte & 0x1)) {
        MV_ERROR("page fault happened");
        return nullptr;
      }

      auto offset = gva & 0xFFF;
      gpa = (pte & 0xFFFFFFFFFF000) + offset;
      hva = machine_->device_manager()->TranslateGuestMemory(gpa);
    } else {
      auto offset = gva & 0x1FFFFF;
      gpa = (pde & 0xFFFFFFFE00000) + offset;
      hva = machine_->device_manager()->TranslateGuestMemory(gpa);
    }
  } else {
    auto offset = gva & 0x3FFFFFFF;
    gpa = (pdpte & 0xFFFFFC0000000) + offset;
    hva = machine_->device_manager()->TranslateGuestMemory(gpa);
  }

  return hva;
}

bool MDebugger::GetGuestProcesses(std::vector<GuestProcess> &processes) {
  machine_->Pause();

  bool success = false;
  for (auto vcpu : machine_->vcpus()) {
    struct kvm_sregs sregs;
    if (ioctl(vcpu->fd(), KVM_GET_SREGS, &sregs) < 0)
      MV_PANIC("KVM_GET_REGS failed");

    auto kpcr = (uint8_t*)GuestVAToHostAddress((uint64_t)sregs.gs.base, sregs.cr3);
    auto kthread_gva = *(uint64_t*)(kpcr + KTHREAD_IN_KPCR_OFFSET);
    if (!kthread_gva) {
      MV_LOG("kthread not exists at vcpu=%d", vcpu->vcpu_id());
      continue;
    }

    auto kthread = (uint8_t*)GuestVAToHostAddress(kthread_gva, sregs.cr3);
    auto kprocess = (uint8_t*)GuestVAToHostAddress(*(uint64_t*)(kthread + KPROCESS_IN_KTHREAD_OFFSET), sregs.cr3);
    auto eprocess = kprocess + EPROCESS_IN_KPROCESS_OFFSET;

    auto active_process_links = (_LIST_ENTRY*)(eprocess + ACTIVE_PROCESS_LINK_IN_EPROCESS_OFFSET);
    auto process_list_entry = active_process_links;
    if (process_list_entry->Flink == nullptr) {
      MV_LOG("vcpu=%d is idle", vcpu->vcpu_id());
      continue;
    }

    do {
      auto eprocess_next = (uint8_t*)process_list_entry - ACTIVE_PROCESS_LINK_IN_EPROCESS_OFFSET;
      GuestProcess process = {
        .eprocess = eprocess_next,
        .pid = *(uint32_t*)((char*)eprocess_next + PROCESS_ID_IN_EPROCESS_OFFSET),
        .ppid = *(uint32_t*)((char*)eprocess_next + PARENT_PROCESS_ID_IN_EPROCESS_OFFSET),
        .cr3 = *(uint64_t*)(eprocess_next + DIRECTORY_TABLE_BASE_IN_EPROCESS_OFFSET),
        .name = (char*)eprocess_next + PROCESS_NAME_IN_EPROCESS_OFFSET};

      // MV_LOG("pid=%d ppid=%d name=%s cr3=0x%llx", process.pid, process.ppid,
      // process.name.c_str(), process.cr3);
      processes.emplace_back(std::move(process));

      process_list_entry = (_LIST_ENTRY*)GuestVAToHostAddress((uint64_t)process_list_entry->Flink, sregs.cr3);
    } while (process_list_entry != active_process_links);

    success = true;
    // succeeded, jump out
    break;
  }

  machine_->Resume();
  return success;
}

bool MDebugger::GetHvaFromUserInput(void** hva, size_t* size) {
  auto index = gtk_combo_box_get_active(GTK_COMBO_BOX(process_list_));
  if (index == -1) {
    gtk_label_set_label(status_, "must select one valid process before reading.");
    return false;
  }

  auto gva_str = gtk_entry_get_text(memory_address_input_);
  if (strlen(gva_str) == 0) {
    gtk_label_set_label(status_, "invalid guest virtual address.");
    return false;
  }

  auto& process = processes_[index];
  auto gva = std::stoll(gva_str, nullptr, 16);
  *hva = GuestVAToHostAddress(gva, process.cr3);
  if (!*hva) {
    gtk_label_set_label(status_, "get host virtul address failed.");
    return false;
  }

  *size = PAGE_SIZE - (gva % PAGE_SIZE);
  return true;
}

void MDebugger::MemoryRead(GtkWidget* widget, gpointer data) {
  MV_UNUSED(widget);
  MV_UNUSED(data);

  void* hva;
  size_t size;
  if (!GetHvaFromUserInput(&hva, &size)) {
    return;
  }

  std::string memory = BytesToHexString((const uint8_t*)hva, size);
  auto buffer = gtk_text_view_get_buffer(memory_text_);
  gtk_text_buffer_set_text(buffer, memory.c_str(), memory.size());

  gtk_label_set_label(status_, "read memory success.");
}

void MDebugger::MemoryWrite(GtkWidget* widget, gpointer data) {
  MV_UNUSED(widget);
  MV_UNUSED(data);

  void* hva;
  size_t size;
  if (!GetHvaFromUserInput(&hva, &size)) {
    return;
  }

  GtkTextIter start, end;
  auto buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(memory_text_));
  gtk_text_buffer_get_start_iter(buffer, &start);
  gtk_text_buffer_get_end_iter(buffer, &end);

  auto memory = gtk_text_buffer_get_text(buffer, &start, &end, FALSE);
  std::string memory_string(memory);
  g_free(memory);

  auto bytes = HexStringToBytes(memory_string);
  if (bytes.empty()) {
    return;
  }

  if (bytes.size() > size) {
    gtk_label_set_label(status_, "cross-page writing is not supported yet.");
    return;
  }

  // write memory to guest
  memcpy(hva, bytes.data(), bytes.size());

  gtk_label_set_label(status_, "write memory success.");
}

void MDebugger::ProcessViewRefresh(GtkWidget* widget, gpointer data) {
  MV_UNUSED(widget);
  MV_UNUSED(data);
  
  gtk_list_store_clear(process_store_);
  gtk_combo_box_text_remove_all(process_list_);
  gtk_entry_set_text(memory_address_input_, "");
  gtk_text_buffer_set_text(gtk_text_view_get_buffer(memory_text_), "", -1);

  processes_.clear();
  if (!GetGuestProcesses(processes_)) {
    gtk_label_set_label(status_, "Get guest process failed for all cpus are idle now. Please try again.");
    return;
  }

  GtkTreeIter iter;
  for (size_t i = 0; i < processes_.size(); i++) {
    auto pid = std::to_string(processes_[i].pid);
    auto ppid = std::to_string(processes_[i].ppid);
    auto cr3 = "0x" + NumberToHexString((uint64_t)processes_[i].cr3);
    std::string name;
    if (processes_[i].pid == 0) {
      name = "idle";
    } else {
      name = processes_[i].name;
    }

    // update process view
    gtk_list_store_append(process_store_, &iter);
    gtk_list_store_set(process_store_, &iter, 
                      PROCESS_VIEW_COLUMN_INDEX, i, 
                      PROCESS_VIEW_COLUMN_PID, pid.c_str(),
                      PROCESS_VIEW_COLUMN_PPID, ppid.c_str(), 
                      PROCESS_VIEW_COLUMN_CR3, cr3.c_str(), 
                      PROCESS_VIEW_COLUMN_NAME, name.c_str(), 
                      -1);

    // update process list in memory hacker
    auto combo_item = name + "(" + pid + ")";
    gtk_combo_box_text_append(process_list_, NULL, combo_item.c_str());
  }
  gtk_combo_box_set_active(GTK_COMBO_BOX(process_list_), 0);

  gtk_label_set_label(status_, "refresh success.");
}

void MDebugger::Run(Machine* machine) { 
  if (!running_) {
    running_ = true;
    machine_ = machine;

    std::thread([this]{
      auto app = gtk_application_new("mvisor.debugger", G_APPLICATION_DEFAULT_FLAGS);
      g_signal_connect(app, "activate", G_CALLBACK(MDebugger::Activate), NULL);
      g_application_run(G_APPLICATION(app), 0, NULL);
      g_object_unref(app);
      running_ = false;
    }).detach();
  }
}
