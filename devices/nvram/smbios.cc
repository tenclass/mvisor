/* 
 * MVisor SMBIOS Support
 * Copyright (C) 2021 Terrence <terrence@tenclass.com>
 * Copyright (C) 2009 Hewlett-Packard Development Company, L.P.
 * Authors:
 *  Alex Williamson <alex.williamson@hp.com>
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

#include "smbios.h"
#include <cstring>

#include "logger.h"
#include "machine.h"


Smbios::Smbios(Machine* machine) : machine_(machine) {
  SetupEntryPoint();
}

void Smbios::GetTables(std::string& anchor, std::string& tables) {
  BuildType16();
  BuildType17();

  tables.clear();
  for (auto& it : tables_) {
    tables.append(it.second);
    if (it.second.size() > entry_point_.max_structure_size) {
      entry_point_.max_structure_size = it.second.size();
    }
  }

  // Add the end of table marker
  tables.append(std::string("\x7F\x04\x00\x7F\x00\x00", 6));
  entry_point_.structure_table_length = tables.length();
  entry_point_.number_of_structures = tables_.size() + 1;
  anchor = std::string((char*)&entry_point_, sizeof(entry_point_));
}

void Smbios::SetupEntryPoint() {
  bzero(&entry_point_, sizeof(entry_point_));
  entry_point_.length = sizeof(smbios_21_entry_point);

  memcpy(entry_point_.anchor_string, "_SM_", 4);
  memcpy(entry_point_.intermediate_anchor_string, "_DMI_", 5);

  /* smbios spec v2.8 */
  entry_point_.smbios_major_version = 2;
  entry_point_.smbios_minor_version = 8;
  entry_point_.smbios_bcd_revision = 0x28;
}

void Smbios::BuildStructure(uint8_t type, void* data, size_t size, std::vector<std::string>& texts) {
  auto header = (smbios_structure_header*)data;
  header->type = type;
  header->length = size;
  header->handle = type << 8;

  std::string entry;
  entry.append((char*)data, size);
  if (texts.empty()) {
    entry.append("\0\0", 2);
  } else {
    for (auto& text : texts) {
      entry.append(text);
      entry.append("\0", 1);
    }
    entry.append("\0", 1);
  }
  tables_[type] = entry;
}

void Smbios::BuildType16() {
  std::vector<std::string> texts;
  // Physical Memory Array
  smbios_type_16 type_16 = {};
  type_16.location = 3; // Memory Array Location: 3 = System Board or Motherboard
  type_16.use = 3; // Memory Use: 3 = System Memory
  type_16.error_correction = 6; // Error Correction Type: 6 = Multi-bit ECC
  type_16.number_of_memory_devices = 1;

  auto ram_size_kb = machine_->ram_size() / 1024;
  MV_ASSERT(ram_size_kb <= 0xFFFFFFFF);
  type_16.maximum_capacity = ram_size_kb;
  type_16.extended_maximum_capacity = 0;
  type_16.memory_error_information_handle = 0xFFFE;

  BuildStructure(16, &type_16, sizeof(type_16), texts);
}

void Smbios::BuildType17() {
  // Memory Device
  std::vector<std::string> texts;
  smbios_type_17 type_17 = {};
  type_17.physical_memory_array_handle = 0x1000;
  type_17.memory_error_information_handle = 0xFFFE;
  type_17.total_width = 0xFFFF;
  type_17.data_width = 0xFFFF;

  auto ram_size_mb = machine_->ram_size() / 1024 / 1024;
  if (ram_size_mb <= 0x7FFF) {
    type_17.size = ram_size_mb;
    type_17.extended_size = 0;
  } else {
    type_17.size = 0x7FFF;
    type_17.extended_size = ram_size_mb;
  }

  type_17.form_factor = 9; // Form Factor: 9 = DIMM
  type_17.device_set = 0;
  texts.push_back("DIMM 0");
  type_17.device_locator_str = 1;
  type_17.bank_locator_str = 0;
  type_17.memory_type = 0x18; // Memory Type: 0x18 = DDR3
  type_17.type_detail = 0x80; // Type Detail: 0x80 = Synchronous
  type_17.speed = 2133; // Speed: 2133 MHz
  texts.push_back("MVISOR");
  type_17.manufacturer_str = 2;
  texts.push_back("00000000");
  type_17.serial_number_str = 3;
  texts.push_back("9876543210");
  type_17.asset_tag_number_str = 4;
  texts.push_back("TC00000000");
  type_17.part_number_str = 5;

  type_17.attributes = 0; // Unknown
  type_17.configured_clock_speed = type_17.speed;

  BuildStructure(17, &type_17, sizeof(type_17), texts);
}
