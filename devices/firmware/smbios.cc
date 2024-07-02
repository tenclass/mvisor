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

#include "version.h"
#include "logger.h"
#include "machine.h"


Smbios::Smbios(Machine* machine) : machine_(machine) {
  default_manufacturer_ = "Tenclass";
  default_product_name_ = "mvisor";
  default_version_ = VERSION;
  if (machine_->vm_uuid().empty()) {
    default_serial_number_ = "00000000";
  } else {
    default_serial_number_ = machine_->vm_uuid();
  }
  SetupEntryPoint();
}

void Smbios::GetTables(std::string& anchor, std::string& table) {
  table_entries_.clear();
  BuildType1();
  BuildType2();
  BuildType3();
  BuildType4();
  BuildType16();
  BuildType17();
  BuildType19();
  BuildType32();
  // Add the end of table marker
  table_entries_.push_back(std::string("\x7F\x04\x00\x7F\x00\x00", 6));

  table.clear();
  for (auto& entry : table_entries_) {
    table.append(entry);
    if (entry.size() > entry_point_.max_structure_size) {
      entry_point_.max_structure_size = entry.size();
    }
  }

  entry_point_.structure_table_length = table.length();
  entry_point_.number_of_structures = table_entries_.size() + 1;
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

void Smbios::BuildStructure(uint8_t type, void* data, size_t size, std::vector<std::string>& texts, uint16_t handle) {
  auto header = (smbios_structure_header*)data;
  header->type = type;
  header->length = size;
  header->handle = handle;
  if (handle == 0) {
    header->handle = type << 8;
  }

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
  table_entries_.push_back(entry);
}

// System Information
void Smbios::BuildType1() {
  std::vector<std::string> texts;
  smbios_type_1 type_1 = {};

  texts.push_back(default_manufacturer_);
  type_1.manufacturer_str = 1;
  texts.push_back(default_product_name_);
  type_1.product_name_str = 2;
  texts.push_back(default_version_);
  type_1.version_str = 3;
  texts.push_back(default_serial_number_);
  type_1.serial_number_str = 4;

  type_1.wake_up_type = 0x06; // Power Switch
  if (!machine_->vm_name().empty()) {
    texts.push_back(machine_->vm_name());
    type_1.sku_number_str = 4;
  }

  texts.push_back("Q35");
  type_1.family_str = 5;

  BuildStructure(1, &type_1, sizeof(type_1), texts);
}

// Baseboard Information
void Smbios::BuildType2() {
  std::vector<std::string> texts;
  smbios_type_2 type_2 = {};

  texts.push_back(default_manufacturer_);
  type_2.manufacturer_str = 1;
  texts.push_back(default_product_name_);
  type_2.product_str = 2;
  texts.push_back(default_version_);
  type_2.version_str = 3;
  texts.push_back(default_serial_number_);
  type_2.serial_number_str = 4;

  type_2.feature_flags = 0x01; // Motherboard
  type_2.chassis_handle = 0x0300; // Type 3 handle
  type_2.board_type = 0x0A; // Motherboard

  BuildStructure(2, &type_2, sizeof(type_2), texts);
}

// System Enclosure
void Smbios::BuildType3() {
  std::vector<std::string> texts;
  smbios_type_3 type_3 = {};
  type_3.type = 0x01; // Other
  texts.push_back(default_manufacturer_);
  type_3.manufacturer_str = 1;
  texts.push_back(default_version_);
  type_3.version_str = 2;

  type_3.boot_up_state = 0x03; // Safe
  type_3.power_supply_state = 0x03; // Safe
  type_3.thermal_state = 0x03; // Safe
  type_3.security_status = 0x02; // Unknown

  if (!machine_->vm_name().empty()) {
    texts.push_back(machine_->vm_name());
    type_3.sku_number_str = 4;
  }

  BuildStructure(3, &type_3, sizeof(type_3), texts);
}

void Smbios::BuildType4() {
  // Processor Information
  std::vector<std::string> texts;
  smbios_type_4 type_4 = {};
  texts.push_back("CPU0");
  type_4.socket_designation_str = 1;
  type_4.processor_type = 3; // CPU
  type_4.processor_family = 0xD6; // Multi-core Intel Xeon
  texts.push_back("Intel");
  type_4.processor_manufacturer_str = 2;

  auto first_vcpu = machine_->first_vcpu();
  type_4.processor_id[0] = first_vcpu->cpuid_version();
  type_4.processor_id[1] = first_vcpu->cpuid_features() >> 32;
  texts.push_back(first_vcpu->cpuid_model());
  type_4.processor_version_str = 3;

  type_4.voltage = 0;
  type_4.max_speed = 2000; // 2 GHz
  type_4.current_speed = 2000; // 2 GHz
  type_4.status = 0x41; // Enabled, Populated
  type_4.l1_cache_handle = 0xFFFF;
  type_4.l2_cache_handle = 0xFFFF;
  type_4.l3_cache_handle = 0xFFFF;

  uint cores_per_socket = machine_->num_cores();
  uint threads_per_socket = machine_->num_threads() * cores_per_socket;
  type_4.core_count = cores_per_socket > 0xFF ? 0xFF : cores_per_socket;
  type_4.core_enabled = type_4.core_count;
  type_4.thread_count = threads_per_socket > 0xFF ? 0xFF : threads_per_socket;
  type_4.processor_characteristics = 0x00FC; // 64-bit capable, Multi-core, Hardware Thread
  type_4.processor_family2 = type_4.processor_family;

  BuildStructure(4, &type_4, sizeof(type_4), texts);
}

// Physical Memory Array
void Smbios::BuildType16() {
  std::vector<std::string> texts;
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

// Memory Device
void Smbios::BuildType17() {
  std::vector<std::string> texts;
  smbios_type_17 type_17 = {};
  type_17.physical_memory_array_handle = 0x1000;
  type_17.memory_error_information_handle = 0xFFFE;
  type_17.total_width = 64;
  type_17.data_width = 64;

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
  texts.push_back(default_manufacturer_);
  type_17.manufacturer_str = 2;
  texts.push_back(default_serial_number_);
  type_17.serial_number_str = 3;
  type_17.asset_tag_number_str = 0;
  texts.push_back("TC00000000");
  type_17.part_number_str = 4;

  type_17.attributes = 0; // Unknown
  type_17.configured_clock_speed = type_17.speed;

  BuildStructure(17, &type_17, sizeof(type_17), texts);
}

// Memory Array Mapped Address
void Smbios::BuildType19() {
  auto mm = machine_->memory_manager();
  uint16_t handle = 19 << 8;
  for (auto region: mm->regions()) {
    if (region->type == kMemoryTypeRam && std::string("System") == region->name) {
      std::vector<std::string> texts;
      auto start_kb = region->gpa / 1024;
      auto end_kb = (region->gpa + region->size - 1) / 1024;
      smbios_type_19 type_19 = {};
      type_19.starting_address = start_kb;
      type_19.ending_address = end_kb;
      type_19.extended_ending_address = 0;

      type_19.memory_array_handle = 0x1000; // Type 16 handle
      type_19.partition_width = 1; // One device per partition

      BuildStructure(19, &type_19, sizeof(type_19), texts, handle++);
    }
  }
}

// System Boot Information
void Smbios::BuildType32() {
  std::vector<std::string> texts;
  smbios_type_32 type_32 = {};
  type_32.boot_status = 0; // No errors

  BuildStructure(32, &type_32, sizeof(type_32), texts);
}