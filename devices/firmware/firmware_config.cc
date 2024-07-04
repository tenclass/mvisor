/* 
 * MVisor
 * Copyright (C) 2021 Terrence <terrence@tenclass.com>
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

#include "firmware_config.h"

#include <cstring>

#include "logger.h"
#include "device_manager.h"
#include "memory_manager.h"
#include "machine.h"
#include "smbios.h"
#include "firmware_config.pb.h"
#include "firmware_config.hex"
#include "acpi_builder.h"


#define FW_CFG_ACPI_DEVICE_ID "QEMU0002"

#define FEATURE_CONTROL_LOCKED                    (1<<0)
#define FEATURE_CONTROL_VMXON_ENABLED_INSIDE_SMX  (1ULL << 1)
#define FEATURE_CONTROL_VMXON_ENABLED_OUTSIDE_SMX (1<<2)
#define FEATURE_CONTROL_SGX_LC                    (1ULL << 17)
#define FEATURE_CONTROL_SGX                       (1ULL << 18)
#define FEATURE_CONTROL_LMCE                      (1<<20)


struct ConfigEntry {
  uint16_t      index;
  std::string   data;
  std::string   filename;
  VoidCallback  select_callback;
};

class FirmwareConfig : public Device, public AcpiTableInterface {
 private:
  uint16_t current_index_ = 0;
  uint32_t current_offset_ = 0;
  uint64_t dma_address_ = 0;
  AcpiBuilder* acpi_builder_ = nullptr;
  std::vector<ConfigEntry> config_entries_;

  std::vector<ConfigEntry>::iterator GetConfigEntry(uint16_t index) {
    return std::find_if(config_entries_.begin(), config_entries_.end(),
      [index](const ConfigEntry& entry) { return entry.index == index; });
  }

  void DmaTransfer() {
    fw_cfg_dma_access* dma = (fw_cfg_dma_access*)manager_->TranslateGuestMemory(dma_address_);
    dma_address_ = 0;

    uint32_t control = be32toh(dma->control);
    uint64_t buffer_address = be64toh(dma->address);
    uint32_t buffer_length = be32toh(dma->length);
    
    if (control & FW_CFG_DMA_CTL_SELECT) {
      SelectCurrentEntry(control >> 16);
    }

    uint8_t* buffer = (uint8_t*)manager_->TranslateGuestMemory(buffer_address);
    auto it = GetConfigEntry(current_index_);
    if (it == config_entries_.end()) {
      if (current_index_ & 0x8000) {
        dma->control = be32toh(FW_CFG_DMA_CTL_ERROR);
        /* Skip ARCH_LOCAL entries like ACPI, SMBIOS */
        return;
      } else {
        bzero(buffer, buffer_length);
        dma->control = 0;
        if (debug_) {
          MV_WARN("config entry 0x%x not found", current_index_);
        }
        return;
      }
    }

    if (control & FW_CFG_DMA_CTL_READ) {
      uint32_t size = std::min((uint32_t)it->data.size() - current_offset_, buffer_length);
      memcpy(buffer, it->data.data() + current_offset_, size);
      current_offset_ += size;
    } else if (control & FW_CFG_DMA_CTL_WRITE) {
      MV_PANIC("not supported");
    }
    dma->control = 0;
  }

  void AddConfigBytes(uint16_t index, std::string bytes) {
    config_entries_.emplace_back(ConfigEntry{index, bytes});
  }

  void AddConfigUInt32(uint16_t index, uint32_t value) {
    config_entries_.emplace_back(ConfigEntry{index, std::string((const char*)&value, sizeof(value))});
  }

  void AddConfigUInt16(uint16_t index, uint16_t value) {
    config_entries_.emplace_back(ConfigEntry{index, std::string((const char*)&value, sizeof(value))});
  }

  void AddConfigFile(std::string path, std::string data) {
    config_entries_.emplace_back(ConfigEntry{0, std::move(data), path});
  }

  void AddConfigFile(std::string path, std::string data, VoidCallback select_callback) {
    config_entries_.emplace_back(ConfigEntry{0, std::move(data), path, select_callback});
  }

  void SelectCurrentEntry(uint16_t index) {
    current_index_ = index;
    current_offset_ = 0;
    auto it = GetConfigEntry(current_index_);
    if (it != config_entries_.end() && it->select_callback) {
      it->select_callback();
    }
  }

  void UpdateConfigFile(std::string path, std::string data) {
    auto it = std::find_if(config_entries_.begin(), config_entries_.end(),
      [path](const ConfigEntry& entry) { return entry.filename == path; });
    if (it == config_entries_.end()) {
      MV_PANIC("config file %s not found", path.c_str());
    }
    it->data = data;
  }

  void AddConfigFileFromLocal(std::string path, std::string local_path) {
    FILE *fp = fopen(local_path.c_str(), "rb");
    if (fp == NULL) {
      MV_PANIC("failed to locate file %s", local_path.c_str());
    }
    fseek(fp, 0, SEEK_END);
    ssize_t file_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    uint8_t* buf = new uint8_t[file_size];
    fread(buf, file_size, 1, fp);
    fclose(fp);

    AddConfigFile(path, std::string((const char*)buf, file_size));
    delete[] buf;
  }

  void InitializeConfig() {
    // Clear all config entries
    config_entries_.clear();

    AddConfigBytes(FW_CFG_SIGNATURE, "QEMU");
    uint32_t version = FW_CFG_VERSION | FW_CFG_VERSION_DMA;
    AddConfigUInt32(FW_CFG_ID, version);

    auto machine = manager_->machine();
    int num_vcpus = machine->num_vcpus();
    AddConfigUInt16(FW_CFG_NB_CPUS, num_vcpus);
    AddConfigUInt16(FW_CFG_MAX_CPUS, num_vcpus);
    uint64_t numa_cfg[num_vcpus + 1] = { 0 };
    AddConfigBytes(FW_CFG_NUMA, std::string((const char*)numa_cfg, sizeof(numa_cfg)));
    AddConfigUInt16(FW_CFG_NOGRAPHIC, 0);
    AddConfigUInt32(FW_CFG_IRQ0_OVERRIDE, 1);

    /* show menu if more than 1 drives */
    if (manager_->io()->GetDiskImageCount() > 1) {
      AddConfigUInt16(FW_CFG_BOOT_MENU, 2);
    } else {
      AddConfigUInt16(FW_CFG_BOOT_MENU, 0);
    }

    InitializeE820Table();

    InitializeFiles();
    InitializeFileDir();
  }

  void InitializeFiles () {
    /* check VMX after vcpu started */
    auto vcpu = manager_->machine()->first_vcpu();
    MV_ASSERT(vcpu);
    if (vcpu->cpuid_features() & (1U << 5)) {
      uint64_t feature_control = FEATURE_CONTROL_VMXON_ENABLED_OUTSIDE_SMX | FEATURE_CONTROL_LOCKED;
      AddConfigFile("etc/msr_feature_control", std::string((char*)&feature_control, sizeof(feature_control)));
    }

    /* ACPI DSDT */
    auto machine = manager_->machine();
    if (acpi_builder_ == nullptr) {
      acpi_builder_ = new AcpiBuilder(machine);
    }
    for (auto file: acpi_builder_->GetTableNames()) {
      std::string data = acpi_builder_->GetTable(file);
      if (data.empty()) {
        continue;
      }
      AddConfigFile(file, data, [this, file]() {
        UpdateConfigFile(file, acpi_builder_->GetTable(file));
      });
    }
    AddConfigFile("etc/table-loader", acpi_builder_->GetTableLoader());

    std::string smbios_anchor, smbios_table;
    Smbios smbios(manager_->machine());
    smbios.GetTables(smbios_anchor, smbios_table);
    AddConfigFile("etc/smbios/smbios-tables", smbios_table);
    AddConfigFile("etc/smbios/smbios-anchor", smbios_anchor);
  }

  void InitializeFileDir() {
    fw_cfg_files dir;
    int index = 0;
    for (auto &entry : config_entries_) {
      if (entry.filename.empty()) {
        continue;
      }
      auto cfg_file = &dir.files[index];
      entry.index = FW_CFG_FILE_FIRST + index;
      strncpy(cfg_file->name, entry.filename.c_str(), sizeof(cfg_file->name));
      cfg_file->size = htobe32(entry.data.size());
      cfg_file->select = htobe16(entry.index);
      cfg_file->reserved = 0;
      if (++index >= FW_CFG_MAX_FILES) {
        break;
      }
    }
  
    dir.count = htobe32(index);
    auto dir_size = sizeof(dir.count) + index * sizeof(dir.files[0]);
    AddConfigBytes(FW_CFG_FILE_DIR, std::string((const char*)&dir, dir_size));
  }

  void InitializeE820Table() {
    MemoryManager* mm = manager_->machine()->memory_manager();
    std::vector<e820_entry> entries;

    for (auto region : mm->regions()) {
      e820_entry entry;
      entry.address = region->gpa;
      entry.length = region->size;

      if (region->type == kMemoryTypeRam && std::string("System") == region->name) {
        entry.type = E820_RAM;
      } else if (region->type == kMemoryTypeReserved) {
        entry.type = E820_RESERVED;
      } else {
        continue;
      }

      entries.emplace_back(std::move(entry));
    }

    AddConfigFile("etc/e820", std::string((char*)entries.data(), sizeof(e820_entry) * entries.size()));
  }


 public:
  FirmwareConfig() {
    set_default_parent_class("Ich9Lpc", "Piix3");

    AddIoResource(kIoResourceTypePio, FW_CFG_IO_BASE, 2, "Config IO");
    AddIoResource(kIoResourceTypePio, FW_CFG_DMA_IO_BASE, 8, "Config DMA");
  }

  ~FirmwareConfig() {
    if (acpi_builder_) {
      delete acpi_builder_;
    }
  }

  void Reset() override {
    Device::Reset();

    InitializeConfig();
  }

  bool SaveState(MigrationWriter* writer) {
    FirmwareConfigState state;
    state.set_current_index(current_index_);
    state.set_current_offset(current_offset_);
    state.set_dma_address(dma_address_);
    writer->WriteProtobuf("FIRMWARE_CONFIG", state);
    return Device::SaveState(writer);
  }

  bool LoadState(MigrationReader* reader) {
    FirmwareConfigState state;
    if (!reader->ReadProtobuf("FIRMWARE_CONFIG", state)) {
      return false;
    }
    current_index_ = state.current_index();
    current_offset_ = state.current_offset();
    dma_address_ = state.dma_address();
    return Device::LoadState(reader);
  }

  void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    if (resource->base == FW_CFG_IO_BASE && size == 2) {
      SelectCurrentEntry(*(uint16_t*)data);
    } else if (resource->base == FW_CFG_DMA_IO_BASE) {
      if (size == 4) {
        if (offset == 0) { // High 32bit address
          dma_address_ = (dma_address_ & 0xffffffff) | (uint64_t(be32toh(*(uint32_t*)data)) << 32);
        } else if (offset == 4) { // Low 32bit address
          dma_address_ = (dma_address_ & 0xffffffff00000000) | be32toh(*(uint32_t*)data);
          DmaTransfer();
        }
      } else if (size == 8) {
        dma_address_ = be64toh(*(uint64_t*)data);
        DmaTransfer();
      }
    } else {
      MV_PANIC("not implemented Write for %s base=0x%lx offset=0x%lx size=%d",
        name_, resource->base, offset, size);
    }
  }

  void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    if (resource->base == FW_CFG_IO_BASE && offset == 1) {
      auto it = GetConfigEntry(current_index_);
      if (it == config_entries_.end()) {
        MV_PANIC("config entry %d not found", current_index_);
      }
      while (size--) {
        if (current_offset_ < it->data.size()) {
          *data++ = it->data[current_offset_++];
        } else {
          *data++ = 0;
        }
      }
    } else if (resource->base == FW_CFG_DMA_IO_BASE) {
      uint64_t signature = htobe64(FW_CFG_DMA_SIGNATURE);
      memcpy(data, (uint8_t*)&signature + offset, size);
    } else {
      bzero(data, size);
      MV_ERROR("%s not implemented Read offset=0x%lx size=%d", name_, offset, size);
    }
  }

  virtual std::string GetAcpiTable() override {
    return std::string((const char*)firmware_config_aml_code, sizeof(firmware_config_aml_code));
  }
};

DECLARE_DEVICE(FirmwareConfig);
