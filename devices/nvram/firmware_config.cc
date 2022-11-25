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
#include "acpi_dsdt/acpi_dsdt.hex.h"
#include "acpi_dsdt/q35_acpi_dsdt.hex.h"


#define FW_CFG_ACPI_DEVICE_ID "QEMU0002"

#define FEATURE_CONTROL_LOCKED                    (1<<0)
#define FEATURE_CONTROL_VMXON_ENABLED_INSIDE_SMX  (1ULL << 1)
#define FEATURE_CONTROL_VMXON_ENABLED_OUTSIDE_SMX (1<<2)
#define FEATURE_CONTROL_SGX_LC                    (1ULL << 17)
#define FEATURE_CONTROL_SGX                       (1ULL << 18)
#define FEATURE_CONTROL_LMCE                      (1<<20)


class FirmwareConfig : public Device {
 private:
  uint16_t current_index_ = 0;
  uint32_t current_offset_ = 0;
  uint64_t dma_address_ = 0;
  std::map<uint16_t, std::string> config_;
  std::map<std::string, std::string> files_;


  void DmaTransfer() {
    fw_cfg_dma_access* dma = (fw_cfg_dma_access*)manager_->TranslateGuestMemory(dma_address_);
    dma_address_ = 0;

    dma->control = be32toh(dma->control);
    dma->address = be64toh(dma->address);
    dma->length = be32toh(dma->length);
    
    if (dma->control & FW_CFG_DMA_CTL_SELECT) {
      current_index_ = dma->control >> 16;
      current_offset_ = 0;
    }

    auto it = config_.find(current_index_);
    if (it == config_.end()) {
      dma->control = be32toh(FW_CFG_DMA_CTL_ERROR);
      if (current_index_ & 0x8000) {
        /* Skip ARCH_LOCAL entries like ACPI, SMBIOS */
        return;
      }
      MV_PANIC("config entry not found 0x%x", current_index_);
    }

    uint8_t* data = (uint8_t*)manager_->TranslateGuestMemory(dma->address);
    if (dma->control & FW_CFG_DMA_CTL_READ) {
      uint32_t size = it->second.size() - current_offset_;
      if (size > dma->length)
        size = dma->length;
      memcpy(data, it->second.data() + current_offset_, size);
      current_offset_ += size;
    } else if (dma->control & FW_CFG_DMA_CTL_WRITE) {
      MV_PANIC("not supported");
    }
    dma->control = 0;
  }

  void SetConfigBytes(uint16_t index, std::string bytes) {
    config_[index] = bytes;
  }

  void SetConfigUInt32(uint16_t index, uint32_t value) {
    config_[index] = std::string((const char*)&value, sizeof(value));
  }

  void SetConfigUInt16(uint16_t index, uint16_t value) {
    config_[index] = std::string((const char*)&value, sizeof(value));
  }

  void AddConfigFile(std::string path, void* data, size_t size) {
    files_[path] = std::string((const char*)data, size);
    if (debug_) {
      MV_HEXDUMP(path.c_str(), files_[path].data(), files_[path].size());
    }
  }

  void AddConfigFile(std::string path, std::string local_path) {
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

    AddConfigFile(path, buf, file_size);
    delete buf;
  }

  void InitializeConfig() {
    SetConfigBytes(FW_CFG_SIGNATURE, "QEMU");
    uint32_t version = FW_CFG_VERSION | FW_CFG_VERSION_DMA;
    SetConfigUInt32(FW_CFG_ID, version);
    SetConfigUInt32(FW_CFG_FILE_DIR, 0);

    auto machine = manager_->machine();
    int num_vcpus = machine->num_vcpus();
    SetConfigUInt16(FW_CFG_NB_CPUS, num_vcpus);
    SetConfigUInt16(FW_CFG_MAX_CPUS, num_vcpus);
    uint64_t numa_cfg[num_vcpus + 1] = { 0 };
    SetConfigBytes(FW_CFG_NUMA, std::string((const char*)numa_cfg, sizeof(numa_cfg)));
    SetConfigUInt16(FW_CFG_NOGRAPHIC, 0);
    SetConfigUInt32(FW_CFG_IRQ0_OVERRIDE, 1);
    SetConfigUInt16(FW_CFG_BOOT_MENU, 0);

    InitializeE820Table();

    InitializeFiles();
    InitializeFileDir();
  }

  void InitializeFiles () {
    /* disable S3 S4 (suspend / hibernate) */
    bool disable_s3 = true, disable_s4 = true;
    uint8_t suspend[6] = {128, 0, 0, 129, 128, 128};
    suspend[3] = 1 | (uint(!disable_s3) << 7);
    suspend[4] = 2 | (uint(!disable_s4) << 7);
    AddConfigFile("etc/system-states", suspend, sizeof(suspend));

    /* ACPI DSDT */
    auto machine = manager_->machine();
    if (machine->LookupObjectByClass("I440fxHost")) {
      AddConfigFile("acpi/dsdt", acpi_dsdt_aml_code, sizeof(acpi_dsdt_aml_code));
    } else if (machine->LookupObjectByClass("Q35Host")) {
      AddConfigFile("acpi/dsdt", q35_acpi_dsdt_aml_code, sizeof(q35_acpi_dsdt_aml_code));
    } else {
      MV_WARN("Unknown motherboard. ACPI might not work.");
    }

    std::string smbios_anchor, smbios_table;
    Smbios smbios(manager_->machine());
    smbios.GetTables(smbios_anchor, smbios_table);

    // FIXME: Not implemented yet. Uncommenting these lines would cause windows BSOD 0x7B
    AddConfigFile("etc/smbios/smbios-tables", smbios_table.data(), smbios_table.size());
    AddConfigFile("etc/smbios/smbios-anchor", smbios_anchor.data(), smbios_anchor.size());
  }

  void InitializeFileDir() {
    fw_cfg_files dir;
    int index = 0;
    for (auto &item : files_) {
      auto cfg_file = &dir.files[index];
      strncpy(cfg_file->name, item.first.c_str(), item.first.size());
      cfg_file->size = htobe32(item.second.size());
      cfg_file->select = htobe16(FW_CFG_FILE_FIRST + index);
      cfg_file->reserved = 0;
      SetConfigBytes(FW_CFG_FILE_FIRST + index, item.second);

      if (++index >= FW_CFG_MAX_FILES) {
        break;
      }
    }
    dir.count = htobe32(index);

    std::string data((const char*)&dir, sizeof(dir.count) + index * sizeof(dir.files[0]));
    SetConfigBytes(FW_CFG_FILE_DIR, std::move(data));
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

    AddConfigFile("etc/e820", entries.data(), sizeof(e820_entry) * entries.size());
  }


 public:
  FirmwareConfig() {
    set_default_parent_class("Ich9Lpc", "Piix3");

    AddIoResource(kIoResourceTypePio, FW_CFG_IO_BASE, 2, "Config IO");
    AddIoResource(kIoResourceTypePio, FW_CFG_DMA_IO_BASE, 8, "Config DMA");
  }

  void Connect() {
    Device::Connect();
    InitializeConfig();
  }

  void Reset() {
    Device::Reset();

    /* show menu if more than 1 drives */
    if (manager_->io()->GetDiskImageCount() > 1) {
      SetConfigUInt16(FW_CFG_BOOT_MENU, 2);
    } else {
      SetConfigUInt16(FW_CFG_BOOT_MENU, 0);
    }

    /* check VMX after vcpu started */
    auto vcpu = manager_->machine()->first_vcpu();
    MV_ASSERT(vcpu);
    if (vcpu->cpuid_features() & (1U << 5)) {
      uint64_t feature_control = FEATURE_CONTROL_VMXON_ENABLED_OUTSIDE_SMX | FEATURE_CONTROL_LOCKED;
      AddConfigFile("etc/msr_feature_control", &feature_control, sizeof(feature_control));
    }

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
      current_index_ = *(uint16_t*)data;
      current_offset_ = 0;
    } else if (resource->base == FW_CFG_DMA_IO_BASE) {
      if (size == 4) {
        if (offset == 0) { // High 32bit address
          dma_address_ = be32toh(*(uint32_t*)data);
          dma_address_ <<= 32;
        } else if (offset == 4) { // Low 32bit address
          dma_address_ |= be32toh(*(uint32_t*)data);
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
      auto it = config_.find(current_index_);
      if (it == config_.end()) {
        MV_PANIC("config entry %d not found", current_index_);
      }
      while (size--) {
        if (current_offset_ < it->second.size()) {
          *data++ = it->second[current_offset_++];
        } else {
          *data++ = 0;
        }
      }
    } else {
      MV_PANIC("not implemented Read for %s offset=0x%lx size=%d", name_, offset, size);
    }
  }
};

DECLARE_DEVICE(FirmwareConfig);
