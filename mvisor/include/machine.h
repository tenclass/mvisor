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

#ifndef MVISOR_MACHINE_H
#define MVISOR_MACHINE_H

#define PAGE_SIZE 4096

#include <string>
#include <thread>
#include <vector>
#include <map>
#include <signal.h>
#include "object.h"
#include "vcpu.h"
#include "io_thread.h"
#include "memory_manager.h"
#include "device_manager.h"
#include "configuration.h"

class Machine {
 public:
  Machine(std::string config_path);
  ~Machine();

  int Run();
  void Quit();
  bool IsValid() { return valid_; }
  void Reset();
  Object* LookupObjectByName(std::string name);
  Object* LookupObjectByClass(std::string class_name);

  inline DeviceManager* device_manager() { return device_manager_; }
  inline MemoryManager* memory_manager() { return memory_manager_; }
  inline const Configuration* configuration() { return config_; }
  inline int num_vcpus() { return num_vcpus_; }
  inline uint64_t ram_size() { return ram_size_; }
  inline bool debug() { return debug_; }

 private:
  friend class Vcpu;
  friend class MemoryManager;
  friend class DeviceManager;
  friend class Configuration;

  void InitializeKvm();
  void CreateArchRelated();
  void CreateVcpu();
  void LoadBiosFile();

  bool valid_ = true;
  int kvm_fd_ = -1;
  int kvm_vcpu_mmap_size_ = 0;
  int vm_fd_ = -1;
  
  uint64_t ram_size_ = 0;
  int num_vcpus_ = 0;
  std::vector<Vcpu*> vcpus_;
  MemoryManager* memory_manager_;
  DeviceManager* device_manager_;
  Configuration* config_;
  IoThread* io_thread_;

  std::string bios_path_;
  size_t bios_size_;
  void* bios_data_ = nullptr;
  void* bios_backup_ = nullptr;

  uint32_t cpuid_version_ = 0;
  uint32_t cpuid_features_ = 0;

  std::map<std::string, Object*> objects_;
  bool debug_ = false;
};

#endif // MVISOR_MACHINE_H
