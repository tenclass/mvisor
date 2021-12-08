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
#include <signal.h>
#include "vcpu.h"
#include "io_thread.h"
#include "memory_manager.h"
#include "device_manager.h"

class Machine {
 public:
  Machine(int vcpus, uint64_t ram_size);
  ~Machine();

  int Run();
  void Quit();
  bool IsValid() { return valid_; }
  void Reset();

  inline DeviceManager* device_manager() { return device_manager_; }
  inline MemoryManager* memory_manager() { return memory_manager_; }
  int num_vcpus() { return num_vcpus_; }
  const std::string& executable_path() { return executable_path_; }

 private:
  friend class Vcpu;
  friend class MemoryManager;
  friend class DeviceManager;

  void InitializePath();
  void InitializeKvm();
  void CreateArchRelated();
  void CreateVcpu();
  Device* CreateQ35();
  void LoadBiosFile(const char* path);

  bool valid_ = true;
  int kvm_fd_ = -1;
  int kvm_vcpu_mmap_size_ = 0;
  int vm_fd_ = -1;
  
  int num_vcpus_ = 0;
  std::vector<Vcpu*> vcpus_;
  MemoryManager* memory_manager_;
  DeviceManager* device_manager_;
  IoThread* io_thread_;

  uint64_t ram_size_;
  size_t bios_size_;
  void* bios_data_ = nullptr;
  void* bios_backup_ = nullptr;

  std::string executable_path_;
  uint32_t cpuid_version_ = 0;
  uint32_t cpuid_features_ = 0;
};

#endif // MVISOR_MACHINE_H
