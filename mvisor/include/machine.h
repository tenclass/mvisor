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
#include <mutex>
#include <condition_variable>
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

  void Quit();
  void Reset();
  void Pause();
  void Resume();
  void Shutdown();
  bool IsValid() { return valid_; }
  bool IsPaused() { return valid_ && paused_; }
  void WaitToResume();
  void Save(std::string path);
  void Load(std::string path);
  const char* GetStatus();

  Object* LookupObjectByName(std::string name);
  Object* LookupObjectByClass(std::string class_name);
  std::vector<Object*> LookupObjects(std::function<bool (Object*)> compare);
  void RegisterStateChangeListener(VoidCallback callback);

  inline DeviceManager* device_manager() { return device_manager_; }
  inline MemoryManager* memory_manager() { return memory_manager_; }
  inline const Configuration* configuration() { return config_; }
  inline int num_vcpus() { return num_vcpus_; }
  inline uint64_t ram_size() { return ram_size_; }
  inline bool debug() { return debug_; }
  inline bool hypervisor() { return hypervisor_; }
  inline bool power_on() { return power_on_; }
  inline const std::string& guest_os() const { return guest_os_; }
  inline const std::string& vm_name() const { return vm_name_; }
  inline const std::string& vm_uuid() const { return vm_uuid_; }
  inline void set_guest_os(std::string os) { guest_os_ = os; }
  inline void set_vm_name(std::string name) { vm_name_ = name; }
  inline void set_vm_uuid(std::string uuid) { vm_uuid_ = uuid; }
  inline void set_power_on(bool on) { power_on_ = on; }
  inline Vcpu* first_vcpu() { return vcpus_.size() ? vcpus_[0] : nullptr; }

 private:
  friend class IoThread;
  friend class Vcpu;
  friend class MemoryManager;
  friend class DeviceManager;
  friend class Configuration;

  void InitializeKvm();

  bool valid_ = true;
  bool paused_ = true;
  bool loading_ = false;
  bool saving_ = false;
  bool power_on_ = false;
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

  std::map<std::string, Object*> objects_;
  bool debug_ = false;
  bool hypervisor_ = false;
  std::string guest_os_;
  std::string vm_name_;
  std::string vm_uuid_;

  std::mutex mutex_;
  std::condition_variable wait_to_resume_;
  std::condition_variable wait_to_pause_condition_;
  uint wait_count_ = 0;
  std::vector<VoidCallback> state_change_listeners_;
};

#endif // MVISOR_MACHINE_H
