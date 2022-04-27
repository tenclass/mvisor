/* 
 * MVisor - Virtual Machine Controller
 * KVM API reference: https://www.kernel.org/doc/html/latest/virt/kvm/api.html
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


#include "machine.h"

#include <linux/kvm.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

#include <filesystem>

#include "logger.h"
#include "disk_image.h"
#include "device_interface.h"
#include "migration.h"


/* The Machine class handles all the VM initialization and common operations
 * such as startup, quit, pause, resume */
Machine::Machine(std::string config_path) {
  /* Load the configuration and set values of num_vcpus & ram_size */
  config_ = new Configuration(this);
  if (!config_->Load(config_path)) {
    MV_PANIC("failed to load config file: %s", config_path.c_str());
  }

  InitializeKvm();

  /* Initialize system RAM and BIOS ROM */
  memory_manager_ = new MemoryManager(this);

  /* Currently, a Q35 chipset mother board is implemented */
  Device* root = dynamic_cast<Device*>(LookupObjectByName("system-root"));
  if (!root) {
    MV_PANIC("failed to find system-root device");
  }
  /* Initialize IO thread before devices */
  io_thread_ = new IoThread(this);
  /* Initialize device manager, connect and reset all devices */
  device_manager_ = new DeviceManager(this, root);

  /* Create vcpu objects */
  for (int i = 0; i < num_vcpus_; ++i) {
    Vcpu* vcpu = new Vcpu(this, i);
    vcpus_.push_back(vcpu);
  }

  /* Start threads and wait to resume */
  wait_count_ = num_vcpus_ + 1;
  paused_ = true;

  for (auto vcpu: vcpus_) {
    vcpu->Start();
  }
  io_thread_->Start();

  /* Reset devices after vCPU created and paused */
  device_manager_->ResetDevices();
}

/* Free VM resources */
Machine::~Machine() {
  valid_ = false;

  // Join all vcpu threads and free resources
  for (auto vcpu: vcpus_) {
    delete vcpu;
  }

  delete device_manager_;
  delete memory_manager_;
  delete io_thread_;

  // delete objects created by configuration
  for (auto it = objects_.begin(); it != objects_.end(); it++) {
    delete it->second;
  }

  if (vm_fd_ > 0)
    safe_close(&vm_fd_);
  if (kvm_fd_ > 0)
    safe_close(&kvm_fd_);
  delete config_;
}

/* Create KVM instance */
void Machine::InitializeKvm() {
  kvm_fd_ = open("/dev/kvm", O_RDWR);
  MV_ASSERT(kvm_fd_ > 0);

  int api_version = ioctl(kvm_fd_, KVM_GET_API_VERSION, 0);
  if (api_version != KVM_API_VERSION) {
    MV_PANIC("kvm api verison %d, expected: %d", api_version, KVM_API_VERSION);
  }

  // Get the vcpu information block size that share with kernel
  // Vcpu uses this value
  kvm_vcpu_mmap_size_ = ioctl(kvm_fd_, KVM_GET_VCPU_MMAP_SIZE, 0);
  MV_ASSERT(kvm_vcpu_mmap_size_ > 0);

  // Create vm so that we can map userspace memory
  vm_fd_ = ioctl(kvm_fd_, KVM_CREATE_VM, 0);
  MV_ASSERT(vm_fd_ > 0);
}

/* Maybe there are lots of things to do before quiting a VM */
void Machine::Quit() {
  if (!valid_)
    return;
  
  /* Pause all threads and flush disk cache as well */
  if (!paused_) {
    Pause();
  }
  valid_ = false;

  /* If paused, threads are waiting to resume */
  wait_to_resume_.notify_all();

  for (auto vcpu: vcpus_) {
    vcpu->Kick();
  }
  io_thread_->Stop();
}

/* Recover BIOS data and reset all vCPU */
void Machine::Reset() {
  if (!valid_)
    return;
  memory_manager_->Reset();
  device_manager_->ResetDevices();

  if (debug_) {
    MV_LOG("Resettings vCPUs");
  }
  for (auto vcpu: vcpus_) {
    vcpu->Schedule([vcpu]() {
      vcpu->Reset();
    });
  }
}

/* Find the first object with matching name */
Object* Machine::LookupObjectByName(std::string name) {
  auto it = objects_.find(name);
  if (it == objects_.end()) {
    return nullptr;
  }
  return it->second;
}

/* Find the first object with matching name */
Object* Machine::LookupObjectByClass(std::string name) {
  for (auto it = objects_.begin(); it != objects_.end(); it++) {
    if (name == it->second->classname()) {
      return it->second;
    }
  }
  return nullptr;
}

/* Find all objects that compare function returns true */
std::vector<Object*> Machine::LookupObjects(std::function<bool (Object*)> compare) {
  std::vector<Object*> result;
  for (auto it = objects_.begin(); it != objects_.end(); it++) {
    if (compare(it->second)) {
      result.push_back(it->second);
    }
  }
  return result;
}

/* Power button is pressed */
void Machine::Shutdown() {
  for (auto o : LookupObjects([](auto o) { return dynamic_cast<PowerDownInterface*>(o); })) {
    auto interface = dynamic_cast<PowerDownInterface*>(o);
    interface->PowerDown();
  }
}

/* Resume from paused state */
void Machine::Resume() {
  std::unique_lock<std::mutex> lock(mutex_);
  MV_ASSERT(paused_);
  MV_ASSERT(wait_count_ == 0);
  paused_ = false;

  /* Resume threads */
  wait_to_resume_.notify_all();

  /* Here all the threads are running, broadcast messages */
  for (auto &callback : state_change_listeners_) {
    callback();
  }
}

/* Currently this method can only be called from UI threads */
void Machine::Pause() {
  /* Mark paused state and wait for vCPU threads and IO thread to stop */
  std::unique_lock<std::mutex> lock(mutex_);
  if (!valid_ || paused_)
    return;
  paused_ = true;
  io_thread_->FlushDiskImages();

  wait_count_ = num_vcpus_ + 1;
  for (auto vcpu : vcpus_) {
    vcpu->Kick();
  }
  io_thread_->Kick();

  wait_to_pause_condition_.wait(lock, [this]() {
    return wait_count_ == 0;
  });

  /* Here all the threads are stopped, broadcast messages */
  for (auto &callback : state_change_listeners_) {
    callback();
  }
}

/* vCPU threads and IO threads call this method to sleep */
void Machine::WaitToResume() {
  std::unique_lock<std::mutex> lock(mutex_);  
  MV_ASSERT(wait_count_ > 0);
  wait_count_--;
  wait_to_pause_condition_.notify_all();
  wait_to_resume_.wait(lock, [this]() {
    return !IsPaused();
  });
}

/* Listeners are called after Pause / Resume */
void Machine::RegisterStateChangeListener(VoidCallback callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  state_change_listeners_.push_back(callback);
}

/* Should call by UI thread */
void Machine::Save(const std::string path) {
  MV_ASSERT(!saving_);
  /* Make sure the machine is paused */
  if (!IsPaused()) {
    Pause();
  }
  saving_ = true;
  MV_LOG("start saving");

  MigrationWriter writer(path);
  /* Save device states */
  if (!device_manager_->SaveState(&writer)) {
    MV_LOG("failed to save device states");
    return;
  }
  /* Save vcpu states */
  for (auto vcpu : vcpus_) {
    vcpu->SaveState(&writer);
  }
  /* Save system RAM */
  if (!memory_manager_->SaveState(&writer)) {
    MV_LOG("failed to save RAM");
    return;
  }
  /* Save disk images */
  if (!io_thread_->SaveDiskImage(&writer)) {
    MV_LOG("failed to sync disk images");
    return;
  }
  /* Save configuration after saving disk images (paths might changed) */
  if (!config_->Save(path + "/configuration.yaml")) {
    MV_LOG("failed to save configuration yaml");
    return;
  }

  saving_ = false;
  MV_LOG("done saving");
}

/* Should call by UI thread */
void Machine::Load(const std::string path) {
  MV_ASSERT(!loading_);
  /* Make sure the machine is paused */
  if (!IsPaused()) {
    Pause();
  }
  loading_ = true;
  MV_LOG("start loading");

  MigrationReader reader(path);
  /* Load system RAM */
  if (!memory_manager_->LoadState(&reader)) {
    MV_PANIC("failed to load RAM");
  }
  /* Load device states */
  if (!device_manager_->LoadState(&reader)) {
    MV_PANIC("failed to load device states");
  }
  /* Load vcpu states */
  for (auto vcpu : vcpus_) {
    if (!vcpu->LoadState(&reader)) {
      MV_PANIC("failed to load %s", vcpu->name());
    }
  }

  loading_ = false;
  MV_LOG("done loading");
}

const char* Machine::GetStatus() {
  if (!valid_) {
    return "invalid";
  }
  if (!paused_) {
    return "running";
  }
  if (saving_) {
    return "saving";
  }
  if (loading_) {
    return "loading";
  }

  /* otherwise return paused */
  return "paused";
}

