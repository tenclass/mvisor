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

#define X86_EPT_IDENTITY_BASE 0xfeffc000

/* The Machine class handles all the VM initialization and common operations
 * such as interrupts, start, quit, pause, resume
 * KVM API reference: https://www.kernel.org/doc/html/latest/virt/kvm/api.html
 */
Machine::Machine(std::string config_path) {
  /* Load the configuration and set values of num_vcpus & ram_size */
  config_ = new Configuration(this);
  if (!config_->Load(config_path)) {
    MV_PANIC("failed to load config file: %s", config_path.c_str());
  }

  InitializeKvm();

  memory_manager_ = new MemoryManager(this);

  CreateArchRelated();

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
}

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


/*
  * On older Intel CPUs, KVM uses vm86 mode to emulate 16-bit code directly.
  * In order to use vm86 mode, an EPT identity map and a TSS  are needed.
  * Since these must be part of guest physical memory, we need to allocate
  * them, both by setting their start addresses in the kernel and by
  * creating a corresponding e820 entry. We need 4 pages before the BIOS.
  *
  * Older KVM versions may not support setting the identity map base. In
  * that case we need to stick with the default, i.e. a 256K maximum BIOS
  * size.
  */
void Machine::CreateArchRelated() {
  /* Allows up to 16M BIOSes. */
  uint64_t identity_base = X86_EPT_IDENTITY_BASE;
  if (ioctl(vm_fd_, KVM_SET_IDENTITY_MAP_ADDR, &identity_base) < 0) {
    MV_PANIC("failed to set identity map address");
  }

  if (ioctl(vm_fd_, KVM_SET_TSS_ADDR, identity_base + 0x1000) < 0) {
    MV_PANIC("failed to set tss");
  }
  
  /* Map these addresses as reserved so the guest never touch it */
  memory_manager_->Map(X86_EPT_IDENTITY_BASE, 4 * PAGE_SIZE, nullptr, kMemoryTypeReserved, "EPT+TSS");
}


/* Start vCPU threads and IO thread */
void Machine::Run() {
  if (config_->snapshot()) {
    auto path = std::filesystem::path(config_->path());
    Load(path.parent_path());
  } else {
    device_manager_->ResetDevices();
  }
  
  /* Set paused = false */
  Resume();

  for (auto vcpu: vcpus_) {
    vcpu->Start();
  }
  io_thread_->Start();
}

/* Maybe there are lots of things to do before quiting a VM */
void Machine::Quit() {
  if (!valid_)
    return;
  valid_ = false;

  /* If paused, threads are waiting to resume */
  wait_to_resume_.notify_all();

  for (auto vcpu: vcpus_) {
    vcpu->Kick();
  }
  io_thread_->Stop();
}

/* Recover BIOS data and reset all vCPU
 * FIXME: vCPU 0 sometimes hangs (CPU 100%) after reset
 */
void Machine::Reset() {
  memory_manager_->ResetBios();
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

std::vector<Object*> Machine::LookupObjects(std::function<bool (Object*)> compare) {
  std::vector<Object*> result;
  for (auto it = objects_.begin(); it != objects_.end(); it++) {
    if (compare(it->second)) {
      result.push_back(it->second);
    }
  }
  return result;
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
void Machine::Save(std::string path) {
  /* Make sure the machine is paused */
  if (!IsPaused()) {
    Pause();
  }
  MV_LOG("start saving");

  MigrationWriter writer(path);
  /* Save vcpu states */
  for (auto vcpu : vcpus_) {
    vcpu->SaveState(&writer);
  }
  /* Save device states */
  if (!device_manager_->SaveState(&writer)) {
    MV_PANIC("failed to save device states");
    return;
  }
  /* Save system RAM */
  if (!memory_manager_->SaveState(&writer)) {
    MV_PANIC("failed to save RAM");
    return;
  }
  /* Save disk images */
  if (!io_thread_->SaveDiskImage(&writer)) {
    MV_PANIC("failed to sync disk images");
    return;
  }
  /* Save configuration after saving disk images (paths might changed) */
  if (!config_->Save(path + "/configuration.yaml")) {
    MV_PANIC("failed to save configuration yaml");
    return;
  }
  MV_LOG("done saving");
}

/* Should call by UI thread */
void Machine::Load(std::string path) {
  /* Make sure the machine is paused */
  if (!IsPaused()) {
    Pause();
  }
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
  MV_LOG("done loading");
}
