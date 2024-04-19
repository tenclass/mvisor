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


Machine::Machine(std::string config_path) {
  /* Load the configuration and set values of num_vcpus & ram_size */
  config_ = new Configuration(this);
  if (!config_->Load(config_path)) {
    MV_PANIC("failed to load config file: %s", config_path.c_str());
  }

  InitializeKvm();

  /* Initialize system RAM and BIOS ROM */
  memory_manager_ = new MemoryManager(this);

  /* Currently, an i440fx / q35 chipset motherboard is implemented */
  Device* root = dynamic_cast<Device*>(LookupObjectByClass("SystemRoot"));
  if (!root) {
    MV_PANIC("failed to find system-root device");
  }
  /* Initialize IO thread before devices */
  io_thread_ = new IoThread(this);
  /* Initialize device manager, connect and reset all devices */
  device_manager_ = new DeviceManager(this, root);

  /* Create vcpu objects */
  for (int i = 0; i < num_vcpus_; ++i) {
    vcpus_.push_back(new Vcpu(this, i));
  }

  /* Start threads and wait to resume */
  std::unique_lock<std::mutex> lock(mutex_);
  wait_count_ = num_vcpus_ + 1;
  paused_ = true;

  for (auto vcpu: vcpus_) {
    vcpu->Start();
  }
  io_thread_->Start();
  /* Wait for all threads ready */
  wait_to_pause_condition_.wait(lock, [this]() {
    return wait_count_ == 0;
  });

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

  safe_close(&vm_fd_);
  safe_close(&kvm_fd_);
  delete config_;
  
  if (network_writer_) {
    delete network_writer_;
  }
}

/* Create KVM instance */
void Machine::InitializeKvm() {
  kvm_fd_ = open("/dev/kvm", O_RDWR);
  MV_ASSERT(kvm_fd_ > 0);

  int api_version = ioctl(kvm_fd_, KVM_GET_API_VERSION, 0);
  if (api_version != KVM_API_VERSION) {
    MV_PANIC("kvm api verison %d, expected: %d", api_version, KVM_API_VERSION);
  }

  // Get the vcpu information block size. Vcpu uses this value
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

  MV_LOG("Resettings vCPUs");

  for (auto vcpu : vcpus_) {
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
  if (!paused_) {
    MV_LOG("unable to resume a running machine");
    return;
  }
  MV_ASSERT(wait_count_ == 0);
  paused_ = false;

  /* Before running, broadcast messages */
  for (auto listener : state_change_listeners_) {
    listener->callback();
  }
  
  /* Resume threads */
  wait_to_resume_.notify_all();
}

/* Currently this method can only be called from UI threads */
void Machine::Pause() {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!valid_ || paused_)
    return;

  /* Mark paused state and wait for vCPU threads */
  wait_count_ = num_vcpus_;
  pausing_ = true;
  paused_ = true;

  for (auto vcpu : vcpus_) {
    vcpu->Kick();
  }
  wait_to_pause_condition_.wait(lock, [this]() {
    return wait_count_ == 0;
  });

  /* Here all the threads are stopped, broadcast messages */
  for (auto listener : state_change_listeners_) {
    listener->callback();
  }

  /* Wait for IO thread to stop */
  io_thread_->FlushDiskImages();

  wait_count_ = 1;
  pausing_ = false;
  io_thread_->Kick();

  wait_to_pause_condition_.wait(lock, [this]() {
    return wait_count_ == 0;
  });
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
const StateChangeListener* Machine::RegisterStateChangeListener(VoidCallback callback) {
  auto listener = new StateChangeListener {
    .callback = callback
  };
  std::lock_guard<std::mutex> lock(mutex_);
  state_change_listeners_.insert(listener);
  return listener;
}

void Machine::UnregisterStateChangeListener(const StateChangeListener** plistener) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_change_listeners_.erase(*plistener)) {
    delete *plistener;
    *plistener = nullptr;
  }
}

bool Machine::PrepareForSaving() {
  bool ret = true;
  bool paused = IsPaused();
  if (!paused) {
    Pause();
  }

  // create snapshot for current disk images
  ret = io_thread_->CreateQcow2ImageSnapshot();
  if (!ret) {
    goto end;
  }

  // start tracking dirty memory before sending memory
  memory_manager_->StartTrackingDirtyMemory();

end:
  if (!paused) {
    Resume();
  }
  return ret;
}

/* Save through network */
bool Machine::Save(const std::string ip, const uint16_t port) {
  if (saving_) {
    MV_ERROR("machine is busy migrating");
    return false;
  }

  MV_LOG("start saving");
  saving_ = true;

  bool ret = false;
  if (network_writer_) {
    MV_LOG("retry save machine, close last connection");
    delete network_writer_;
  }

  /* Make connection to target machine */
  network_writer_ = new MigrationNetworkWriter();
  if (!network_writer_->Connect(ip, port)) {
    MV_ERROR("failed to connect target machine");
    goto end;
  }

  if (!PrepareForSaving()) {
    MV_ERROR("failed to prepare for saving");
    goto end;
  }

  /* Save backing disk images */
  if(!io_thread_->SaveBackingDiskImage(network_writer_)) {
    MV_ERROR("failed to save backing disk images");
    goto end;
  }
  
  if (!network_writer_->WaitForSignal(kMigrateBackingImageComplete)) {
    goto end;
  }

  /* Save system RAM */
  if (!memory_manager_->SaveState(network_writer_)) {
    MV_ERROR("failed to save memory");
    goto end;
  }

  if (!network_writer_->WaitForSignal(kMigrateRamComplete)) {
    goto end;
  }
  ret = true;

end:
  if (!ret) {
    delete network_writer_;
    network_writer_ = nullptr;
    memory_manager_->StopTrackingDirtyMemory();
  }

  saving_ = false;
  MV_LOG("done saving");
  return ret;
}

/* Save through network */
bool Machine::PostSave() {
  if (saving_) {
    MV_ERROR("machine is busy migrating");
    return false;
  }

  if (!network_writer_) {
    MV_ERROR("Save must be called successfully before PostSave");
    return false;
  }

  if (!IsPaused()) {
    Pause();
  }

  MV_LOG("start post-saving");
  saving_ = true;
  bool ret = false;

  if(!io_thread_->SaveDiskImage(network_writer_)) {
    MV_ERROR("failed to save disk images");
    goto end;
  }

  if (!network_writer_->WaitForSignal(kMigrateImageComplete)) {
    goto end;
  }

  if (!memory_manager_->SaveDirtyMemory(network_writer_, kDirtyMemoryTypeKvm)) {
    MV_ERROR("failed to save dirty memory from kvm");
    goto end;
  }

  if (!network_writer_->WaitForSignal(kMigrateDirtyMemoryFromKvmComplete)) {
    goto end;
  }

  if (!memory_manager_->SaveDirtyMemory(network_writer_, kDirtyMemoryTypeListener)) {
    MV_ERROR("failed to save dirty memory from listener");
    goto end;
  }

  if (!network_writer_->WaitForSignal(kMigrateDirtyMemoryFromListenerComplete)) {
    goto end;
  }

  /* Target machine need to get all memory before load device state,
   * so we send device state and dirty memory from dma together. */
  if (!device_manager_->SaveState(network_writer_)) {
    MV_ERROR("failed to save device states");
    goto end;
  }

  /* Save dirty memory from dma,
   * this step must be put after saving device state completely. */
  if (!memory_manager_->SaveDirtyMemory(network_writer_, kDirtyMemoryTypeDma)) {
    MV_ERROR("failed to save dirty memory from dma");
    goto end;
  }

  if (!network_writer_->WaitForSignal(kMigrateDirtyMemoryFromDmaComplete)) {
    goto end;
  }

  if (!network_writer_->WaitForSignal(kMigrateDeviceComplete)) {
    goto end;
  }

  /* Save vcpu states */
  for (auto vcpu : vcpus_) {
    if (!vcpu->SaveState(network_writer_)) {
      MV_ERROR("failed to save vcpu=%d states", vcpu->vcpu_id());
      goto end;
    }
  }

  if (!network_writer_->WaitForSignal(kMigrateVcpuComplete)) {
    goto end;
  }

  /* Wait for a finish signal */
  if (!network_writer_->WaitForSignal(kMigrateComplete)) {
    goto end;
  }
  ret = true;

end:
  delete network_writer_;
  network_writer_ = nullptr;

  saving_ = false;
  memory_manager_->StopTrackingDirtyMemory();
  MV_LOG("done post-saving");
  return ret;
}

/* Load through network */
void Machine::Load(uint16_t port) {
  MV_ASSERT(!loading_);
  if (!IsPaused()) {
    Pause();
  }

  MV_LOG("start loading");
  loading_ = true;
  
  // Bind port to wait connection
  MigrationNetworkReader reader;
  if (!reader.WaitForConnection(port)) {
    MV_PANIC("failed to setup connection");
  }
  
  /* Load backing disk image */
  if (!io_thread_->LoadBackingDiskImage(&reader)) {
    MV_PANIC("failed to load backing disk image");
  }
  reader.SendSignal(kMigrateBackingImageComplete);

  /* Load system RAM */
  if (!memory_manager_->LoadState(&reader)) {
    MV_PANIC("failed to load system ram");
  }
  reader.SendSignal(kMigrateRamComplete);

  /* Load disk image, LoadDiskImage must be called before virtio-block loadstate */
  if (!io_thread_->LoadDiskImage(&reader)) {
    MV_PANIC("failed to load disk image");
  }
  reader.SendSignal(kMigrateImageComplete);

  /* Load dirty memory from kvm */
  if (!memory_manager_->LoadDirtyMemory(&reader, kDirtyMemoryTypeKvm)) {
    MV_PANIC("failed to load dirty memory from kvm");
  }
  reader.SendSignal(kMigrateDirtyMemoryFromKvmComplete);

  /* Load dirty memory from listener */
  if (!memory_manager_->LoadDirtyMemory(&reader, kDirtyMemoryTypeListener)) {
    MV_PANIC("failed to load dirty memory from listener");
  }
  reader.SendSignal(kMigrateDirtyMemoryFromListenerComplete);

  /* Load dirty memory from Dma */
  if (!memory_manager_->LoadDirtyMemory(&reader, kDirtyMemoryTypeDma)) {
    MV_PANIC("failed to load dirty memory from dma");
  }
  reader.SendSignal(kMigrateDirtyMemoryFromDmaComplete);

  /* Load device states */
  if (!device_manager_->LoadState(&reader)) {
    MV_PANIC("failed to load device states");
  }
  reader.SendSignal(kMigrateDeviceComplete);

  /* Load vcpu states */
  for (auto vcpu : vcpus_) {
    if (!vcpu->LoadState(&reader)) {
      MV_PANIC("failed to load %s", vcpu->name());
    }
  }
  reader.SendSignal(kMigrateVcpuComplete);

  /* Make a finish signal to notice the source vm */
  reader.SendSignal(kMigrateComplete);

  loading_ = false;
  MV_LOG("done loading");
}

/* Should be called by UI thread */
void Machine::Save(const std::string path) {
  MV_ASSERT(!saving_);
  /* Make sure the machine is paused */
  if (!IsPaused()) {
    Pause();
  }
  saving_ = true;
  MV_LOG("start saving");

  MigrationFileWriter writer(path);
  /* Save device states */
  if (!device_manager_->SaveState(&writer)) {
    MV_ERROR("failed to save device states");
    goto end;
  }
  /* Save vcpu states */
  for (auto vcpu : vcpus_) {
    vcpu->SaveState(&writer);
  }
  /* Save system RAM */
  if (!memory_manager_->SaveState(&writer)) {
    MV_ERROR("failed to save RAM");
    goto end;
  }
  /* Save disk images */
  if (!io_thread_->SaveDiskImage(&writer)) {
    MV_ERROR("failed to sync disk images");
    goto end;
  }
  /* Save configuration after saving disk images (paths might changed) */
  if (!config_->Save(path + "/configuration.yaml")) {
    MV_ERROR("failed to save configuration yaml");
    goto end;
  }

end:
  saving_ = false;
  MV_LOG("done saving");
}

/* Should be called by UI thread */
void Machine::Load(const std::string path) {
  MV_ASSERT(!loading_);
  /* Make sure the machine is paused */
  if (!IsPaused()) {
    Pause();
  }
  loading_ = true;
  MV_LOG("start loading");

  MigrationFileReader reader(path);
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

  if (saving_) {
    return "saving";
  }
  if (loading_) {
    return "loading";
  }
  if (!paused_) {
    return "running";
  }

  /* otherwise return paused */
  return "paused";
}
