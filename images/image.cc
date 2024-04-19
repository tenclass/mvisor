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

#include "disk_image.h"

#include "logger.h"
#include "utilities.h"
#include "device_manager.h"
#include "pci_device.h"
#include "qcow2.h"


DiskImage::DiskImage() {
}

DiskImage::~DiskImage()
{
  if (!finalized_) {
    Finalize();
  }
}

DiskImage* DiskImage::Create(Device* device, std::string path, bool readonly, bool snapshot) {
  DiskImage* image;
  if (path.find(".qcow2") != std::string::npos) {
    image = dynamic_cast<Qcow2Image*>(Object::Create("qcow2-image"));
  } else {
    image = dynamic_cast<DiskImage*>(Object::Create("raw-image"));
  }
  MV_ASSERT(image);
  image->filepath_ = path;
  image->readonly_ = readonly;
  image->snapshot_ = snapshot;
  image->device_ = device;
  image->Initialize();
  
  image->io_ = device->manager()->io();
  image->worker_thread_ = std::thread(&DiskImage::WorkerProcess, image);
  return image;
}

void DiskImage::Finalize() {
  finalized_ = true;

  if (worker_thread_.joinable()) {
    worker_cv_.notify_all();
    worker_thread_.join();
  }
}

void DiskImage::WorkerProcess() {
  SetThreadName("mvisor-disk");
  
  io_->RegisterDiskImage(this);

  while (!finalized_) {
    std::unique_lock<std::mutex> lock(worker_mutex_);
    worker_cv_.wait(lock, [this]() {
      return !worker_queue_.empty() || finalized_;
    });

    if (finalized_) {
      break;
    }

    auto& callback = worker_queue_.front();
    lock.unlock();
  
    callback();

    /* Only remove item after job is done.
     * Remember to lock mutex again when operating on worker_queue_
     */
    lock.lock();
    worker_queue_.pop_front();
  }
  
  io_->UnregisterDiskImage(this);
}


void DiskImage::QueueIoRequest(ImageIoRequest request, IoCallback callback) {
  worker_mutex_.lock();
  worker_queue_.emplace_back([this, request = std::move(request), callback = std::move(callback)]() {
    auto ret = HandleIoRequest(std::move(request));
    std::lock_guard<std::recursive_mutex> device_lock(device_->mutex());
    callback(ret);
  });

  worker_mutex_.unlock();
  worker_cv_.notify_all();
}

void DiskImage::QueueMultipleIoRequests(std::vector<ImageIoRequest> requests, IoCallback callback) {
  worker_mutex_.lock();
  worker_queue_.emplace_back([this, requests = std::move(requests), callback = std::move(callback)]() {
    long ret, total = 0;
    for (auto &req: requests) {
      ret = HandleIoRequest(req);
      if (ret < 0) {
        total = ret;
        break;
      }
      total += ret;
    }

    std::lock_guard<std::recursive_mutex> device_lock(device_->mutex());
    callback(total);
  });

  worker_mutex_.unlock();
  worker_cv_.notify_all();
}

bool DiskImage::busy() {
  std::lock_guard<std::mutex> lock(worker_mutex_);
  return !worker_queue_.empty();
}
