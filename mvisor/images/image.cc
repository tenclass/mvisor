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

DiskImage::DiskImage() {
}

DiskImage::~DiskImage()
{
  if (!finalized_) {
    Finalize();
  }
}

DiskImage* DiskImage::Create(Device* device, std::string path, bool readonly) {
  DiskImage* image;
  if (path.find(".qcow2") != std::string::npos) {
    image = dynamic_cast<DiskImage*>(Object::Create("qcow2-image"));
  } else {
    image = dynamic_cast<DiskImage*>(Object::Create("raw-image"));
  }
  MV_ASSERT(image);
  image->device_ = device;
  image->Initialize(path, readonly);
  
  image->io_ = device->manager()->io();
  image->worker_thread_ = std::thread(&DiskImage::WorkerProcess, image);
  return image;
}

void DiskImage::Connect() {
  if (!initialized_) {
    initialized_ = true;
    std::string path = std::get<std::string>(key_values_["path"]);
    bool readonly = std::get<bool>(key_values_["readonly"]);
    Initialize(path, readonly);
  }
}

ssize_t DiskImage::Discard(off_t position, size_t length) {
  return 0;
}

void DiskImage::Finalize() {
  finalized_ = true;

  if (worker_thread_.joinable()) {
    worker_cv_.notify_all();
    worker_thread_.join();
  }
}


void DiskImage::WorkerProcess() {
  SetThreadName("mvisor-qcow2");
  
  while (!finalized_) {
    std::unique_lock<std::mutex> lock(worker_mutex_);
    worker_cv_.wait(lock, [this]() {
      return !worker_queue_.empty() || finalized_;
    });

    if (worker_queue_.empty()) {
      break;
    }
    auto callback = worker_queue_.front();
    worker_queue_.pop_front();
    lock.unlock();

    callback();
  }
}

void DiskImage::ReadAsync(void *buffer, off_t position, size_t length, IoCallback callback) {
  worker_mutex_.lock();
  worker_queue_.push_back([this, buffer, position, length, callback]() {
    auto ret = Read(buffer, position, length);
    io_->Schedule([=]() { callback(ret); });
  });
  worker_cv_.notify_all();
  worker_mutex_.unlock();
}

void DiskImage::WriteAsync(void *buffer, off_t position, size_t length, IoCallback callback) {
  if (readonly_) {
    return callback(0);
  }

  worker_mutex_.lock();
  worker_queue_.push_back([this, buffer, position, length, callback]() {
    auto ret = Write(buffer, position, length);
    io_->Schedule([=]() { callback(ret); });
  });
  worker_cv_.notify_all();
  worker_mutex_.unlock();
}

void DiskImage::DiscardAsync(off_t position, size_t length, IoCallback callback) {
  if (readonly_) {
    return callback(0);
  }

  worker_mutex_.lock();
  worker_queue_.push_back([this, position, length, callback]() {
    auto ret = Discard(position, length);
    io_->Schedule([=]() { callback(ret); });
  });
  worker_cv_.notify_all();
  worker_mutex_.unlock();
}

void DiskImage::FlushAsync(IoCallback callback) {
  worker_mutex_.lock();
  worker_queue_.push_back([this, callback]() {
    auto ret = Flush();
    io_->Schedule([=]() {
      callback(ret);
    });
  });
  worker_cv_.notify_all();
  worker_mutex_.unlock();
}
