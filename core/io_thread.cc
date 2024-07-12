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


#include "io_thread.h"

#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/eventfd.h>
#include <sys/mman.h>

#include <filesystem>

#include "logger.h"
#include "machine.h"
#include "qcow2.h"
#include "disk_image.h"


#define MAX_ENTRIES 256

IoThread::IoThread(Machine* machine) : machine_(machine) {
  epoll_fd_ = epoll_create(MAX_ENTRIES);
  event_fd_ = eventfd(0, 0);
}

IoThread::~IoThread() {
  Kick();

  if (thread_.joinable()) {
    thread_.join();
  }

  safe_close(&event_fd_);
  safe_close(&epoll_fd_);

  for (auto it = epoll_events_.begin(); it != epoll_events_.end(); it++) {
    delete it->second;
  }
  for (auto timer : timers_) {
    if (!timer->removed) {
      MV_WARN("timer %s is not removed when disconnected", timer->callback.target_type().name());
    }
    delete timer;
  }

  for (auto it = qcow2_image_backing_files_.begin(); it != qcow2_image_backing_files_.end(); it++) {
    auto& files = it->second;
    while (!files.empty()) {
      auto file = files.front();
      files.pop();
      remove(file.c_str());
    }
  }
  qcow2_image_backing_files_.clear();
}

void IoThread::Start() {
  thread_ = std::thread(&IoThread::RunLoop, this);

  StartPolling(nullptr, event_fd_, EPOLLIN, [this](auto ret) {
    MV_UNUSED(ret);
    uint64_t tmp;
    read(event_fd_, &tmp, sizeof(tmp));
  });
}

void IoThread::Kick() {
  if (event_fd_ > 0) {
    uint64_t tmp = 1;
    write(event_fd_, &tmp, sizeof(tmp));
  }
  wait_to_resume_.notify_all();
}

/* Check paused state before epoll_wait */
bool IoThread::PreRun() {
  std::unique_lock<std::mutex> lock(mutex_);
  if (wait_count_ > 0) {
    // Someone wants me to pause, check if I can pause now
    // Drain all disk
    for (auto image : disk_images_) {
      if (image->busy()) {
        return true;
      }
    }

    // Drain all IO schedule jobs
    for (auto timer : timers_) {
      if (timer->interval_ns == 0) {
        return true;
      }
    }

    // Can pause now
    paused_ = true;
    wait_for_paused_.notify_all();
  }

  wait_to_resume_.wait(lock, [this] {
    bool should_wait = machine_->IsValid() && (machine_->IsPaused() || wait_count_ > 0);
    return !should_wait;
  });
  if (!machine_->IsValid()) {
    return false;
  }
  paused_ = false;
  return true;
}

void IoThread::RunLoop() {
  SetThreadName("mvisor-iothread");
  signal(SIGPIPE, SIG_IGN);

  struct epoll_event events[MAX_ENTRIES];

  while (true) {
    try {
      /* Execute timer events and calculate the next timeout */
      int64_t next_timeout_ns;
      do {
        next_timeout_ns = CheckTimers();
      } while (next_timeout_ns <= 0);

      if (!PreRun()) {
        break;
      }

      /* epoll_wait limits to 1ms at least. epoll_pwait2 is only available after kernel 5.11 */
      int nfds = epoll_wait(epoll_fd_, events, MAX_ENTRIES, std::max(1LL, next_timeout_ns / 1000000LL));
      if (nfds < 0 && errno != EINTR) {
        MV_PANIC("nfds=%d", nfds);
        break;
      }
      
      for (int i = 0; i < nfds; i++) {
        EpollEvent* event = nullptr;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          auto it = epoll_events_.find(events[i].data.fd);
          if (it != epoll_events_.end()) {
            event = it->second;
          }
        }
        if (event == nullptr) {
          /* Maybe the fd is deleted just now */
          continue;
        }

        auto start_time = std::chrono::steady_clock::now();
        if (event->device) {
          std::lock_guard<std::recursive_mutex> device_lock(event->device->mutex_);
          event->callback(events[i].events);
        } else {
          event->callback(events[i].events);
        }

        if (machine_->debug()) {
          auto cost_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - start_time).count();
          if (cost_us >= 50000) {
            MV_LOG("%s SLOW fd=%d events=%d cost=%.3lfms", event->callback.target_type().name(),
              event->fd, events[i].events, double(cost_us) / 1000.0);
          }
        }
      }
    } catch (const std::exception& e) {
      MV_LOG("iothread exception: %s", e.what());
      break;
    }
  }

  paused_ = true;
  if (machine_->debug()) MV_LOG("mvisor-iothread ended");
}

EpollEvent* IoThread::StartPolling(Device* device, int fd, uint poll_mask, IoCallback callback) {
  EpollEvent* event = new EpollEvent;
  event->fd = fd;
  event->callback = callback;
  event->device = device;
  event->event = epoll_event {
    .events = poll_mask,
    .data = {
      .fd = fd
    }
  };

  int ret = epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &event->event);
  if (ret < 0) {
    MV_PANIC("failed to add epoll event, ret=%d", ret);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (epoll_events_.find(fd) != epoll_events_.end()) {
    MV_PANIC("repeated polling fd=%d, mask=0x%x, callback=%s",
      fd, poll_mask, callback.target_type().name());
  }
  epoll_events_[fd] = event;
  return event;
}

void IoThread::StopPolling(int fd) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = epoll_events_.find(fd);
  MV_ASSERT(it != epoll_events_.end());

  auto event = it->second;
  int ret = epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, event->fd, &event->event);
  if (ret < 0) {
    MV_PANIC("failed to delete epoll event, ret=%d", ret);
  }
  epoll_events_.erase(it);
  delete event;
}

IoTimer* IoThread::AddTimer(Device* device, int64_t interval_ns, bool permanent, VoidCallback callback) {
  IoTimer* timer = new IoTimer;
  timer->device = device;
  timer->permanent = permanent;
  timer->interval_ns = interval_ns;
  timer->callback = std::move(callback);
  timer->removed = false;
  timer->next_timepoint = std::chrono::steady_clock::now() + std::chrono::nanoseconds(interval_ns);

  std::lock_guard<std::mutex> lock(mutex_);
  timers_.push_back(timer);

  /* Wakeup io thread and recalculate the timeout */
  Kick();

  return timer;
}

void IoThread::RemoveTimer(IoTimer** timer) {
  std::lock_guard<std::mutex> lock(mutex_);
  (*timer)->removed = true;
  *timer = nullptr;
}

void IoThread::ModifyTimer(IoTimer* timer, int64_t interval_ns) {
  timer->interval_ns = interval_ns;
  timer->next_timepoint = std::chrono::steady_clock::now() + std::chrono::nanoseconds(interval_ns);
}

int64_t IoThread::CheckTimers() {
  auto now = std::chrono::steady_clock::now();
  int64_t min_timeout_ns = 100 * NS_PER_SECOND;

  std::vector<IoTimer*> triggered;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = timers_.begin(); it != timers_.end();) {
      auto timer = *it;
      if (timer->removed) {
        it = timers_.erase(it);
        delete timer;
      } else {
        auto delta_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(timer->next_timepoint - now).count();
        if (delta_ns <= 0) {
          triggered.push_back(timer);
          timer->next_timepoint = now + std::chrono::nanoseconds(timer->interval_ns);
          delta_ns = timer->interval_ns;
        }
        if (delta_ns < min_timeout_ns) {
          min_timeout_ns = delta_ns;
        }
        ++it;
      }
    }
  }
  
  for (auto timer : triggered) {
    std::lock_guard<std::recursive_mutex> device_lock(timer->device->mutex_);
    /* better check again, in case of removing a timer in during a timer event or device IO */
    if (timer->removed) {
      continue;
    }
    timer->callback();

    if (!timer->permanent) {
      timer->removed = true;
      continue;
    }
  }
  return min_timeout_ns;
}

void IoThread::Schedule(Device* device, VoidCallback callback) {
  AddTimer(device, 0, false, std::move(callback));
}

void IoThread::RegisterDiskImage(DiskImage* image) {
  std::lock_guard<std::mutex> lock(mutex_);
  disk_images_.push_back(image);
}

void IoThread::UnregisterDiskImage(DiskImage* image) {
  std::lock_guard<std::mutex> lock(mutex_);
  disk_images_.remove(image);
}

void IoThread::FlushDiskImages() {
  std::lock_guard<std::mutex> lock(mutex_);

  for (auto image : disk_images_) {
    ImageIoRequest r = {
      .type = kImageIoFlush
    };
    image->QueueIoRequest(r, [](auto ret) {
      MV_UNUSED(ret);
    });
  }
}

/* Make sure call Flush() before save disk images */
bool IoThread::SaveDiskImage(MigrationWriter* writer) {
  for (auto image : disk_images_) {
    auto& device = *image->deivce();
    if (image->readonly())
      continue;
    if (!image->snapshot())
      continue;

    /* Only copy snapshot image */
    auto network_writer = dynamic_cast<MigrationNetworkWriter*>(writer);
    if (network_writer) {
      if (!dynamic_cast<Qcow2Image*>(image)) {
        return false;
      }
      if (!network_writer->WriteFromFile("IMAGE", image->filepath(), QCOW2_MIGRATE_DATA_OFFSET)) {
        return false;
      }
    } else {
      auto file_writer = dynamic_cast<MigrationFileWriter*>(writer);
      MV_ASSERT(file_writer);

      file_writer->SetPrefix(device.name());
      auto new_path = file_writer->base_path() + "/" + device.name();
      if (!std::filesystem::exists(new_path)) {
        std::filesystem::create_directories(new_path);
      }
      new_path += "/disk.qcow2";
      if (std::filesystem::exists(new_path)) {
        std::filesystem::remove(new_path);
      }
      std::filesystem::copy_file(image->filepath(), new_path);
      device["image"] = new_path;
    }
  }
  return true;
}

// only could be called when vm was paused
bool IoThread::CreateQcow2ImageSnapshot() {
  for (auto image : disk_images_) {
    auto qcow2_image = dynamic_cast<Qcow2Image*>(image);
     if (!qcow2_image) {
      MV_ERROR("snapshot was only supported by qcow2");
      return false;
    }

    auto image_path = qcow2_image->filepath();
    if (!qcow2_image->CreateSnapshot()) {
      MV_ERROR("failed to create snapshot for qcow2 image=%s", image_path);
      return false;
    }
    qcow2_image_backing_files_[qcow2_image].push(image_path);
  }
  return true;
}

bool IoThread::SaveBackingDiskImage(MigrationNetworkWriter* writer) {
  for (auto image : disk_images_) {
    auto qcow2_image = dynamic_cast<Qcow2Image*>(image);
    auto files(qcow2_image_backing_files_[qcow2_image]);

    ImageDescriptor image_descriptor;
    image_descriptor.set_offset(QCOW2_MIGRATE_DATA_OFFSET);
    while (!files.empty()) {
      if (!writer->WriteProtobuf("IMAGE_DESCRIPTOR", image_descriptor)) {
        return false;
      }
      if (!writer->WriteFromFile("BACKING_IMAGE", files.front(), image_descriptor.offset())) {
        return false;
      }
      files.pop();
    }

    // offset=-1 means we transfer backing images data completely for current image
    image_descriptor.set_offset(-1);
    if (!writer->WriteProtobuf("IMAGE_DESCRIPTOR", image_descriptor)) {
      return false;
    }
  }
  return true;
}

bool IoThread::LoadBackingDiskImage(MigrationNetworkReader* reader) {
  for (auto image : disk_images_) { 
    auto qcow2_image = dynamic_cast<Qcow2Image*>(image);
    if (!qcow2_image) {
      MV_ERROR("snapshot was only supported by qcow2");
      return false;
    }

    ImageDescriptor image_descriptor;
    while (true) {
      if (!reader->ReadProtobuf("IMAGE_DESCRIPTOR", image_descriptor)) {
        MV_ERROR("failed to read disk image descriptor");
        return false;
      }

      if (image_descriptor.offset() == -1) {
        break;
      }
      
      reader->ReadToFile("BACKING_IMAGE", qcow2_image->filepath(), image_descriptor.offset());
      qcow2_image_backing_files_[qcow2_image].push(qcow2_image->filepath());
      if (!qcow2_image->CreateSnapshot()) {
        MV_ERROR("failed to create disk image snapshot, image=%s", qcow2_image->filepath().c_str());
        return false;
      }
    }
  }
  return true;
}

bool IoThread::LoadDiskImage(MigrationNetworkReader* reader) {
  for (auto image : disk_images_) {
    if (!dynamic_cast<Qcow2Image*>(image)) {
      MV_ERROR("migration was only supported by qcow2");
      return false;
    }
    reader->ReadToFile("IMAGE", image->filepath(), QCOW2_MIGRATE_DATA_OFFSET);
  }
  return true;
}
