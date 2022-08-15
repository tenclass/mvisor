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

#include <filesystem>

#include "logger.h"
#include "machine.h"
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

  if (event_fd_ > 0) {
    safe_close(&event_fd_);
  }
  if (epoll_fd_ > 0) {
    safe_close(&epoll_fd_);
  }
  for (auto it = epoll_events_.begin(); it != epoll_events_.end(); it++) {
    delete it->second;
  }
  for (auto timer : timers_) {
    if (!timer->removed) {
      MV_LOG("warning: timer %s is not removed when disconnected", timer->callback.target_type().name());
    }
    delete timer;
  }
}


void IoThread::Start() {

  thread_ = std::thread(&IoThread::RunLoop, this);

  StartPolling(event_fd_, EPOLLIN, [this](auto ret) {
    MV_UNUSED(ret);
    uint64_t tmp;
    read(event_fd_, &tmp, sizeof(tmp));
  });
}

void IoThread::Stop() {
  /* Just wakeup the thread and found machine is stopped */
  MV_ASSERT(!machine_->IsValid());
  Kick();
}

void IoThread::Kick() {
  if (event_fd_ > 0) {
    uint64_t tmp = 1;
    write(event_fd_, &tmp, sizeof(tmp));
  }
}

bool IoThread::IsCurrentThread() {
  return std::this_thread::get_id() == thread_.get_id();
}

void IoThread::RunLoop() {
  SetThreadName("mvisor-iothread");
  signal(SIGPIPE, SIG_IGN);

  struct epoll_event events[MAX_ENTRIES];

  while (true) {
    /* Execute timer events and calculate the next timeout */
    int64_t next_timeout_ns = CheckTimers();
    MV_ASSERT(next_timeout_ns > 0);

    /* Check paused state before sleep */
    while(machine_->IsPaused() && CanPauseNow()) {
      machine_->WaitToResume();
    }
    if (!machine_->IsValid()) {
      break;
    }

    /* epoll_wait limits to 1ms at least. epoll_pwait2 is only available after kernel 5.11 */
    int nfds = epoll_wait(epoll_fd_, events, MAX_ENTRIES, std::max(1LL, next_timeout_ns / 1000000LL));
    if (nfds < 0 && errno != EINTR) {
      MV_PANIC("nfds=%d", nfds);
      break;
    }
    
    for (int i = 0; i < nfds; i++) {
      auto event_it = epoll_events_.find(events[i].data.fd);
      if (event_it == epoll_events_.end()) {
        /* Maybe the fd is deleted just now */
        continue;
      }
      auto event = event_it->second;
      auto start_time = std::chrono::steady_clock::now();
      event->callback(events[i].events);

      if (machine_->debug()) {
        auto cost_us = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - start_time).count();
        if (cost_us >= 50000) {
          MV_LOG("%s SLOW fd=%d events=%d cost=%.3lfms", event->callback.target_type().name(),
            event->fd, events[i].events, double(cost_us) / 1000.0);
        }
      }
    }
  }

  if (machine_->debug()) MV_LOG("mvisor-iothread ended");
}

EpollEvent* IoThread::StartPolling(int fd, uint poll_mask, IoCallback callback) {
  EpollEvent* event = new EpollEvent;
  event->fd = fd;
  event->callback = callback;
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

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (epoll_events_.find(fd) != epoll_events_.end()) {
    MV_PANIC("repeated polling fd=%d, mask=0x%x, callback=%s",
      fd, poll_mask, callback.target_type().name());
  }
  epoll_events_[fd] = event;
  return event;
}

void IoThread::ModifyPolling(int fd, uint poll_mask) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = epoll_events_.find(fd);
  MV_ASSERT(it != epoll_events_.end());

  auto event = it->second;
  event->event.events = poll_mask;
  int ret = epoll_ctl(epoll_fd_, EPOLL_CTL_MOD, event->fd, &event->event);
  if (ret < 0) {
    MV_PANIC("failed to modify epoll event, ret=%d", ret);
  }
}

void IoThread::StopPolling(int fd) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
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

IoTimer* IoThread::AddTimer(int64_t interval_ns, bool permanent, VoidCallback callback) {
  IoTimer* timer = new IoTimer;
  timer->permanent = permanent;
  timer->interval_ns = interval_ns;
  timer->callback = std::move(callback);
  timer->removed = false;
  timer->next_timepoint = std::chrono::steady_clock::now() + std::chrono::nanoseconds(interval_ns);

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  timers_.push_back(timer);

  /* Wakeup io thread and recalculate the timeout */
  Kick();

  return timer;
}

void IoThread::RemoveTimer(IoTimer* timer) {
  timer->removed = true;
}

void IoThread::ModifyTimer(IoTimer* timer, int64_t interval_ns) {
  timer->interval_ns = interval_ns;
  timer->next_timepoint = std::chrono::steady_clock::now() + std::chrono::nanoseconds(interval_ns);
}

int64_t IoThread::CheckTimers() {
  auto now = std::chrono::steady_clock::now();
  int64_t min_timeout_ns = 100 * NS_PER_SECOND;

  std::vector<IoTimer*> triggered;

  mutex_.lock();
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
      if (delta_ns > 0 && delta_ns < min_timeout_ns) {
        min_timeout_ns = delta_ns;
      }
      ++it;
    }
  }
  mutex_.unlock();
  
  for (auto timer : triggered) {
    /* better check again, in case of removing a timer in during a timer event */
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

void IoThread::Schedule(VoidCallback callback) {
  AddTimer(0, false, std::move(callback));
}

void IoThread::RegisterDiskImage(DiskImage* image) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  disk_images_.insert(image);
}

void IoThread::UnregisterDiskImage(DiskImage* image) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  disk_images_.erase(image);
}

void IoThread::FlushDiskImages() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  for (auto image : disk_images_) {
    image->FlushAsync([](auto ret) {
      MV_UNUSED(ret);
    });
  }
}

bool IoThread::CanPauseNow() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (machine_->pausing_)
    return false;

  /* Drain all disk IO */
  for (auto image : disk_images_) {
    if (image->busy()) {
      return false;
    }
  }

  /* Drain all IO schedule jobs */
  for (auto timer : timers_) {
    if (timer->interval_ns == 0) {
      return false;
    }
  }

  return true;
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
    writer->SetPrefix(device.name());
    auto new_path = writer->base_path() + "/" + device.name();
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
  return true;
}
