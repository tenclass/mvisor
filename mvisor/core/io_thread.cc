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
#include "logger.h"
#include "machine.h"

#define MAX_ENTRIES 256

IoThread::IoThread(Machine* machine) : machine_(machine) {
  int ret = io_uring_queue_init(MAX_ENTRIES, &ring_, 0);
  MV_ASSERT(ret == 0);
}

IoThread::~IoThread() {
  if (thread_.joinable()) {
    thread_.join();
  }

  /* Cleanup io operations */
  io_uring_queue_exit(&ring_);
}


void IoThread::Start() {
  thread_ = std::thread(&IoThread::RunLoop, this);
}

void IoThread::Stop() {
  /* Wakeup io thread */
  auto sqe = io_uring_get_sqe(&ring_);
  io_uring_prep_nop(sqe);
  io_uring_submit(&ring_);

}

void IoThread::RunLoop() {
  SetThreadName("mvisor-iothread");

  struct __kernel_timespec timeout = {
    .tv_sec = 0,
    .tv_nsec = 1
  };
  struct io_uring_cqe* cqe;

  while (machine_->IsValid()) {
    int next_timeout_ms = CheckTimers();
    timeout.tv_nsec = next_timeout_ms * 1000000;
    int ret = io_uring_wait_cqe_timeout(&ring_, &cqe, &timeout);
    if (ret < 0) {
      if (ret == -ETIME) {
        continue;
      }
      MV_PANIC("failed in io_uring_wait_cqe_timeout, ret=%d", ret);
    }
    
    // ret is the number of events
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    for (int i = 0; i < 1; i++) {
      void* data = io_uring_cqe_get_data(&cqe[i]);
      if (data) {
        auto request = reinterpret_cast<IoRequest*>(data);
        if (request->removed) {
          requests_.erase(request);
          delete request;
        } else {
          request->callback(cqe[i].res);
          if (request->type != kIoRequestPoll) {
            /* if not poll, delete the request object */
            requests_.erase(request);
            delete request;
          } else {
            auto sqe = io_uring_get_sqe(&ring_);
            io_uring_prep_poll_add(sqe, request->fd, request->poll_mask);
            io_uring_sqe_set_data(sqe, request);
          }
        }
      }
      io_uring_cqe_seen(&ring_, &cqe[i]);
    }
  }
}

IoRequest* IoThread::Read(int fd, void* buffer, size_t bytes, off_t offset, IoCallback callback) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  IoRequest* r = new IoRequest {
    .type = kIoRequestRead,
    .fd = fd,
    .callback = callback
  };
  requests_.insert(r);

  auto sqe = io_uring_get_sqe(&ring_);
  io_uring_prep_read(sqe, fd, buffer, bytes, offset);
  io_uring_sqe_set_data(sqe, r);
  io_uring_submit(&ring_);
  return r;
}

IoRequest* IoThread::Write(int fd, void* buffer, size_t bytes, off_t offset, IoCallback callback) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  IoRequest* r = new IoRequest {
    .type = kIoRequestWrite,
    .fd = fd,
    .callback = callback
  };
  requests_.insert(r);

  auto sqe = io_uring_get_sqe(&ring_);
  io_uring_prep_write(sqe, fd, buffer, bytes, offset);
  io_uring_sqe_set_data(sqe, r);
  io_uring_submit(&ring_);
  return r;
}

IoRequest* IoThread::FSync(int fd, bool data_sync, IoCallback callback) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  IoRequest* r = new IoRequest {
    .type = kIoRequestWrite,
    .fd = fd,
    .callback = callback
  };
  requests_.insert(r);

  auto sqe = io_uring_get_sqe(&ring_);
  io_uring_prep_fsync(sqe, fd, data_sync ? IORING_FSYNC_DATASYNC : 0);
  io_uring_sqe_set_data(sqe, r);
  io_uring_submit(&ring_);
  return r;
}

IoRequest* IoThread::StartPolling(int fd, uint poll_mask, IoCallback callback) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  IoRequest* r = new IoRequest {
    .type = kIoRequestPoll,
    .fd = fd,
    .poll_mask = poll_mask,
    .callback = callback
  };
  requests_.insert(r);

  auto sqe = io_uring_get_sqe(&ring_);
  io_uring_prep_poll_add(sqe, fd, poll_mask);
  io_uring_sqe_set_data(sqe, r);
  io_uring_submit(&ring_);
  return r;
}

void IoThread::ModifyPolling(IoRequest* request, uint poll_mask) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  MV_ASSERT(requests_.find(request) != requests_.end());
  MV_ASSERT(request->type == kIoRequestPoll);
  
  request->poll_mask = poll_mask;
}

void IoThread::StopPolling(IoRequest* request) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  MV_ASSERT(requests_.find(request) != requests_.end());
  MV_ASSERT(request->type == kIoRequestPoll);

  request->removed = true;
  auto sqe = io_uring_get_sqe(&ring_);
  io_uring_prep_poll_remove(sqe, (void*)request);
  io_uring_submit(&ring_);
}

void IoThread::StopPolling(int fd) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  for (auto r : requests_) {
    if (r->type == kIoRequestPoll && r->fd == fd) {
      StopPolling(r);
      return;
    }
  }
  MV_PANIC("not found fd=%d", fd);
}

IoTimer* IoThread::AddTimer(int interval_ms, bool permanent, VoidCallback callback) {
  IoTimer* timer = new IoTimer {
    .permanent = permanent,
    .interval_ms = interval_ms,
    .callback = callback
  };
  timer->next_timepoint = std::chrono::steady_clock::now() + std::chrono::milliseconds(interval_ms);

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  timers_.insert(timer);

  /* Wakeup io thread and recalculate the timeout */
  auto sqe = io_uring_get_sqe(&ring_);
  io_uring_prep_nop(sqe);
  io_uring_submit(&ring_);

  return timer;
}

void IoThread::RemoveTimer(IoTimer* timer) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  timers_.erase(timer);
  delete timer;
}

void IoThread::ModifyTimer(IoTimer* timer, int interval_ms) {
  timer->interval_ms = interval_ms;
  timer->next_timepoint = std::chrono::steady_clock::now() + std::chrono::milliseconds(interval_ms);
}

int IoThread::CheckTimers() {
  auto now = std::chrono::steady_clock::now();
  int64_t min_timeout_ms = 100000;

  std::vector<IoTimer*> triggered;

  mutex_.lock();
  for (auto timer : timers_) {
    auto delta_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timer->next_timepoint - now).count();
    if (delta_ms <= 0) {
      triggered.push_back(timer);
      timer->next_timepoint = now + std::chrono::milliseconds(timer->interval_ms);
      delta_ms = timer->interval_ms;
    }
    if (delta_ms < min_timeout_ms) {
      min_timeout_ms = delta_ms;
    }
  }
  mutex_.unlock();
  
  for (auto timer : triggered) {
    timer->callback();
    if (!timer->permanent) {
      RemoveTimer(timer);
      continue;
    }
  }
  return min_timeout_ms;
}

void IoThread::Schedule(VoidCallback callback) {
  AddTimer(0, false, callback);
}

