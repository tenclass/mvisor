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
#include <sys/eventfd.h>
#include <arpa/inet.h>
#include "logger.h"
#include "poll.h"
#include "machine.h"

#define MAX_ENTRIES 256

IoThread::IoThread(Machine* machine) : machine_(machine) {
  int ret = io_uring_queue_init(MAX_ENTRIES, &ring_, 0);
  MV_ASSERT(ret == 0);
  event_fd_ = eventfd(0, 0);
}

IoThread::~IoThread() {
  if (thread_.joinable()) {
    thread_.join();
  }

  if (event_fd_ > 0) {
    close(event_fd_);
  }

  /* Cleanup io operations */
  io_uring_queue_exit(&ring_);
}


void IoThread::Start() {

  thread_ = std::thread(&IoThread::RunLoop, this);

  StartPolling(event_fd_, POLLIN, [this](auto ret) {
    uint64_t tmp;
    read(event_fd_, &tmp, sizeof(tmp));
  });
  /* Send a NOP to wake up the thread */
  auto sqe = io_uring_get_sqe(&ring_);
  MV_ASSERT(sqe);
  io_uring_prep_nop(sqe);
}

void IoThread::Stop() {
  /* Just wakeup the thread and found machine is stopped */
  WakeUp();
}

void IoThread::WakeUp() {
  uint64_t tmp = 1;
  write(event_fd_, &tmp, sizeof(tmp));
}

void IoThread::RunLoop() {
  SetThreadName("mvisor-iothread");
  signal(SIGPIPE, SIG_IGN);

  struct __kernel_timespec timeout = {
    .tv_sec = 0,
    .tv_nsec = 1
  };
  struct io_uring_cqe* cqe;

  while (machine_->IsValid()) {
    /* Execute timer events and calculate the next timeout */
    int next_timeout_ms = CheckTimers();
    timeout.tv_nsec = next_timeout_ms * 1000000;
    int ret = io_uring_wait_cqe_timeout(&ring_, &cqe, &timeout);
    
    if (ret == 0) {
      /* Pop the ring item */
      mutex_.lock();
      void* data = io_uring_cqe_get_data(cqe);
      auto result = cqe->res;
      io_uring_cqe_seen(&ring_, cqe);
      
      /* IO thread is working */
      busy_ = true;
      mutex_.unlock();

      if (data) {
        auto request = reinterpret_cast<IoRequest*>(data);
        if (request->removed) {
          FreeIoRequest(request);
        } else {
          auto start_time = std::chrono::steady_clock::now();
          request->callback(result);

          if (machine_->debug()) {
            auto cost_us = std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now() - start_time).count();
            if (cost_us >= 50000) {
              MV_LOG("%s SLOW fd=%d type=%d res=%d cost=%.3lfms", request->callback.target_type().name(),
                request->fd, request->type, result, double(cost_us) / 1000.0);
            }
          }
          
          if (request->type == kIoRequestPoll) {
            /* keep polling */
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            auto sqe = io_uring_get_sqe(&ring_);
            MV_ASSERT(sqe);
            io_uring_prep_poll_add(sqe, request->fd, request->poll_mask);
            io_uring_sqe_set_data(sqe, request);
          } else {
            /* if not poll, delete the request object */
            FreeIoRequest(request);
          }
        }
      }

      /* IO thread finished */
      mutex_.lock();
      busy_ = false;
      mutex_.unlock();
    } else if (ret < 0) {
      if (ret != -ETIME && ret != -EAGAIN) {
        MV_PANIC("failed in io_uring_wait_cqe_timeout, ret=%d", ret);
      }
    }
  }
}

void IoThread::FreeIoRequest(IoRequest* request) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (requests_.find(request) != requests_.end()) {
    requests_.erase(request);
    delete request;
  }
}


void IoThread::SubmitRequest(IoRequest* request) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  requests_.insert(request);
  auto sqe = io_uring_get_sqe(&ring_);
  MV_ASSERT(sqe);
  request->input(sqe);
  io_uring_sqe_set_data(sqe, request);

  /* If caller is not IO thread, submit the item */
  if (!busy_) {
    io_uring_submit(&ring_);
  }
}

IoRequest* IoThread::Read(int fd, void* buffer, size_t bytes, off_t offset, IoCallback callback) {
  IoRequest* request = new IoRequest { .type = kIoRequestRead, .fd = fd, .callback = callback };
  request->input = [=](auto sqe) {
    io_uring_prep_read(sqe, fd, buffer, bytes, offset);
  };
  SubmitRequest(request);
  return request;
}

IoRequest* IoThread::Write(int fd, void* buffer, size_t bytes, off_t offset, IoCallback callback) {
  IoRequest* request = new IoRequest { .type = kIoRequestWrite, .fd = fd, .callback = callback };
  request->input = [=](auto sqe) {
    io_uring_prep_write(sqe, fd, buffer, bytes, offset);
  };
  SubmitRequest(request);
  return request;
}

IoRequest* IoThread::Connect(int fd, struct sockaddr_in address, IoCallback callback) {
  IoRequest* request = new IoRequest { .type = kIoRequestConnect, .fd = fd, .callback = callback };
  request->input = [=](auto sqe) {
    io_uring_prep_connect(sqe, fd, (struct sockaddr*)&address, sizeof(address));
  };
  SubmitRequest(request);
  return request;
}

IoRequest* IoThread::Send(int fd, void* buffer, size_t bytes, int flags, IoCallback callback) {
  IoRequest* request = new IoRequest { .type = kIoRequestSend, .fd = fd, .callback = callback };
  request->input = [=](auto sqe) {
    io_uring_prep_send(sqe, fd, buffer, bytes, flags);
  };
  SubmitRequest(request);
  return request;
}

IoRequest* IoThread::Receive(int fd, void* buffer, size_t bytes, int flags, IoCallback callback) {
  IoRequest* request = new IoRequest { .type = kIoRequestReceive, .fd = fd, .callback = callback };
  request->input = [=](auto sqe) {
    io_uring_prep_recv(sqe, fd, buffer, bytes, flags);
  };
  SubmitRequest(request);
  return request;
}

IoRequest* IoThread::FSync(int fd, bool data_sync, IoCallback callback) {
  IoRequest* request = new IoRequest { .type = kIoRequestFSync, .fd = fd, .callback = callback };
  request->input = [=](auto sqe) {
    io_uring_prep_fsync(sqe, fd, data_sync ? IORING_FSYNC_DATASYNC : 0);
  };
  SubmitRequest(request);
  return request;
}

IoRequest* IoThread::StartPolling(int fd, uint poll_mask, IoCallback callback) {
  IoRequest* request = new IoRequest {
    .type = kIoRequestPoll, .fd = fd, .poll_mask = poll_mask, .callback = callback
  };
  request->input = [=](auto sqe) {
    io_uring_prep_poll_add(sqe, fd, poll_mask);
  };
  SubmitRequest(request);
  return request;
}

void IoThread::ModifyPolling(IoRequest* request, uint poll_mask) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  MV_ASSERT(requests_.find(request) != requests_.end());
  MV_ASSERT(request->type == kIoRequestPoll);
  
  request->poll_mask = poll_mask;
}

void IoThread::CancelRequest(IoRequest* request) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  MV_ASSERT(requests_.find(request) != requests_.end());

  request->removed = true;
  if (request->type == kIoRequestPoll) {
    auto sqe = io_uring_get_sqe(&ring_);
    MV_ASSERT(sqe);
    io_uring_prep_poll_remove(sqe, (void*)request);
    io_uring_submit(&ring_);
  }
}

void IoThread::CancelRequest(int fd) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  for (auto r : requests_) {
    if (r->fd == fd) {
      CancelRequest(r);
    }
  }
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
  WakeUp();

  return timer;
}

void IoThread::RemoveTimer(IoTimer* timer) {
  timer->removed = true;
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
  for (auto it = timers_.begin(); it != timers_.end();) {
    auto timer = *it;
    if (timer->removed) {
      it = timers_.erase(it);
      delete timer;
    } else {
      auto delta_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timer->next_timepoint - now).count();
      if (delta_ms <= 0) {
        triggered.push_back(timer);
        timer->next_timepoint = now + std::chrono::milliseconds(timer->interval_ms);
        delta_ms = timer->interval_ms;
      }
      if (delta_ms < min_timeout_ms) {
        min_timeout_ms = delta_ms;
      }
      ++it;
    }
  }
  mutex_.unlock();
  
  for (auto timer : triggered) {
    timer->callback();
    if (!timer->permanent) {
      timer->removed = true;
      continue;
    }
  }
  return min_timeout_ms;
}

void IoThread::Schedule(VoidCallback callback) {
  AddTimer(0, false, callback);
}

