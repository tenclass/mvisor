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

#ifndef _MVISOR_IO_THREAD_H
#define _MVISOR_IO_THREAD_H

#include <liburing.h>
#include <thread>
#include <functional>
#include <chrono>
#include <unordered_set>
#include <mutex>

typedef std::function<void(long)> IoCallback;
typedef std::function<void()> VoidCallback;
typedef std::chrono::steady_clock::time_point IoTimePoint;

struct IoTimer {
  bool          permanent;
  int           interval_ms;
  IoTimePoint   next_timepoint;
  VoidCallback  callback;
};

enum IoRequestType {
  kIoRequestPoll,
  kIoRequestRead,
  kIoRequestWrite
};

struct IoRequest {
  enum IoRequestType  type;
  int                 fd;
  uint                poll_mask;
  IoCallback          callback;
  bool                removed;
};

class Machine;

class IoThread {
 public:
  IoThread(Machine* machine);
  ~IoThread();
  void Start();
  void Stop();

  IoRequest* Read(int fd, void* buffer, size_t bytes, off_t offset, IoCallback callback);
  IoRequest* Write(int fd, void* buffer, size_t bytes, off_t offset, IoCallback callback);
  IoRequest* FSync(int fd, bool data_sync, IoCallback callback);
  IoRequest* StartPolling(int fd, uint poll_mask, IoCallback callback);
  void ModifyPolling(IoRequest* request, uint poll_mask);
  void StopPolling(IoRequest* request);
  void StopPolling(int fd);

  IoTimer* AddTimer(int interval_ms, bool permanent, VoidCallback callback);
  void RemoveTimer(IoTimer* timer);
  void ModifyTimer(IoTimer* timer, int interval_ms);
  void Schedule(VoidCallback callback);

 private:
  void RunLoop();
  int CheckTimers();
  void FreeIoRequest(IoRequest* request);
  void WakeUp();

  std::thread           thread_;
  Machine*              machine_;
  struct io_uring       ring_;
  std::recursive_mutex  mutex_;
  std::unordered_set<IoTimer*>    timers_;
  std::unordered_set<IoRequest*>  requests_;
  int                   event_fd_;
};

#endif // _MVISOR_IO_THREAD_H
