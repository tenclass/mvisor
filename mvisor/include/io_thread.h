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

#include <sys/socket.h>
#include <sys/epoll.h>

#include <deque>
#include <set>
#include <unordered_map>
#include <thread>
#include <functional>
#include <chrono>
#include <mutex>

typedef std::function<void()> VoidCallback;
typedef std::function<void(long)> IoCallback;
typedef std::chrono::steady_clock::time_point IoTimePoint;

struct IoTimer {
  bool          permanent;
  int           interval_ms;
  IoTimePoint   next_timepoint;
  VoidCallback  callback;
  bool          removed;
};

struct EpollEvent {
  int           fd;
  IoCallback    callback;
  epoll_event   event;
};

class Machine;
class DiskImage;
class MigrationWriter;
class IoThread {
 public:
  IoThread(Machine* machine);
  ~IoThread();
  void Start();
  void Stop();
  void Kick();

  /* Async event polling */
  EpollEvent* StartPolling(int fd, uint poll_mask, IoCallback callback);
  void ModifyPolling(int fd, uint poll_mask);
  void StopPolling(int fd);

  /* Timer events handled by IO thread */
  IoTimer* AddTimer(int interval_ms, bool permanent, VoidCallback callback);
  void RemoveTimer(IoTimer* timer);
  void ModifyTimer(IoTimer* timer, int interval_ms);
  void Schedule(VoidCallback callback);

  /* Disk images */
  void RegisterDiskImage(DiskImage* image);
  void UnregisterDiskImage(DiskImage* image);
  void FlushDiskImages();
  bool SaveDiskImage(MigrationWriter* writer);
  uint GetDiskImageCount() { return disk_images_.size(); }

 private:
  void RunLoop();
  int  CheckTimers();
  bool CanPauseNow();

  std::thread           thread_;
  Machine*              machine_;
  std::recursive_mutex  mutex_;
  int                   event_fd_;
  int                   epoll_fd_;
  std::set<IoTimer*>    timers_;
  std::set<DiskImage*>  disk_images_;
  std::unordered_map<int, EpollEvent*>  epoll_events_;
};

#endif // _MVISOR_IO_THREAD_H
