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
#include <list>
#include <unordered_map>
#include <thread>
#include <functional>
#include <chrono>
#include <mutex>
#include <queue>

#include "migration.h"
#include "image.pb.h"

typedef std::function<void()> VoidCallback;
typedef std::function<void(long)> IoCallback;
typedef std::chrono::steady_clock::time_point IoTimePoint;

#define NS_PER_SECOND (1000000000LL)

struct IoTimer {
  bool          permanent;
  int64_t       interval_ns;
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
class Qcow2Image;
class MigrationWriter;
class IoThread {
 public:
  IoThread(Machine* machine);
  ~IoThread();
  void Start();
  void Stop();
  void Kick();
  bool IsCurrentThread();

  /* Async event polling */
  EpollEvent* StartPolling(int fd, uint poll_mask, IoCallback callback);
  void ModifyPolling(int fd, uint poll_mask);
  void StopPolling(int fd);

  /* Timer events handled by IO thread */
  IoTimer* AddTimer(int64_t interval_ns, bool permanent, VoidCallback callback);
  void RemoveTimer(IoTimer* timer);
  void ModifyTimer(IoTimer* timer, int64_t interval_ns);
  void Schedule(VoidCallback callback);

  /* Disk images */
  void RegisterDiskImage(DiskImage* image);
  void UnregisterDiskImage(DiskImage* image);
  void FlushDiskImages();
  bool SaveDiskImage(MigrationWriter* writer);
  bool LoadDiskImage(MigrationNetworkReader* reader);
  bool SaveBackingDiskImage(MigrationNetworkWriter* writer);
  bool LoadBackingDiskImage(MigrationNetworkReader* reader);
  bool CreateQcow2ImageSnapshot();
  uint GetDiskImageCount() { return disk_images_.size(); }

 private:
  void    RunLoop();
  int64_t CheckTimers();
  bool    CanPauseNow();

  std::thread           thread_;
  Machine*              machine_;
  std::recursive_mutex  mutex_;
  int                   event_fd_;
  int                   epoll_fd_;
  std::list<IoTimer*>   timers_;
  std::list<DiskImage*>  disk_images_;
  std::unordered_map<Qcow2Image*, std::queue<std::string>> qcow2_image_backing_files_;
  std::unordered_map<int, EpollEvent*>  epoll_events_;
};

#endif // _MVISOR_IO_THREAD_H
