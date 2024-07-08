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

/* 
 * MVisor Threading Model
 * Every device object is event-driven, so locks are not necessary, except
 * you are coding an interface method which should be called by a UI thread.
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
#include <condition_variable>
#include <queue>

#include "migration.h"
#include "io_thread.pb.h"

typedef std::function<void()> VoidCallback;
typedef std::function<void(long)> IoCallback;
typedef std::chrono::steady_clock::time_point IoTimePoint;

#define NS_PER_SECOND (1000000000LL)

class Device;
struct IoTimer {
  bool          permanent;
  int64_t       interval_ns;
  IoTimePoint   next_timepoint;
  VoidCallback  callback;
  bool          removed;
  Device*       device;
};

struct EpollEvent {
  int           fd;
  IoCallback    callback;
  epoll_event   event;
  Device*       device;
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
  void Kick();

  /* Async event polling */
  EpollEvent* StartPolling(Device* device, int fd, uint poll_mask, IoCallback callback);
  void StopPolling(int fd);

  /* Timer events handled by IO thread */
  IoTimer* AddTimer(Device* device, int64_t interval_ns, bool permanent, VoidCallback callback);
  void RemoveTimer(IoTimer** timer);
  void ModifyTimer(IoTimer* timer, int64_t interval_ns);
  void Schedule(Device* device, VoidCallback callback);

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
  bool    PreRun();

  std::thread           thread_;
  Machine*              machine_;
  std::mutex            mutex_;
  std::condition_variable wait_to_resume_;
  std::condition_variable wait_for_paused_;
  int                   wait_count_ = 0;
  int                   event_fd_;
  int                   epoll_fd_;
  bool                  paused_ = true;
  std::list<IoTimer*>   timers_;
  std::list<DiskImage*>  disk_images_;
  std::unordered_map<Qcow2Image*, std::queue<std::string>> qcow2_image_backing_files_;
  std::unordered_map<int, EpollEvent*>  epoll_events_;
  
  friend class IoThreadLockGuard;
};

// Pause Iothread
class IoThreadLockGuard {
 private:
  IoThread* iothread_;

 public:
  IoThreadLockGuard(IoThread* iothread) : iothread_(iothread) {
    Pause();
  }
  ~IoThreadLockGuard() {
    Resume();
  }

  void Pause() {
    std::unique_lock<std::mutex> lock(iothread_->mutex_);
    iothread_->wait_count_++;
    if (iothread_->paused_) {
      return;
    }
    iothread_->Kick();
    iothread_->wait_for_paused_.wait(lock, [this]() {
      return iothread_->paused_;
    });
  }

  void Resume() {
    std::unique_lock<std::mutex> lock(iothread_->mutex_);
    iothread_->wait_count_--;
    if (iothread_->wait_count_ == 0) {
      iothread_->wait_to_resume_.notify_all();
    }
  }
};

#endif // _MVISOR_IO_THREAD_H
