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

#ifndef _MVISOR_IMAGE_H
#define _MVISOR_IMAGE_H

#include <string>
#include <functional>
#include <thread>
#include <mutex>
#include <deque>
#include <condition_variable>

#include "utilities.h"
#include "object.h"
#include "io_thread.h"

typedef std::function<void(ssize_t ret)> IoCallback;

enum ImageIoType {
  kImageIoInformation,
  kImageIoRead,
  kImageIoWrite,
  kImageIoDiscard,
  kImageIoFlush
};

struct ImageInformation {
  /* Disk size is block_size * total_blocks */
  size_t block_size;
  size_t total_blocks;
};

class Device;
class DiskImage : public Object {
 public:
  static DiskImage* Create(Device* device, std::string path, bool readonly);

  DiskImage();
  virtual ~DiskImage();
  virtual void Connect();
  bool readonly() { return readonly_; }

  /* Always use this static method to create a DiskImage */

  /* Interface for a image format to implement */
  virtual ImageInformation information() = 0;
  virtual ssize_t Read(void *buffer, off_t position, size_t length) = 0;
  virtual ssize_t Write(void *buffer, off_t position, size_t length) = 0;
  virtual ssize_t Flush() = 0;
  /* Optional */
  virtual ssize_t Discard(off_t position, size_t length);

  /* Interface for user */
  virtual void ReadAsync(void *buffer, off_t position, size_t length, IoCallback callback);
  virtual void WriteAsync(void *buffer, off_t position, size_t length, IoCallback callback);
  virtual void DiscardAsync(off_t position, size_t length, IoCallback callback);
  virtual void FlushAsync(IoCallback callback);

 protected:
  bool        initialized_ = false;
  bool        readonly_ = false;
  Device*     device_ = nullptr;
  IoThread*   io_ = nullptr;

  virtual void Initialize(const std::string& path, bool readonly) = 0;
  virtual void Finalize();

 private:
  /* Worker thread to implemente Async IO */
  std::thread worker_thread_;
  std::mutex  worker_mutex_;
  std::condition_variable   worker_cv_;
  std::deque<VoidCallback>  worker_queue_;
  bool        finalized_ = false;

  void WorkerProcess();
};


#endif // _MVISOR_DISK_IMAGE_H
