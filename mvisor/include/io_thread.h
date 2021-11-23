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

#include <libaio.h>
#include <thread>
#include <functional>

enum IoRequestType {
  kIoRequestRead,
  kIoRequestWrite
};

typedef std::function<void()> IoCallback;
struct IoRequest {
  enum IoRequestType type;
  IoCallback callback;
  // io control block ??
  struct iocb iocb;
};

class Machine;

class IoThread {
 public:
  IoThread(Machine* machine);
  ~IoThread();

  void Start();
  const IoRequest* QueueIo(IoRequestType type, int fd, void* buffer, size_t bytes,
    off_t offset, IoCallback callback);
  const IoRequest* QueueIov(IoRequestType type, int fd, const struct iovec* iov, int iov_count,
    off_t offset, IoCallback callback);
  void CancelIo(const IoRequest* request);

 private:
  void EventLoop();

  std::thread thread_;
  Machine* machine_;
  io_context_t context_;
};

#endif // _MVISOR_IO_THREAD_H
