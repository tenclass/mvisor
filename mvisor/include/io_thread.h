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
