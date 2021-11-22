#include "io_thread.h"
#include <cstring>
#include "logger.h"
#include "machine.h"

#define MAX_EVENTS 256

IoThread::IoThread(Machine* machine) : machine_(machine) {
  bzero(&context_, sizeof(context_));
}

IoThread::~IoThread() {
  if (thread_.joinable()) {
    thread_.join();
  }
}

void IoThread::Start() {
  MV_ASSERT(io_setup(MAX_EVENTS, &context_) == 0);
  thread_ = std::thread(&IoThread::EventLoop, this);
}

void IoThread::EventLoop() {
  struct timespec timeout = {
    .tv_sec = 1,
    .tv_nsec = 0
  };
  const long min_nr = 1, max_nr = MAX_EVENTS;
  struct io_event events[max_nr];

  while (machine_->IsValid()) {
    int ret = io_getevents(context_, min_nr, max_nr, events, &timeout);
    if (ret < 0) {
      if (ret == -EINTR)
        continue;
      MV_PANIC("failed in io_getevents, ret=%d", ret);
    }
  
    if (ret == 0) {
      /* timeout */
      continue;
    }
    
    // ret is the number of events
    for (int i = 0; i < ret; i++) {
      auto request = reinterpret_cast<IoRequest*>(events[i].data);
      request->callback();
      delete request;
    }
  }

  /* Cleanup io operations */
  io_destroy(context_);
}


const IoRequest* IoThread::QueueIo(IoRequestType type, int fd, void* buffer, size_t bytes,
  off_t offset, IoCallback callback) {

  IoRequest* r = new IoRequest;
  if (type == kIoRequestRead) {
    io_prep_pread(&r->iocb, fd, buffer, bytes, offset);
  } else {
    io_prep_pwrite(&r->iocb, fd, buffer, bytes, offset);
  }
  r->iocb.data = r;
  r->type = type;
  r->callback = callback;

  struct iocb* p = &r->iocb;  
  int ret = io_submit(context_, 1, &p);
  if (ret != 1) {
    MV_PANIC("failed to submit io request, ret=%d", ret);
  }
  return r;
}


const IoRequest* IoThread::QueueIov(IoRequestType type, int fd, const struct iovec* iov, int iov_count,
  off_t offset, IoCallback callback) {

  IoRequest* r = new IoRequest;
  if (type == kIoRequestRead) {
    io_prep_preadv(&r->iocb, fd, iov, iov_count, offset);
  } else {
    io_prep_pwritev(&r->iocb, fd, iov, iov_count, offset);
  }
  r->iocb.data = r;
  r->type = type;
  r->callback = callback;

  struct iocb* p = &r->iocb;  
  int ret = io_submit(context_, 1, &p);
  if (ret != 1) {
    MV_PANIC("failed to submit io request, ret=%d", ret);
  }
  return r;
}

void IoThread::CancelIo(const IoRequest* request) {
  io_event event;
  int ret = io_cancel(context_, (struct iocb*)&request->iocb, &event);
  if (ret != 0) {
    MV_PANIC("failed to cancel io request, ret=%d", ret);
  }
}
