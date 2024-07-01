#include <sys/eventfd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/poll.h>

#include "server.h"
#include "connection.h"
#include "logger.h"

#define POLL_FD_NUM 16

VncServer::VncServer(Machine* machine, uint16_t port) : machine_(machine), port_(port) {
  event_fd_ = eventfd(0, 0);
  MV_ASSERT(event_fd_ >= 0);

  server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  MV_ASSERT(server_fd_ >= 0);

  int opt = 1;
  if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    MV_PANIC("failed to set SO_REUSEADDR");
  }

  struct sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port);

  if (bind(server_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    MV_PANIC("failed to bind");
  }

  if (listen(server_fd_, 1) < 0) {
    MV_PANIC("failed to listen");
  }

  MV_LOG("VNC server started port=%d", port);
}

VncServer::~VncServer() {
  for (auto conn : connections_) {
    delete conn;
  }

  safe_close(&event_fd_);
  safe_close(&server_fd_);
}

void VncServer::Close() {
  safe_close(&server_fd_);

  if (event_fd_ != -1) {
    uint64_t tmp = 1;
    write(event_fd_, &tmp, sizeof(tmp));
  }
}

void VncServer::SetPassword(const std::string& password) {
  security_type_ = kVncSecurityVncAuth;
  password_ = password;
}

void VncServer::MainLoop() {
  SetThreadName("mvisor-vnc-server");

  while (machine_->IsValid() && server_fd_ != -1) {
    pollfd fds[POLL_FD_NUM] = {
      { .fd = event_fd_, .events = POLLIN },
      { .fd = server_fd_, .events = POLLIN },
    };

    int fd_num = 2;
    for (auto conn : connections_) {
      fds[fd_num].fd = conn->fd();
      fds[fd_num].events = POLLIN | POLLERR;
      ++fd_num;
    }
    

    /* Poll and wait infinitely */
    int ret = poll(fds, fd_num, -1);
    if (ret < 0) {
      if (errno == EINTR) {
        continue;
      } else {
        MV_ERROR("poll ret=%d errno=%d", ret, errno);
        break;
      }
    }

    for (int i = 0; i < fd_num; i++) {
      if (fds[i].revents & (POLLIN | POLLERR)) {
        if (i == 0) {
          OnEvent();
        } else if (i == 1) {
          OnAccept();
        } else if (i >= 2) {
          auto conn = GetConnectionByFd(fds[i].fd);
          if (!conn->OnReceive()) {
            RemoveConnection(conn);
          }
        }
      }
    }
  }
}

void VncServer::Schedule(VoidCallback callback) {
  mutex_.lock();
  tasks_.push_back(callback);
  mutex_.unlock();

  uint64_t tmp = 1;
  write(event_fd_, &tmp, sizeof(tmp));
}

void VncServer::OnEvent() {
  uint64_t tmp;
  read(event_fd_, &tmp, sizeof(tmp));
  
  std::list<VoidCallback> tasks_copy;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    tasks_copy.swap(tasks_);
  }
  for (auto& task : tasks_copy) {
    task();
  }
}

void VncServer::OnAccept() {
  int child_fd = accept(server_fd_, nullptr, nullptr);
  if (child_fd < 0) {
    MV_ERROR("child_fd=%d errno=%d", child_fd, errno);
    return;
  }

  auto conn = new VncConnection(this, child_fd);
  connections_.push_back(conn);
}

VncConnection* VncServer::GetConnectionByFd(int fd) {
  for (auto conn : connections_) {
    if (conn->fd() == fd) {
      return conn;
    }
  }
  return nullptr;
}

void VncServer::RemoveConnection(VncConnection* conn) {
  mutex_.lock();
  connections_.remove(conn);
  mutex_.unlock();
  delete conn;
}

void VncServer::SetExclusiveConnnction(VncConnection* conn) {
  for (auto c : connections_) {
    if (c != conn) {
      c->Close();
    }
  }
}
