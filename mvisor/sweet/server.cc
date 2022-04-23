/* 
 * MVisor - Sweet Server
 * Copyright (C) 2022 Terrence <terrence@tenclass.com>
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

#include "sweet/server.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <sys/eventfd.h>
#include <sys/poll.h>

#include "connection.h"
#include "display_encoder.h"
#include "logger.h"


#define POLL_FD_NUM 16


SweetServer::SweetServer(Machine* machine, std::string unix_path) :
  machine_(machine), unix_path_(unix_path) {
  
  /* Event fd is used for thread notification */
  event_fd_ = eventfd(0, 0);
  MV_ASSERT(event_fd_ != -1);

  /* Unix socket listener fd */
  server_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
  MV_ASSERT(server_fd_ != -1);

  /* in case the unix path already exists */
  unlink(unix_path_.c_str());

  struct sockaddr_un un = { .sun_family = AF_UNIX };
  strncpy(un.sun_path, unix_path_.c_str(), sizeof(un.sun_path) - 1);
  if (bind(server_fd_, (sockaddr*)&un, sizeof(un)) < 0) {
    MV_PANIC("failed to bind unix socket %s", unix_path_.c_str());
  }

  MV_ASSERT(listen(server_fd_, 1) == 0);

  /* Find all devices and register listeners */
  LookupDevices();
  /* Set default encoder configuration */
  SetDefaultConfig();
}

SweetServer::~SweetServer() {
  for (auto conn : connections_) {
    delete conn;
  }
  if (display_encoder_) {
    delete display_encoder_;
  }
  if (playback_encoder_) {
    opus_encoder_destroy(playback_encoder_);
    playback_encoder_ = nullptr;
  }

  safe_close(&event_fd_);
  safe_close(&server_fd_);
}

/* Use event_fd to wake up thread from polling */
void SweetServer::WakeUp() {
  if (event_fd_ > 0) {
    uint64_t tmp = 1;
    write(event_fd_, &tmp, sizeof(tmp));
  }
}

/* The main loop is a poll call to handle events / new connections / connection read */
int SweetServer::MainLoop() {
  SetThreadName("sweet-server");

  while (machine_->IsValid() && server_fd_ != -1) {
    /* Setup poll fds (event fd, server fd and connection fds) */
    pollfd fds[POLL_FD_NUM] = {
      {
        .fd = event_fd_,
        .events = POLLIN
      }, {
        .fd = server_fd_,
        .events = POLLIN
      }
    };

    auto fd_num = 2;
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
        MV_LOG("poll ret=%d errno=%d", ret, errno);
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
            /* lock here to prevent io threads from using connection object */
            std::lock_guard<std::mutex> lock(mutex_);
            /* connection is closed or error */
            if (display_connection_ == conn) {
              display_encoder_->Stop();
              display_connection_ = nullptr;
            }
            if (playback_connection_ == conn) {
              playback_connection_ = nullptr;
            }
            if (guest_command_connection_ == conn) {
              guest_command_connection_ = nullptr;
            }
            connections_.remove(conn);
            delete conn;
          }
        }
      }
    }
  }
  return 0;
}

void SweetServer::Close() {
  /* if server_fd_ is valid, accept() will return -1 and error=9 (EBADF) */
  safe_close(&server_fd_);
}

void SweetServer::SetDefaultConfig() {
  display_config_.set_codec("h264");
  display_config_.set_profile("high");
  display_config_.set_preset("superfast");
  display_config_.set_bitrate(4000000);
  display_config_.set_qmin(23);
  display_config_.set_fps(32);
  display_config_.set_threads(1);
  display_config_.set_rate_control("vbr");
  display_config_.set_flags(0b111);
}

void SweetServer::OnEvent() {
  uint64_t tmp;
  read(event_fd_, &tmp, sizeof(tmp));

  if (display_mode_changed_) {
    display_mode_changed_ = false;
    /* Recreate renderer if resolution changed */
    StartDisplayStreamOnConnection(display_connection_, nullptr);
  }

  if (display_updated_) {
    display_updated_ = false;

    DisplayUpdate update;
    display_->AcquireUpdate(update);
    if (display_connection_) {
      display_connection_->UpdateCursor(&update.cursor);
    }
    display_encoder_->Render(update.partials);
    display_->ReleaseUpdate();
  }
}

void SweetServer::OnAccept() {
  struct sockaddr_un un;
  socklen_t len;
  int child_fd = accept(server_fd_, (struct sockaddr *)&un, &len);
  if (child_fd < 0) {
    MV_LOG("child_fd=%d errno=%d", child_fd, errno);
    return;
  }

  auto conn = new SweetConnection(this, child_fd);
  connections_.push_back(conn);
}

SweetConnection* SweetServer::GetConnectionByFd(int fd) {
  for (auto conn : connections_) {
    if (conn->fd() == fd) {
      return conn;
    }
  }
  return nullptr;
}

void SweetServer::LookupDevices() {
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<KeyboardInputInterface*>(o); })) {
    keyboard_ = dynamic_cast<KeyboardInputInterface*>(o);
  }
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<DisplayInterface*>(o); })) {
    display_ = dynamic_cast<DisplayInterface*>(o);
  }
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<PlaybackInterface*>(o); })) {
    playback_ = dynamic_cast<PlaybackInterface*>(o);
  }
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<PointerInputInterface*>(o); })) {
    pointers_.push_back(dynamic_cast<PointerInputInterface*>(o));
  }
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<DisplayResizeInterface*>(o); })) {
    resizers_.push_back(dynamic_cast<DisplayResizeInterface*>(o));
  }
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<SerialPortInterface*>(o); })) {
    auto port = dynamic_cast<SerialPortInterface*>(o);
    if (strcmp(port->port_name(), "org.qemu.guest_agent.0") == 0) {
      qemu_guest_agent_ = port;
      break;
    }
  }
  MV_ASSERT(display_);

  display_->RegisterDisplayModeChangeListener([this]() {
    display_mode_changed_ = true;
    WakeUp();
  });
  display_->RegisterDisplayUpdateListener([this]() {
    display_updated_ = true;
    WakeUp();
  });
  if (playback_) {
    playback_->RegisterPlaybackListener([this](PlaybackState state, struct iovec iov) {
      std::lock_guard<std::mutex> lock(mutex_);
      OnPlayback(state, iov);
    });
  }
  if (qemu_guest_agent_) {
    qemu_guest_agent_->set_callback([this](uint8_t* data, size_t size) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (guest_command_connection_) {
        guest_command_connection_->Send(kQemuGuestCommandResponse, data, size);
      }
    });
  }
}

/* There maybe more than one connections at the same time,
 * but only one display stream connection allowed */
void SweetServer::StartDisplayStreamOnConnection(SweetConnection* conn, SweetProtocol::DisplayStreamConfig* config) {
  if (config) {
    display_config_ = *config;
  }

  uint w, h, bpp, stride;
  display_->GetDisplayMode(&w, &h, &bpp, &stride);

  if (display_encoder_) {
    delete display_encoder_;
  }
  display_encoder_ = new SweetDisplayEncoder(w, h, &display_config_);
  display_->Redraw();

  /* Send stop to previous connection */
  if (display_connection_) {
    display_connection_->SendDisplayStreamStopEvent();
  }

  display_connection_ = conn;
  if (display_connection_) {
    display_connection_->SendDisplayStreamStartEvent(w, h);
    display_encoder_->Start([this](void* data, size_t length) {
      display_connection_->SendDisplayStreamDataEvent(data, length);
    });
  }
}

void SweetServer::StopDisplayStream() {
  display_encoder_->Stop();
  if (display_connection_) {
    display_connection_->SendDisplayStreamStopEvent();
    display_connection_ = nullptr;
  }
}

void SweetServer::RefreshDisplayStream() {
  display_encoder_->ForceKeyframe();
}

void SweetServer::QemuGuestCommand(SweetConnection* conn, std::string& command) {
  if (qemu_guest_agent_) {
    guest_command_connection_ = conn;
    qemu_guest_agent_->SendMessage((uint8_t*)command.data(), command.size());
  }
}

void SweetServer::OnPlayback(PlaybackState state, struct iovec& iov) {
  switch (state)
  {
  case kPlaybackStart:
    playback_->GetPlaybackFormat(&playback_format_.format, &playback_format_.channels,
      &playback_format_.frequency, &playback_format_.interval_ms);
    break;
  case kPlaybackStop:
    if (playback_encoder_) {
      opus_encoder_destroy(playback_encoder_);
      playback_encoder_ = nullptr;
      if (playback_connection_) {
        playback_connection_->SendPlaybackStreamStopEvent();
      }
    }
    break;
  case kPlaybackData:
    if (!playback_encoder_) {
      int error;
      playback_encoder_ = opus_encoder_create(playback_format_.frequency, playback_format_.channels, OPUS_APPLICATION_AUDIO, &error);
      if (!playback_encoder_) {
        MV_PANIC("failed to create opus encoder, error=%d", error);
      }
      if (playback_connection_) {
        playback_connection_->SendPlaybackStreamStartEvent("opus", 0, playback_format_.channels, playback_format_.frequency);
      }
    }
    if (playback_encoder_ && playback_connection_) {
      uint8_t buffer[1000];
      int frame_size = 480;
      size_t frame_bytes = frame_size * playback_format_.channels * sizeof(opus_int16);
      auto remain = iov.iov_len;
      auto ptr = (uint8_t*)iov.iov_base;
      while (remain >= frame_bytes) {
        auto ret = opus_encode(playback_encoder_, (const opus_int16*)ptr, frame_size, buffer, sizeof(buffer));
        if (ret > 0) {
          playback_connection_->SendPlaybackStreamDataEvent(buffer, ret);
        }
        ptr += frame_bytes;
        remain -= frame_bytes;
      }
    }
    break;
  }
}


void SweetServer::StartPlaybackStreamOnConnection(SweetConnection* conn, PlaybackStreamConfig* config) {
  bool is_playing = playback_encoder_ != nullptr;
  /* Send stop to previous connection */
  if (playback_connection_ && is_playing) {
    playback_connection_->SendPlaybackStreamStopEvent();
  }

  playback_connection_ = conn;
  if (playback_connection_ && is_playing) {
    playback_connection_->SendPlaybackStreamStartEvent("opus", 0, playback_format_.channels, playback_format_.frequency);
  }
}

void SweetServer::StopPlaybackStream() {
  bool is_playing = playback_encoder_ != nullptr;
  if (playback_connection_ && is_playing) {
    playback_connection_->SendPlaybackStreamStopEvent();
  }
  playback_connection_ = nullptr;
}

