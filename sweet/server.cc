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

  MV_ASSERT(listen(server_fd_, 10) == 0);

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
  if (record_decoder_) {
    opus_decoder_destroy(record_decoder_);
    record_decoder_ = nullptr;
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
            RemoveConnection(conn);
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

void SweetServer::Schedule(VoidCallback callback) {
  mutex_.lock();
  tasks_.emplace_back(std::move(callback));
  mutex_.unlock();
  WakeUp();
}

void SweetServer::OnEvent() {
  uint64_t tmp;
  read(event_fd_, &tmp, sizeof(tmp));

  std::unique_lock<std::mutex> lock(mutex_);
  while (!tasks_.empty()) {
    auto& task = tasks_.front();
    lock.unlock();
    task();
    lock.lock();
    tasks_.pop_front();
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

void SweetServer::RemoveConnection(SweetConnection* conn) {
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
  if (audio_record_connection_ == conn) {
    audio_record_connection_ = nullptr;
  }
  if (clipboard_connection_ == conn) {
    clipboard_connection_ = nullptr;
  }
  if (guest_command_connection_ == conn) {
    guest_command_connection_ = nullptr;
  }
  if (virtio_fs_connection_ == conn) {
    virtio_fs_connection_ = nullptr;
  }
  connections_.remove(conn);
  delete conn;
}

void SweetServer::LookupDevices() {
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<WacomInputInterface*>(o); })) {
    wacom_ = dynamic_cast<WacomInputInterface*>(o);
  }
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<KeyboardInputInterface*>(o); })) {
    keyboard_ = dynamic_cast<KeyboardInputInterface*>(o);
  }
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<MidiInputInterface*>(o); })) {
    midi_ = dynamic_cast<MidiInputInterface*>(o);
  }
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<VirtioFsInterface*>(o); })) {
    virtio_fs_ = dynamic_cast<VirtioFsInterface*>(o);
  }
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<DisplayInterface*>(o); })) {
    display_ = dynamic_cast<DisplayInterface*>(o);
  }
  for (auto o : machine_->LookupObjects([](auto o) { return dynamic_cast<PlaybackInterface*>(o); })) {
    playback_ = dynamic_cast<PlaybackInterface*>(o);
  }
  for (auto o : machine_->LookupObjects([](auto o){ return dynamic_cast<RecordInterface*>(o); })) {
    audio_record_ = dynamic_cast<RecordInterface*>(o);
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
    } else if (strcmp(port->port_name(), "com.redhat.spice.0") == 0) {
      clipboard_ = dynamic_cast<ClipboardInterface*>(o);
    }
    port->set_callback([this, port](SerialPortEvent event, uint8_t* data, size_t size) {
      Schedule([this, port, event, data = std::string((const char*)data, size)] () {
        switch (event)
        {
        case kSerialPortStatusChanged:
          for (auto conn : connections_) {
            conn->SendSerialPortStatusEvent(port->port_name(), port->ready());
          }
          break;
        case kSerialPortData:
          if (port == qemu_guest_agent_ && guest_command_connection_) {
            guest_command_connection_->Send(kQemuGuestCommandResponse, data);
          }
        }
      });
    });
  }
  MV_ASSERT(display_);

  display_->RegisterDisplayModeChangeListener([this]() {
    Schedule([this] () {
      /* Recreate renderer if resolution changed */
      StartDisplayStreamOnConnection(display_connection_, nullptr);
    });
  });
  display_->RegisterDisplayUpdateListener([this]() {
    Schedule([this] () {
      UpdateDisplay();
    });
  });
  if (playback_) {
    playback_->RegisterPlaybackListener([this](PlaybackState state, struct iovec iov) {
      Schedule([this, state, data = std::string((const char*)iov.iov_base, iov.iov_len)] () {
        OnPlayback(state, data);
      });
    });
  }
  if(audio_record_) {
    audio_record_->RegisterRecordListener([this](RecordState state){
      Schedule([this, state] () {
        OnRecordStats(state);
      });
    });
  }
  if(clipboard_) {
    clipboard_->RegisterClipboardListener([this](const ClipboardData clipboard_data) {
      /* std::move don't copy data, but replace data reference */
      Schedule([this, clipboard_data = std::move(clipboard_data)] () {
        if(clipboard_connection_) {
          clipboard_connection_->SendClipboardStreamDataEvent(clipboard_data);
        }
      });
    });
  }
  if (virtio_fs_) {
    virtio_fs_->RegisterVirtioFsListener([this]() {
      Schedule([this] () {
        if (virtio_fs_connection_) {
          virtio_fs_connection_->Send(kVirtioFsNotifyEvent);
        }
      });
    });
  }
}

void SweetServer::UpdateDisplay() {
  DisplayUpdate update;
  display_->AcquireUpdate(update);
  if (display_connection_) {
    display_connection_->UpdateCursor(&update.cursor);
  }
  display_encoder_->Render(update.partials);
  display_->ReleaseUpdate();
}

/* There maybe more than one connections at the same time,
 * but only one display stream connection allowed */
void SweetServer::StartDisplayStreamOnConnection(SweetConnection* conn, SweetProtocol::DisplayStreamConfig* config) {
  /* Send stop to previous connection */
  if (display_connection_) {
    display_connection_->Send(kDisplayStreamStopEvent);
  }

  if (config) {
    display_config_ = *config;
  }

  uint w, h, bpp, stride;
  display_->GetDisplayMode(&w, &h, &bpp, &stride);

  if (display_encoder_) {
    delete display_encoder_;
  }
  display_encoder_ = new SweetDisplayEncoder(w, h, &display_config_);

  /* Regenerate all partials after encoder recreated */
  display_->Redraw();
  UpdateDisplay();

  display_connection_ = conn;
  if (display_connection_) {
    display_connection_->SendDisplayStreamStartEvent(w, h);
    display_encoder_->Start([this](void* data, size_t length) {
      /* Use sweet server thread to send data */
      Schedule([this, data = std::string((const char*)data, length)]() {
        if (display_connection_) {
          display_connection_->Send(kDisplayStreamDataEvent, data);
        }
      });
    });
  }
}

void SweetServer::StopDisplayStream() {
  display_encoder_->Stop();
  if (display_connection_) {
    display_connection_->Send(kDisplayStreamStopEvent);
    display_connection_ = nullptr;
  }
}

void SweetServer::RefreshDisplayStream() {
  display_encoder_->ForceKeyframe();
}

void SweetServer::QemuGuestCommand(SweetConnection* conn, std::string& command) {
  if (qemu_guest_agent_) {
    if (qemu_guest_agent_->ready()) {
      guest_command_connection_ = conn;
      qemu_guest_agent_->SendMessage((uint8_t*)command.data(), command.size());
    } else {
      /* FIXME: how to send error response??? */
      conn->Send(kQemuGuestCommandResponse);
    }
  }
}

void SweetServer::OnPlayback(PlaybackState state, const std::string& data) {
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
        playback_connection_->Send(kPlaybackStreamStopEvent);
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
      auto remain = data.size();
      auto ptr = (uint8_t*)data.data();
      while (remain >= frame_bytes) {
        auto ret = opus_encode(playback_encoder_, (const opus_int16*)ptr, frame_size, buffer, sizeof(buffer));
        if (ret > 0) {
          playback_connection_->Send(kPlaybackStreamDataEvent, buffer, ret);
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
    playback_connection_->Send(kPlaybackStreamStopEvent);
  }

  playback_connection_ = conn;
  if (playback_connection_ && is_playing) {
    playback_connection_->SendPlaybackStreamStartEvent("opus", 0, playback_format_.channels, playback_format_.frequency);
  }
}

void SweetServer::StopPlaybackStream() {
  bool is_playing = playback_encoder_ != nullptr;
  if (playback_connection_ && is_playing) {
    playback_connection_->Send(kPlaybackStreamStopEvent);
  }
  playback_connection_ = nullptr;
}

void SweetServer::StartClipboardStreamOnConnection(SweetConnection* conn) {
  /* Send stop to previous connection */
  if (clipboard_connection_ ) {
    clipboard_connection_->Send(kClipboardStreamStopEvent);
  }

  clipboard_connection_ = conn;

  if (clipboard_connection_) {
    clipboard_connection_->Send(kClipboardStreamStartEvent);
  }
}

void SweetServer::StopClipboardStream() {
  if (clipboard_connection_) {
    clipboard_connection_->Send(kClipboardStreamStopEvent);
  }
  clipboard_connection_ = nullptr;
}

void SweetServer::StartVirtioFsConnection(SweetConnection* conn) {
  virtio_fs_connection_ = conn;
  if(virtio_fs_connection_) {
    virtio_fs_connection_->Send(kVirtioFsStartEvent);
  }
}

void SweetServer::StopVirtioFsConnection() {
  if(virtio_fs_connection_) {
    virtio_fs_connection_->Send(kVirtioFsStopEvent);
  }
  virtio_fs_connection_ = nullptr;
}

void SweetServer::StartMidiConnection(SweetConnection* conn) {
  midi_connection_ = conn;
  if(midi_connection_) {
    midi_connection_->Send(kMidiStartEvent);
  }
}

void SweetServer::StopMidiConnection() {
  if(midi_connection_) {
    midi_connection_->Send(kMidiStopEvent);
  }
  midi_connection_ = nullptr;
}
void SweetServer::StartWacomConnection(SweetConnection* conn) {
  wacom_connection_ = conn;
  if(wacom_connection_) {
    wacom_connection_->Send(kWacomStartEvent);
  }
}

void SweetServer::StopWacomConnection() {
  if(wacom_connection_) {
    wacom_connection_->Send(kWacomStopEvent);
  }
  wacom_connection_ = nullptr;
}

void SweetServer::StartRecordStreamOnConnection(SweetConnection* conn, RecordStreamConfig* config) {
  record_format_.frequency = config->frequency();
  record_format_.channels = config->channels();
  audio_record_connection_ = conn;
  if (recording_) {
    audio_record_connection_->Send(kAudioRecordStartEvent);
  }
}

void SweetServer::StopRecordStream() {
  if (recording_) {
    audio_record_connection_->Send(kAudioRecoreStopEvent);
  }
  audio_record_connection_ = nullptr;
}

void SweetServer::OnRecordStats(RecordState state) {
  if(state == kRecordStart) {
    if(!record_decoder_) {
      int error;
      record_decoder_ = opus_decoder_create(record_format_.frequency, record_format_.channels, &error);
      if (!record_decoder_ || error != OPUS_OK) {
          MV_PANIC("failed to create opus decoder, error=%d", error);
          return;
      }
      recording_ = true;
    }

    if (audio_record_connection_) {
      audio_record_connection_->Send(kAudioRecordStartEvent);
    }
  } else if(state == kRecordStop) {
    if(record_decoder_) {
      opus_decoder_destroy(record_decoder_);
      record_decoder_ = nullptr;
    }

    recording_ = false;
    if (audio_record_connection_) {
      audio_record_connection_->Send(kAudioRecoreStopEvent);
    }
  } else {
    MV_LOG("unknown record event");
  }
}

void SweetServer::SendRecordStreamData(const std::string& data) {
  if(record_decoder_ && audio_record_connection_) {
    opus_int16 out[3840] = {0};
    int samples = opus_decode(record_decoder_, (const unsigned char *)data.data(),  data.size(), out, 1920, 0);
    if (samples < OPUS_OK) {
      MV_ERROR("opus decode failed:%s", opus_strerror(samples));
      return;
    }

    size_t pcm_length = samples * record_format_.channels * sizeof(opus_int16);
    audio_record_->WriteRecordDataToDevice(std::string((const char*)out, pcm_length));
  }
}
