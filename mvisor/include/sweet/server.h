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


#ifndef _MVISOR_SWEET_SERVER_H
#define _MVISOR_SWEET_SERVER_H

#include <list>
#include <string>
#include <mutex>
#include <opus/opus.h>

#include "machine.h"
#include "device_interface.h"
#include "pb/sweet.pb.h"

using namespace SweetProtocol;

class SweetConnection;
class SweetDisplayEncoder;
class SweetServer {
 public:
  SweetServer(Machine* machine, std::string unix_path);
  ~SweetServer();

  int MainLoop();
  void Close();
  void WakeUp();

  void StartDisplayStreamOnConnection(SweetConnection* conn, DisplayStreamConfig* config);
  void StopDisplayStream();
  void StartPlaybackStreamOnConnection(SweetConnection* conn, PlaybackStreamConfig* config);
  void StopPlaybackStream();
  void StartClipboardStreamOnConnection(SweetConnection* conn);
  void StopClipboardStream();
  void RefreshDisplayStream();
  void QemuGuestCommand(SweetConnection* conn, std::string& command);

  inline Machine* machine() { return machine_; }
  inline std::vector<PointerInputInterface*>& pointers() { return pointers_; }
  inline std::vector<DisplayResizeInterface*>& resizers() { return resizers_; }
  inline KeyboardInputInterface* keyboard() { return keyboard_; }
  inline PlaybackInterface* playback() { return playback_; }
  inline DisplayInterface* display() { return display_; }
  inline SweetDisplayEncoder* display_encoder() { return display_encoder_; }
  inline ClipboardInterface* clipboard() { return clipboard_; }

 private:
  SweetConnection* GetConnectionByFd(int fd);
  void RemoveConnection(SweetConnection* conn);
  void LookupDevices();
  void OnEvent();
  void OnAccept();
  void OnPlayback(PlaybackState state, struct iovec& iov);
  void SetDefaultConfig();
  void OnClipboardEvent(ClipboardData clipboard_data);

  Machine*                    machine_;
  std::list<SweetConnection*> connections_;
  std::mutex                  mutex_;
  std::string                 unix_path_;
  int                         server_fd_ = -1;
  int                         event_fd_ = -1;
  
  DisplayInterface*                     display_ = nullptr;
  PlaybackInterface*                    playback_ = nullptr;
  KeyboardInputInterface*               keyboard_ = nullptr;
  SerialPortInterface*                  qemu_guest_agent_ = nullptr;
  ClipboardInterface*                   clipboard_ = nullptr;
  std::vector<PointerInputInterface*>   pointers_;
  std::vector<DisplayResizeInterface*>  resizers_;

  PlaybackFormat                        playback_format_;

  bool                        display_mode_changed_ = false;
  bool                        display_updated_ = false;
  SweetDisplayEncoder*        display_encoder_ = nullptr;
  SweetConnection*            display_connection_ = nullptr;
  DisplayStreamConfig         display_config_;
  OpusEncoder*                playback_encoder_ = nullptr;
  SweetConnection*            playback_connection_ = nullptr;
  SweetConnection*            clipboard_connection_ = nullptr;
  SweetConnection*            guest_command_connection_ = nullptr;
};

#endif // _MVISOR_SWEET_SERVER_H
