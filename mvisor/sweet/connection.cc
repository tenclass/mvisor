/* 
 * MVisor - Sweet Connection
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

#include "connection.h"
#include "display_encoder.h"
#include "utilities.h"
#include "logger.h"
#include "pb/sweet.pb.h"


SweetConnection::SweetConnection(SweetServer* server, int fd) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  server_ = server;
  machine_ = server->machine();
  fd_ = fd;
}

SweetConnection::~SweetConnection() {
  safe_close(&fd_);
}

bool SweetConnection::OnReceive() {
  SweetPacketHeader header;
  int nbytes = recv(fd_, &header, sizeof(header), MSG_WAITALL);
  if (nbytes <= 0) {
    return false;
  }
  
  ParsePacket(&header);
  return true;
}

void SweetConnection::ParsePacket(SweetPacketHeader* header) {
  if (header->length) {
    buffer_.resize(header->length);
    int ret = recv(fd_, buffer_.data(), header->length, MSG_WAITALL);
    if (ret != (int)header->length) {
      MV_LOG("failed to recv %u bytes, ret=%d", header->length, ret);
      return;
    }
  }

  switch (header->type)
  {
  case kQueryStatus:
    OnQueryStatus();
    break;
  case kPauseMachine:
    machine_->Pause();
    break;
  case kResumeMachine:
    machine_->Resume();
    break;
  case kResetMachine:
    machine_->Reset();
    break;
  case kShutdownMachine:
    machine_->Shutdown();
    break;
  case kQuitMachine:
    machine_->Quit();
    break;
  case kSaveMachine:
    OnSaveMachine();
    break;
  case kQemuGuestCommand:
    server_->QemuGuestCommand(this, buffer_);
    break;
  case kStartDisplayStream:
    OnStartDisplayStream();
    break;
  case kStopDisplayStream:
    server_->StopDisplayStream();
    break;
  case kRefreshDisplayStream:
    server_->RefreshDisplayStream();
    break;
  case kSendKeyboardInput:
    OnKeyboardInput();
    break;
  case kSendPointerInput:
    OnSendPointerInput();
    break;
  case kConfigMonitors:
    OnConfigMonitors();
    break;
  case kStartPlaybackStream:
    OnStartPlaybackStream();
    break;
  case kStopPlaybackStream:
    server_->StopPlaybackStream();
    break;
  case kQueryScreenshot:
    OnQueryScreenshot();
    break;
  case kClipboardDataToGuest:
    OnClipboardDataToGuest();
    break;
  case kStartClipboardStream:
    OnStartClipboardStream();
    break;
  case kStopClipboardStream:
    OnStopClipboardStream();
  case kStartVirtioFs:
    OnStartVirtioFs();
    break;
  case kStopVirtioFs:
    OnStopVirtioFs();
    break;
  default:
    MV_LOG("unhandled sweet type=0x%x", header->type);
    return;
  }

  /* command acknowledge */
  Send(kCommandAcknowledge);
}

bool SweetConnection::Send(uint32_t type, void* data, size_t length) {
  /* Send() is called by encoder thread to send encoded frames */
  std::lock_guard<std::mutex> lock(mutex_);

  SweetPacketHeader header = {
    .type = type,
    .length = (uint32_t)length
  };
  int ret = send(fd_, &header, sizeof(header), 0);
  if (ret != sizeof(header)) {
    MV_LOG("failed to send message, ret=%d", ret);
    return false;
  }

  if (length > 0) {
    ret = send(fd_, data, length, 0);
    if (ret != (int)length) {
      MV_LOG("failed to send message, ret=%d", ret);
      return false;
    }
  }
  return true;
}

bool SweetConnection::Send(uint32_t type, Message& message) {
  /* Reuse the buffer */
  if (!message.SerializeToString(&buffer_)) {
    MV_LOG("failed to serialize message type=0x%x", type);
    return false;
  }
  return Send(type, buffer_.data(), buffer_.size());
}

bool SweetConnection::Send(uint32_t type) {
  return Send(type, nullptr, 0);
}

void SweetConnection::OnQueryStatus() {
  QueryStatusResponse response;  
  response.set_debug(machine_->debug());
  response.set_status(machine_->GetStatus());
  response.set_hypervisor(machine_->hypervisor());
  response.set_vcpu_count(machine_->num_vcpus());
  response.set_memory_size(machine_->ram_size());
  response.set_vm_uuid(machine_->vm_uuid());
  response.set_vm_name(machine_->vm_name());
  response.set_config_path(machine_->configuration()->path());

  auto spice_agent = dynamic_cast<SerialPortInterface*>(machine_->LookupObjectByClass("SpiceAgent"));
  if (spice_agent) {
    response.set_spice_agent(true);
  }
  auto qemu_agent = dynamic_cast<SerialPortInterface*>(machine_->LookupObjectByClass("QemuGuestAgent"));
  if (qemu_agent) {
    response.set_qemu_agent(true);
  }

  Send(kQueryStatusResponse, response);
}

void SweetConnection::OnSaveMachine() {
  SaveMachineOptions options;
  if (!options.ParseFromString(buffer_)) {
    MV_PANIC("failed to parse buffer");
  }
  machine_->Save(options.path());
}

void SweetConnection::OnKeyboardInput() {
  SendKeyboardInput input;
  if (!input.ParseFromString(buffer_)) {
    MV_PANIC("failed to parse buffer");
  }
  if (machine_->IsPaused()) {
    return;
  }

  uint code = input.scancode();
  uint8_t scancode[10] = { 0 };
  for (int i = 0; i < 10 && code; i++) {
    scancode[i] = (uint8_t)code;
    code >>= 8;
  }
  server_->keyboard()->QueueKeyboardEvent(scancode, input.modifiers());
}

void SweetConnection::OnSendPointerInput() {
  SendPointerInput input;
  if (!input.ParseFromString(buffer_)) {
    MV_PANIC("failed to parse buffer");
  }
  if (machine_->IsPaused()) {
    return;
  }

  PointerEvent event;
  event.buttons = input.buttons();
  event.x = input.x();
  event.y = input.y();
  event.z = input.z();

  uint w, h;
  server_->display()->GetDisplayMode(&w, &h, nullptr, nullptr);
  event.screen_width = w;
  event.screen_height = h;
  
  for (auto pointer : server_->pointers()) {
    if (pointer->InputAcceptable()) {
      pointer->QueuePointerEvent(event);
      break;
    }
  }
}

void SweetConnection::OnConfigMonitors() {
  MonitorsConfig config;
  if (!config.ParseFromString(buffer_)) {
    MV_PANIC("failed to parse buffer");
  }
  
  MV_ASSERT(config.count() > 0);
  auto& monitor = config.monitors(0);

  for (auto resizer : server_->resizers()) {
    if (resizer->Resize(monitor.width(), monitor.height())) {
      break;
    }
  }
}

void SweetConnection::OnStartDisplayStream() {
  DisplayStreamConfig config;
  if (!config.ParseFromString(buffer_)) {
    MV_PANIC("failed to parse buffer");
  }
  server_->StartDisplayStreamOnConnection(this, &config);
}

void SweetConnection::OnStartPlaybackStream() {
  PlaybackStreamConfig config;
  if (!config.ParseFromString(buffer_)) {
    MV_PANIC("failed to parse buffer");
  }
  server_->StartPlaybackStreamOnConnection(this, &config);
}

void SweetConnection::SendDisplayStreamStartEvent(uint w, uint h) {
  DisplayStreamStartEvent event;
  event.set_width(w);
  event.set_height(h);
  Send(kDisplayStreamStartEvent, event);
}

void SweetConnection::SendDisplayStreamStopEvent() {
  Send(kDisplayStreamStopEvent);
}

void SweetConnection::SendDisplayStreamDataEvent(void* data, size_t length) {
  Send(kDisplayStreamDataEvent, data, length);
}

void SweetConnection::UpdateCursor(const DisplayMouseCursor* cursor_update) {
  if (cursor_update->visible) {
    if (cursor_update->shape.id == cursor_shape_id_) {
      return;
    }
    auto& shape = cursor_update->shape;

    cursor_shape_id_ = shape.id;
    cursor_visible_ = true;

    SetCursorEvent event;
    event.set_visible(true);
    event.set_x(cursor_update->x);
    event.set_y(cursor_update->y);
    auto cursor = event.mutable_shape();
    cursor->set_type(shape.type);
    cursor->set_width(shape.width);
    cursor->set_height(shape.height);
    cursor->set_hotspot_x(shape.hotspot_x);
    cursor->set_hotspot_y(shape.hotspot_y);

    cursor->set_data(shape.data.data(), shape.data.size());
    Send(kSetCursorEvent, event);
  } else {
    if (cursor_visible_) {
      cursor_visible_ = false;
  
      SetCursorEvent event;
      event.set_visible(false);
      Send(kSetCursorEvent, event);
    }
  }
}

void SweetConnection::SendPlaybackStreamStartEvent(std::string codec, uint format, uint channels, uint frequency) {
  PlaybackStreamStartEvent event;
  event.set_codec(codec);
  event.set_format(format);
  event.set_channels(channels);
  event.set_frequency(frequency);
  Send(kPlaybackStreamStartEvent, event);
}

void SweetConnection::SendPlaybackStreamStopEvent() {
  Send(kPlaybackStreamStopEvent);
}

void SweetConnection::SendPlaybackStreamDataEvent(void* data, size_t length) {
  Send(kPlaybackStreamDataEvent, data, length);
}

void SweetConnection::SendClipboardStreamStartEvent() {
  Send(kClipboardStreamStartEvent);
}

void SweetConnection::SendClipboardStreamStopEvent() {
  Send(kClipboardStreamStopEvent);
}

void SweetConnection::SendClipboardStreamDataEvent(ClipboardData& clipboard_data) {
  ClipboardStreamDataEvent  event;
  event.set_type(SweetProtocol::ClipboardDataType(clipboard_data.type));
  event.set_data(clipboard_data.msg_data, clipboard_data.msg_size);
  Send(kClipboardStreamDataEvent, event);
}

void SweetConnection::OnQueryScreenshot() {
  QueryScreeenshot query;
  if (!query.ParseFromString(buffer_)) {
    MV_PANIC("failed to parse buffer");
  }
  std::string image_data;

  auto encoder = server_->display_encoder();
  if (encoder) {
    encoder->Screendump(query.format(), query.width(), query.height(), 80, image_data);
  }

  QueryScreenshotResponse response;
  response.set_format(query.format());
  response.set_width(query.width());
  response.set_height(query.height());
  response.set_data(image_data);
  Send(kQueryScreenshotResponse, response);
}

void SweetConnection::OnClipboardDataToGuest() {
  ClipboardDataToGuest clipboard;
  if(!clipboard.ParseFromString(buffer_)) {
    MV_PANIC("failed to parse buffer");
  }

  switch(clipboard.type()) {
    case kSweetClipboard_NONE: {
      MV_LOG("sweet clipboard type none");
      break;
    }
    case kSweetClipboard_UTF8_TEXT: {
      auto clipboard_interface = server_->clipboard();
      if(clipboard_interface)
        clipboard_interface->ClipboardDataToGuest(clipboard.type(), clipboard.data().size(), (void*)clipboard.data().data());
      break;
    }
    case kSweetClipboard_IMAGE_PNG:
      break;
    case kSweetClipboard_IMAGE_BMP:
      break;
    case kSweetClipboard_IMAGE_TIFF:
      break;
    case kSweetClipboard_IMAGE_JPG:
      break;
    case kSweetClipboard_FILE_LIST:
      break;
    default:
    break;
  }
}

void SweetConnection::OnStartClipboardStream() {
  if(server_) {
    server_->StartClipboardStreamOnConnection(this);
  }
}

void SweetConnection::OnStopClipboardStream() {
  if(server_) {
    server_->StopClipboardStream();
  }
}
void SweetConnection::OnStartVirtioFs(){
  server_->StartVirtioFsConnection(this);
}

void SweetConnection::OnStopVirtioFs() {
  server_->StopVirtioFsConnection();
}
