/* 
 * MVisor
 * Copyright (C) 2022 cair <rui.cai@tenclass.com>
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

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include "logger.h"
#include "machine.h"
#include "migration.h"


MigrationNetworkWriter::MigrationNetworkWriter() {
  
}

MigrationNetworkWriter::~MigrationNetworkWriter() {
  safe_close(&socket_fd_);
}

int MigrationNetworkWriter::BeginWrite(std::string tag) {
  MV_UNUSED(tag);
  return 0;
}

void MigrationNetworkWriter::SetPrefix(std::string prefix) {
  MV_UNUSED(prefix);
}

void MigrationNetworkWriter::EndWrite(std::string tag) {
  MV_UNUSED(tag);
}

bool MigrationNetworkWriter::Connect(std::string ip, uint16_t port) {
  in_addr ip_address;
  if (!inet_aton(ip.c_str(), &ip_address)) {
    MV_ERROR("invalid ip=%s", ip.c_str());
    return false;
  }

  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ == -1) {
    MV_ERROR("create socket error");
    return false;
  }

  sockaddr_in dest_addr = {
    .sin_family = AF_INET,
    .sin_port = htons(port),
    .sin_addr = {
      .s_addr = htonl(ntohl(ip_address.s_addr))
    },
    .sin_zero = {0}
  };
  if (connect(socket_fd_, (sockaddr*)&dest_addr, sizeof(dest_addr)) != 0) {
    MV_ERROR("failed to connect target ip=%s", ip.c_str());
    return false;
  }
  return true;
}

bool MigrationNetworkWriter::Write(void* data, size_t size) {
  auto pos = (uint8_t*)data;
  auto remain_size = size;
  while (remain_size > 0) {
    auto ret = send(socket_fd_, pos, remain_size, 0);
    if (ret <= 0) {
      MV_ERROR("socket error happened");
      return false;
    }
    pos += ret;
    remain_size -= ret;
  }
  return true;
}

bool MigrationNetworkWriter::WriteRaw(std::string tag, void* data, size_t size) {
  // make migration data header
  MigrationNetworkDataHeader header;
  MV_ASSERT(tag.length() < sizeof(header.tag));
  strcpy(header.tag, tag.c_str());
  header.size = size;

  // send data header
  bool ret = Write(&header, sizeof(header));
  if (ret && data && size) {
    // send data body
    ret = Write(data, size);
  }
  return ret;
}

bool MigrationNetworkWriter::WriteProtobuf(std::string tag, const Message& message) {
  auto size = message.ByteSizeLong();
  auto data = new uint8_t[size];
  auto ret = message.SerializePartialToArray(data, size);
  if (ret) {
    ret = WriteRaw(tag, data, size);
  }
  delete[] data;
  return ret;
}

bool MigrationNetworkWriter::WriteMemoryPages(std::string tag, void* pages, size_t size) {
  bool ret = false;
  auto pos = (uint8_t*)pages;
  auto remain_size = size;
  while (remain_size > 0) {
    if (test_zero(pos, PAGE_SIZE)) {
      ret = WriteRaw(tag, nullptr, 0);
    } else {
      ret = WriteRaw(tag, pos, PAGE_SIZE);
    }

    if (!ret) {
      break;
    }
    pos += PAGE_SIZE;
    remain_size -= PAGE_SIZE;
  }
  return ret;
}

bool MigrationNetworkWriter::WaitForSignal(MigrationSignalType type) {
  MigrationSignal signal;
  auto size = sizeof(signal);
  auto pos = (uint8_t*)&signal;
  while (size > 0) {
    auto ret = recv(socket_fd_, pos, size, 0);
    if (ret <= 0) {
      MV_ERROR("socket error happened");
      return false;
    }
    size -= ret;
    pos += ret;
  }
  return type == signal.type;
}

bool MigrationNetworkWriter::WriteFromFile(std::string tag, std::string path, size_t offset) {
  auto fd = open(path.c_str(), O_RDONLY);
  if (fd <= 0) {
    return false;
  }

  struct stat st;
  MV_ASSERT(fstat(fd, &st) != -1);
  MV_ASSERT(offset < (size_t)st.st_size);

  auto size = st.st_size - offset;
  auto file_ptr = mmap(nullptr, size, PROT_READ, MAP_SHARED, fd, offset);
  MV_ASSERT(file_ptr != MAP_FAILED);

  auto ret = WriteRaw(tag, file_ptr, size);
  munmap(file_ptr, size);
  safe_close(&fd);
  return ret;
}
