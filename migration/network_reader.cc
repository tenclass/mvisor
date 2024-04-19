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


MigrationNetworkReader::MigrationNetworkReader() {

}

MigrationNetworkReader::~MigrationNetworkReader() {
  MV_ASSERT(cache_map_.empty());
  safe_close(&socket_fd_);
}

bool MigrationNetworkReader::WaitForConnection(uint16_t port) {
  auto server_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  MV_ASSERT(server_socket != -1);

  int flag = 1;
  MV_ASSERT(setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag)) == 0);

  sockaddr_in server_addr = {
    .sin_family = AF_INET,
    .sin_port = htons(port),
    .sin_addr = {
      .s_addr = htonl(INADDR_ANY)
    },
    .sin_zero = {0}
  };
  MV_ASSERT(bind(server_socket, (sockaddr*)&server_addr, sizeof(server_addr)) == 0);
  MV_ASSERT(listen(server_socket, 1) == 0);

  sockaddr_in client_addr;
  socklen_t addr_len = sizeof(client_addr);

  // wait for connection to source vm
  socket_fd_ = accept(server_socket, (sockaddr*)&client_addr, &addr_len);
  MV_ASSERT(socket_fd_ != -1);

  safe_close(&server_socket);
  return true;
}

int MigrationNetworkReader::BeginRead(std::string tag) {
  MV_UNUSED(tag);
  return 0;
}

void MigrationNetworkReader::SetPrefix(std::string prefix) {
  MV_UNUSED(prefix);
}

void MigrationNetworkReader::EndRead(std::string tag) {
  MV_UNUSED(tag);
}

void MigrationNetworkReader::Read(void* data, size_t size) {
  MV_ASSERT(socket_fd_ != -1);
  auto pos = (uint8_t*)data;
  auto remain_size = size;
  while (remain_size > 0) {
    auto ret = recv(socket_fd_, pos, remain_size, 0);
    if (ret <= 0) {
      MV_PANIC("socket error happened");
      break;
    }
    pos += ret;
    remain_size -= ret;
  }
  MV_ASSERT(remain_size == 0);
}

void MigrationNetworkReader::FreeCacheData(MigrationNetworkData* data) {
  MV_ASSERT(data != nullptr);
  if (data->header.size) {
    MV_ASSERT(data->body != nullptr);
    delete data->body;
  }
  delete data;
}

MigrationNetworkData* MigrationNetworkReader::ReadFromCache(std::string tag)  {
  MigrationNetworkData* data = nullptr;
  auto iter = cache_map_.find(tag);
  if (iter != cache_map_.end()) {
    auto& cache_queue = iter->second;
    MV_ASSERT(!cache_queue.empty());

    // get cache data
    data = cache_queue.front();
    cache_queue.pop();

    if (cache_queue.empty()) {
      cache_map_.erase(tag);
    }
  }
  return data;
}

MigrationNetworkDataHeader MigrationNetworkReader::WaitForDataHeader(std::string tag) {
  MV_ASSERT(socket_fd_ != -1);
  MigrationNetworkDataHeader header;
  while (true) {
    Read(&header, sizeof(header));
    if (0 == strcmp(tag.c_str(), header.tag)) {
      // find the right data header
      break;
    } else {
      // cache header+body when tag was mismatched
      MigrationNetworkData* data = new MigrationNetworkData;
      data->header = header;
      if (header.size != 0) {
        auto body = new uint8_t[header.size];
        Read(body, header.size);
        data->body = body;
      }
      cache_map_[data->header.tag].push(data);
    }
  }
  return header;
}

size_t MigrationNetworkReader::ReadRawWithLimit(std::string tag, void* data, size_t limit) {
  MV_ASSERT(limit > 0);
  size_t size = 0;
  auto cached_data = ReadFromCache(tag);
  if (cached_data) {
    size = std::min(cached_data->header.size, limit);
    if (size) {
      memcpy(data, cached_data->body, size);
    }
    FreeCacheData(cached_data);
  } else {
    size = std::min(WaitForDataHeader(tag).size, limit);
    if (size) {
      Read(data, size);
    }
  }
  return size;
}

bool MigrationNetworkReader::ReadRaw(std::string tag, void* data, size_t size) {
  auto cached_data = ReadFromCache(tag);
  if (cached_data) {
    MV_ASSERT(cached_data->header.size == size);
    if (size) {
      memcpy(data, cached_data->body, size);
    }
    FreeCacheData(cached_data);
  } else {
    MV_ASSERT(size == WaitForDataHeader(tag).size);
    if (size) {
      Read(data, size);
    }
  }
  return true;
}

bool MigrationNetworkReader::ReadProtobuf(std::string tag, Message& message) {
  auto cached_data = ReadFromCache(tag);
  if (cached_data) {
    message.ParsePartialFromArray(cached_data->body, cached_data->header.size);
    FreeCacheData(cached_data);
  } else {
    size_t size = WaitForDataHeader(tag).size;
    auto data = new uint8_t[size];
    Read(data, size);
    message.ParsePartialFromArray(data, size);
    delete[] data;
  }
  return true;
}

bool MigrationNetworkReader::ReadMemoryPages(std::string tag, void** pages_ptr, size_t size) {
  *pages_ptr = mmap(*pages_ptr, size, PROT_READ | PROT_WRITE,
    MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE | MAP_FIXED, -1, 0);
  MV_ASSERT(*pages_ptr != MAP_FAILED);

  auto remain_size = size;
  auto ptr = (uint8_t*)*pages_ptr;

  auto cached_data = ReadFromCache(tag);
  if (cached_data) {
    while (true) {
      // handle the cached data first
      if (cached_data->header.size) {
        memcpy(ptr, cached_data->body, PAGE_SIZE);
      }
      FreeCacheData(cached_data);

      // next page
      ptr += PAGE_SIZE;
      remain_size -= PAGE_SIZE;
      if (remain_size == 0) {
        // jump out
        break;
      }

      // get next cached data in list
      cached_data = ReadFromCache(tag);

      // make sure the next cached_data exists
      MV_ASSERT(cached_data != nullptr);
    }
  } else {
    while (remain_size > 0) {
      MigrationNetworkDataHeader header = WaitForDataHeader(tag);
      if (header.size) {
        Read(ptr, PAGE_SIZE);
      }

      // next page
      ptr += PAGE_SIZE;
      remain_size -= PAGE_SIZE;
    }
  }
  MV_ASSERT(remain_size == 0);

  /* Make memory pages DONTDUMP */
  MV_ASSERT(madvise(*pages_ptr, size, MADV_MERGEABLE) == 0);
  MV_ASSERT(madvise(*pages_ptr, size, MADV_DONTDUMP) == 0);
  return true;
}

void MigrationNetworkReader::SendSignal(MigrationSignalType type) {
  MV_ASSERT(socket_fd_ != -1);
  MigrationSignal signal = {
    .type = type
  };
  auto size = sizeof(signal);
  auto pos = (uint8_t*)&signal;
  while (size > 0) {
    auto ret = send(socket_fd_, pos, size, 0);
    MV_ASSERT(ret > 0);
    size -= ret;
    pos += ret;
  }
}

void MigrationNetworkReader::ReadToFile(std::string tag, std::string path, size_t offset) {
  auto fd = open(path.c_str(), O_RDWR);
  MV_ASSERT(fd > 0);

  auto cached_data = ReadFromCache(tag);
  if (cached_data) { 
    auto pos = cached_data->body;
    auto remain_size = cached_data->header.size;
    while (remain_size > 0) {
      auto ret = pwrite(fd, pos, remain_size, offset);
      MV_ASSERT(ret > 0);

      remain_size -= ret;
      offset += ret;
      pos += ret;
    }
    MV_ASSERT(remain_size == 0);
    FreeCacheData(cached_data);
  } else {
    auto header = WaitForDataHeader(tag);
    MV_ASSERT(ftruncate(fd, offset + header.size) == 0);

    auto file_ptr = mmap(nullptr, header.size, PROT_WRITE, MAP_SHARED, fd, offset);
    MV_ASSERT(file_ptr != MAP_FAILED);

    // write to file directly from socket buffer
    Read(file_ptr, header.size);
    munmap(file_ptr, header.size);
  }

  safe_close(&fd);
}

bool MigrationNetworkReader::Exists(std::string tag) {
  MV_UNUSED(tag);
  return true;
}
