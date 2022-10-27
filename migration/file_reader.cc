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

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <filesystem>

#include "logger.h"
#include "machine.h"
#include "migration.h"


MigrationFileReader::MigrationFileReader(std::string base_path) {
  base_path_ = base_path;
}

MigrationFileReader::~MigrationFileReader() {

}

void MigrationFileReader::SetPrefix(std::string prefix) {
  prefix_ = prefix;
}

bool MigrationFileReader::ReadRaw(std::string tag, void* data, size_t size) {
  BeginRead(tag);
  auto ptr = (uint8_t*)data;
  while (size > 0) {
    auto ret = read(fd_, ptr, size);
    if (ret <= 0) {
      MV_PANIC("failed to read fd=%d ret=%ld", fd_, ret);
    }
    ptr += ret;
    size -= ret;
  }
  EndRead(tag);
  return true;
}

bool MigrationFileReader::ReadProtobuf(std::string tag, Message& message) {
  BeginRead(tag);
  message.Clear();
  bool ret = message.ParseFromFileDescriptor(fd_);
  EndRead(tag);
  return ret;
}

bool MigrationFileReader::ReadMemoryPages(std::string tag, void** pages_ptr, size_t size) {
  BeginRead(tag);
  *pages_ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_NORESERVE, fd_, 0);
  if (*pages_ptr == MAP_FAILED) {
    MV_PANIC("failed to map memory %s", tag.c_str());
  }
  EndRead(tag);

  /* Make memory pages DONTDUMP */
  MV_ASSERT(madvise(*pages_ptr, size, MADV_MERGEABLE) == 0);
  MV_ASSERT(madvise(*pages_ptr, size, MADV_DONTDUMP) == 0);
  return true;
}

int MigrationFileReader::BeginRead(std::string tag) {
  MV_ASSERT(fd_ == -1);
  auto full_path = std::filesystem::path(base_path_) / prefix_ / tag;
  fd_ = open(full_path.c_str(), O_RDONLY);
  if (fd_ == -1) {
    MV_PANIC("failed to read %s", full_path.c_str());
  }

  struct stat st;
  fstat(fd_, &st);
  file_size_ = st.st_size;
  return fd_;
}

void MigrationFileReader::EndRead(std::string tag) {
  MV_UNUSED(tag);
  safe_close(&fd_);
}

size_t MigrationFileReader::ReadRawWithLimit(std::string tag, void* data, size_t limit) {
  MV_UNUSED(tag);
  MV_UNUSED(data);
  MV_UNUSED(limit);
  MV_PANIC("not implemented");
  return 0;
}
