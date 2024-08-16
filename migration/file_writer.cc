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


MigrationFileWriter::MigrationFileWriter(std::string base_path) {
if (std::filesystem::exists(base_path)) {
    std::filesystem::remove_all(base_path);
  }
  std::filesystem::create_directory(base_path);

  base_path_ = base_path;
}

MigrationFileWriter::~MigrationFileWriter() {
  
}

void MigrationFileWriter::SetPrefix(std::string prefix) {
  prefix_ = prefix;
}

bool MigrationFileWriter::WriteRaw(std::string tag, void* data, size_t size) {
  BeginWrite(tag);
  auto ptr = (uint8_t*)data;
  while (size > 0) {
    auto ret = write(fd_, ptr, size);
    if (ret <= 0) {
      MV_PANIC("failed to write fd=%d ret=%ld", fd_, ret);
    }
    ptr += ret;
    size -= ret;
  }
  EndWrite(tag);
  return true;
}

bool MigrationFileWriter::WriteProtobuf(std::string tag, const Message& message) {
  BeginWrite(tag);
  message.SerializePartialToFileDescriptor(fd_);
  EndWrite(tag);
  return true;
}

bool MigrationFileWriter::WriteMemoryPages(std::string tag, void* pages, size_t size) {
  /* Write RAM to sparse file */
  BeginWrite(tag);
  MV_ASSERT(ftruncate(fd_, size) == 0);

  auto ptr = (uint8_t*)pages;
  for (size_t pos = 0; pos < size; pos += PAGE_SIZE) {
    if (!test_zero(ptr, PAGE_SIZE)) {
      MV_ASSERT(pwrite(fd_, ptr, PAGE_SIZE, pos) == PAGE_SIZE);
    }
    ptr += PAGE_SIZE;
  }
  
  EndWrite(tag);
  return true;
}

int MigrationFileWriter::BeginWrite(std::string tag) {
  MV_ASSERT(fd_ == -1);
  auto full_path = std::filesystem::path(base_path_) / prefix_;
  if (!std::filesystem::exists(full_path)) {
    std::filesystem::create_directories(full_path);
  }
  full_path /= tag;

  /* Removing a file is faster than truncating, why ?? */
  if (exists(full_path)) {
    std::filesystem::remove(full_path);
  }

  fd_ = open(full_path.c_str(), O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
  MV_ASSERT(fd_ != -1);
  return fd_;
}

void MigrationFileWriter::EndWrite(std::string tag) {
  MV_UNUSED(tag);
  safe_close(&fd_);
}

