/* 
 * MVisor
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

#include "migration.h"
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <filesystem>
#include "logger.h"
#include "utilities.h"
#include <google/protobuf/text_format.h>

using namespace std::filesystem;

MigrationWriter::MigrationWriter(std::string base_path) {
  if (!std::filesystem::exists(base_path)) {
    std::filesystem::create_directory(base_path);
  }
  base_path_ = base_path;
}

MigrationWriter::~MigrationWriter() {

}

void MigrationWriter::SetPrefix(std::string prefix) {
  prefix_ = prefix;
}

void MigrationWriter::WriteRaw(std::string tag, void* data, size_t size) {
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
}

void MigrationWriter::WriteString(std::string tag, const std::string& data) {
  BeginWrite(tag);
  write(fd_, data.data(), data.length());
  EndWrite(tag);
}

void MigrationWriter::WriteProtobuf(std::string tag, const Message& message) {
  BeginWrite(tag);
  message.SerializePartialToFileDescriptor(fd_);
  EndWrite(tag);
}

void MigrationWriter::WriteMemoryPages(std::string tag, void* target, size_t size, uint64_t base) {
  MV_PANIC("not impl");
}

int MigrationWriter::BeginWrite(std::string tag) {
  MV_ASSERT(fd_ == -1);
  auto full_path = path(base_path_) / prefix_;
  if (!exists(full_path)) {
    create_directories(full_path);
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

void MigrationWriter::EndWrite(std::string tag) {
  safe_close(&fd_);
}



MigrationReader::MigrationReader(std::string base_path) {
  base_path_ = base_path;
}

MigrationReader::~MigrationReader() {

}

void MigrationReader::SetPrefix(std::string prefix) {
  prefix_ = prefix;
}

bool MigrationReader::ReadRaw(std::string tag, void* data, size_t size) {
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

std::string MigrationReader::ReadString(std::string tag) {
  std::string str;
  BeginRead(tag);
  str.resize(file_size_);
  read(fd_, str.data(), file_size_);
  EndRead(tag);
  return str;
}

bool MigrationReader::ReadProtobuf(std::string tag, Message& message) {
  BeginRead(tag);
  message.Clear();
  bool ret = message.ParseFromFileDescriptor(fd_);
  EndRead(tag);
  return ret;
}

bool MigrationReader::ReadMemoryPages(std::string tag, void* target, size_t size, uint64_t base) {
  MV_PANIC("not impl");
  return true;
}

int MigrationReader::BeginRead(std::string tag) {
  MV_ASSERT(fd_ == -1);
  auto full_path = path(base_path_) / prefix_ / tag;
  fd_ = open(full_path.c_str(), O_RDONLY);
  MV_ASSERT(fd_ != -1);

  struct stat st;
  fstat(fd_, &st);
  file_size_ = st.st_size;
  return fd_;
}

void MigrationReader::EndRead(std::string tag) {
  safe_close(&fd_);
}
