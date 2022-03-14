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

#ifndef _MVISOR_MIGRATION_H
#define _MVISOR_MIGRATION_H

#include <string>
#include <google/protobuf/message.h>

using google::protobuf::Message;

enum MigrationDataType {
  kMigrationDataTypeRaw,
  kMigrationDataTypeString,
  kMigrationDataTypeProtobuf
};

class MigrationWriter {
 public:
  MigrationWriter(std::string base_path);
  ~MigrationWriter();

  void SetPrefix(std::string prefix);
  void WriteRaw(std::string tag, void* data, size_t size);
  void WriteString(std::string tag, const std::string& data);
  void WriteProtobuf(std::string tag, const Message& message);
  void WriteMemoryPages(std::string tag, void* pages, size_t size, uint64_t base);

  inline std::string base_path() { return base_path_; }
 private:
  void BeginWrite(std::string& tag);
  void EndWrite(std::string& tag);

  int         fd_ = -1;
  std::string prefix_;
  std::string base_path_;
};

class MigrationReader {
 public:
  MigrationReader(std::string base_path);
  ~MigrationReader();

  void SetPrefix(std::string prefix);
  bool ReadRaw(std::string tag, void* data, size_t size);
  std::string ReadString(std::string tag);
  bool ReadProtobuf(std::string tag, Message& message);
  bool ReadMemoryPages(std::string tag, void* target, size_t size, uint64_t base);
 private:
  void BeginRead(std::string& tag);
  void EndRead(std::string& tag);

  int         fd_ = -1;
  size_t      file_size_ = 0;
  std::string prefix_;
  std::string base_path_;
};

#endif // _MVISOR_MIGRATION_H
