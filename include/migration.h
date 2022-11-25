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

#include <google/protobuf/message.h>

#include <list>
#include <string>
#include <unordered_map>
#include <queue>

using google::protobuf::Message;


enum MigrationDataType {
  kMigrationDataTypeRaw,
  kMigrationDataTypeString,
  kMigrationDataTypeProtobuf
};

enum MigrationSignalType {
  kMigrateBackingImageComplete,
  kMigrateRamComplete,
  kMigrateImageComplete,
  kMigrateDirtyMemoryFromKvmComplete,
  kMigrateDirtyMemoryFromListenerComplete,
  kMigrateDeviceComplete,
  kMigrateDirtyMemoryFromDmaComplete,
  kMigrateVcpuComplete,
  kMigrateComplete
};

struct MigrationNetworkDataHeader {
  char tag[24];
  size_t size;
};

struct MigrationNetworkData {
  MigrationNetworkDataHeader header;
  uint8_t* body;
};

struct MigrationSignal {
  MigrationSignalType type;
};

class MigrationWriter { 
  public:
    MigrationWriter() = default;
    virtual ~MigrationWriter() = default;
    virtual int  BeginWrite(std::string tag) = 0;
    virtual void SetPrefix(std::string prefix) = 0;
    virtual void EndWrite(std::string tag) = 0;
    virtual bool WriteRaw(std::string tag, void* data, size_t size) = 0;
    virtual bool WriteProtobuf(std::string tag, const Message& message) = 0;
    virtual bool WriteMemoryPages(std::string tag, void* pages, size_t size) = 0;
};

class MigrationFileWriter : public MigrationWriter {
 public:
  MigrationFileWriter(std::string base_path);
  virtual ~MigrationFileWriter();

  virtual int  BeginWrite(std::string tag);
  virtual void SetPrefix(std::string prefix);
  virtual void EndWrite(std::string tag);
  virtual bool WriteRaw(std::string tag, void* data, size_t size);
  virtual bool WriteProtobuf(std::string tag, const Message& message);
  virtual bool WriteMemoryPages(std::string tag, void* pages, size_t size);

  inline std::string base_path() { return base_path_; }

 private:
  int         fd_ = -1;
  std::string prefix_;
  std::string base_path_;
};

class MigrationNetworkWriter : public MigrationWriter {
 public:
  MigrationNetworkWriter();
  virtual ~MigrationNetworkWriter();

  virtual int  BeginWrite(std::string tag);
  virtual void SetPrefix(std::string prefix);
  virtual void EndWrite(std::string tag);
  virtual bool WriteRaw(std::string tag, void* data, size_t size);
  virtual bool WriteProtobuf(std::string tag, const Message& message);
  virtual bool WriteMemoryPages(std::string tag, void* pages, size_t size);

  bool Connect(std::string ip, uint16_t port);
  bool WaitForSignal(MigrationSignalType type);
  bool WriteFromFile(std::string tag, std::string path, size_t offset);

 private:
  int         socket_fd_ = -1;

  bool Write(void* data, size_t size);
};

class MigrationReader {
  public: 
    MigrationReader() = default;
    virtual ~MigrationReader() = default;

    virtual int  BeginRead(std::string tag) = 0;
    virtual void SetPrefix(std::string prefix) = 0;
    virtual void EndRead(std::string tag) = 0;
    virtual bool ReadRaw(std::string tag, void* data, size_t size) = 0;
    virtual bool ReadProtobuf(std::string tag, Message& message) = 0;
    virtual bool ReadMemoryPages(std::string tag, void** pages_ptr, size_t size) = 0;
    virtual size_t ReadRawWithLimit(std::string tag, void* data, size_t limit) = 0;
    virtual bool Exists(std::string tag) = 0;
};

class MigrationFileReader : public MigrationReader {
 public:
  MigrationFileReader(std::string base_path);
  virtual ~MigrationFileReader();

  virtual int  BeginRead(std::string tag);
  virtual void SetPrefix(std::string prefix);
  virtual void EndRead(std::string tag);
  virtual bool ReadRaw(std::string tag, void* data, size_t size);
  virtual bool ReadProtobuf(std::string tag, Message& message);
  virtual bool ReadMemoryPages(std::string tag, void** pages_ptr, size_t size);
  virtual size_t ReadRawWithLimit(std::string tag, void* data, size_t limit);
  virtual bool Exists(std::string tag);

 private:
  int         fd_ = -1;
  size_t      file_size_ = 0;
  std::string prefix_;
  std::string base_path_;
};

class MigrationNetworkReader : public MigrationReader {
 public:
  MigrationNetworkReader();
  virtual ~MigrationNetworkReader();

  virtual int  BeginRead(std::string tag);
  virtual void SetPrefix(std::string prefix);
  virtual void EndRead(std::string tag);
  virtual bool ReadRaw(std::string tag, void* data, size_t size);
  virtual bool ReadProtobuf(std::string tag, Message& message);
  virtual bool ReadMemoryPages(std::string tag, void** pages_ptr, size_t size);
  virtual size_t ReadRawWithLimit(std::string tag, void* data, size_t limit);
  virtual bool Exists(std::string tag);

  bool WaitForConnection(uint16_t port);
  void SendSignal(MigrationSignalType type);
  void ReadToFile(std::string tag, std::string path, size_t offset);

 private:
  int socket_fd_ = -1;
  std::unordered_map<std::string, std::queue<MigrationNetworkData*>> cache_map_;

  void Read(void* data, size_t size);
  void FreeCacheData(MigrationNetworkData* data);
  MigrationNetworkDataHeader WaitForDataHeader(std::string tag);
  MigrationNetworkData* ReadFromCache(std::string tag);
};

#endif // _MVISOR_MIGRATION_H
