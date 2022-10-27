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

#ifndef _MVISOR_FUSE_H
#define _MVISOR_FUSE_H

#include <dirent.h>

#include <cmath>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <thread>

#define FUSE_USE_VERSION 34

#include <dirent.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <deque>
#include <algorithm>
#include <climits>
#include <list>

#include "fuse_kernel.h"
#include "fuse_lowlevel.h"
#include "logger.h"

#define MAX_PATH 260
#define BLOCK_SIZE 4096

struct Inode {
  int fd;
  int refcount;
  ino_t ino;
  dev_t dev;
};

struct UserConfig {
  char* source;
  double timeout;
  struct Inode* root;
};

struct Directory {
  DIR* dp;
  struct dirent* entry;
  off_t offset;
};

struct DataBuffer {
  void* address;
  size_t   size;
};

struct DiskInfo {
  uint64_t size_used;
  uint64_t size_limit;
  uint64_t inode_count;
  uint64_t inode_limit;
};

class Fuse {
 private:
  struct UserConfig user_config_;
  std::list<Inode*> inode_list_;
  DiskInfo disk_info_;

  bool FilePathSafeCheck(int fd);

  Inode* CreateInodeFromFd(int fd, struct stat* stat, int refcount);

  void UpdateDiskInfo();

  size_t AddDirentry(char* buf, size_t buf_size, const char* name, const struct stat* stbuf, off_t off);
  size_t AddDirentryPlus(char* buf, size_t buf_size, const char* name, const struct fuse_entry_param* entry_param, off_t off);

  inline bool IsDotOrDotdot(const char* name) {
    return name[0] == '.' && (name[1] == '\0' || (name[1] == '.' && name[2] == '\0'));
  }

 public:
  Fuse(std::string& mount_path, uint64_t disk_size_limit, uint64_t inode_count_limit);
  virtual ~Fuse();

  int GetFdFromInode(fuse_ino_t ino);

  void ClearInodeList(bool clear_root);
  void UnrefInode(struct Inode* inode, uint64_t refcount);
  void CopyStatFs(const struct statvfs* stbuf, struct fuse_kstatfs* kstatfs);
  void ConvertStatToFuseAttr(const struct stat* stbuf, struct fuse_attr* attr);
  void ModifyDiskInformationToVm(struct statvfs* new_stat_vfs);

  bool Lookup(fuse_ino_t parent, const char* name, struct fuse_entry_param* entry_param);
  bool ReadDirectory(fuse_read_in* read_in, uint64_t nodeid, char* result_buf, uint32_t* result_buf_len, bool is_plus);
  
  Inode* GetInode(fuse_ino_t inode);
  Inode* GetInodeFromFd(int fd);
  Inode* GetInodeFromStat(struct stat* stat);

  DataBuffer GetDataBufferFromIovec(std::deque<struct iovec>& iovec, size_t size);

  inline struct UserConfig user_config() const { return user_config_; }

  inline bool IsDiskSpaceLeftEnough(uint64_t size) { return disk_info_.size_used + size <= disk_info_.size_limit; }

  inline bool IsInodeListFull() { return inode_list_.size() > disk_info_.inode_limit; }

  inline void CostDiskSpace(uint64_t size) { disk_info_.size_used += size; }

  inline void ReleaseDiskSpace(uint64_t size) { 
    if (disk_info_.size_used < size) {
      // the host can write in the mount path shared with vm
      UpdateDiskInfo();
    } else {
      disk_info_.size_used -= size;
    }
  }

  inline unsigned long CalcTimeoutSecond(double t) {
    if (t > (double)ULONG_MAX) {
      return ULONG_MAX;
    } else if (t < 0.0) {
      return 0;
    } else {
      return (unsigned long)t;
    }
  }

  inline unsigned int CalcTimeoutNsecond(double t) {
    double f = t - (double)CalcTimeoutSecond(t);
    if (f < 0.0) {
      return 0;
    } else if (f >= 0.999999999) {
      return 999999999;
    } else {
      return (unsigned int)(f * 1.0e9);
    }
  }

  inline void MakeResponse(fuse_out_header* response, uint32_t len, int32_t error, uint64_t unique) {
    MV_ASSERT(response != nullptr);
    response->error = error;
    response->unique = unique;
    response->len = sizeof(fuse_out_header) + len;
  }
};

#endif  // _MVISOR_FUSE_H
