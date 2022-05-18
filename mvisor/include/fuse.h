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
#include <thread>

#define FUSE_USE_VERSION 34

#include <dirent.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>

#include <algorithm>
#include <climits>
#include <list>

#include "fuse/fuse_kernel.h"
#include "fuse/fuse_lowlevel.h"
#include "logger.h"

#define OFFSET(in_arg) (((char*)(in_arg)) + sizeof(*(in_arg)))

struct lo_inode {
  struct lo_inode* next; /* protected by lo->mutex */
  struct lo_inode* prev; /* protected by lo->mutex */
  int fd;
  ino_t ino;
  dev_t dev;
  uint64_t refcount; /* protected by lo->mutex */
};

struct lo_data {
  pthread_mutex_t mutex;
  int debug;
  int writeback;
  int flock;
  int xattr;
  const char* source;
  double timeout;
  int cache;
  int timeout_set;
  struct lo_inode* root; /* protected by lo->mutex */
};

struct lo_dirp {
  DIR* dp;
  struct dirent* entry;
  off_t offset;
};

enum {
  CACHE_NEVER,
  CACHE_NORMAL,
  CACHE_ALWAYS,
};

class Fuse {
 private:
  Fuse(std::string& mount_path);

  static Fuse* instance_;
  struct lo_data* lo_data_;
  std::list<lo_inode*> inode_list_;

  struct lo_inode* LowLevelFind(struct stat* stat);

  /* fuse functions */
  int FuseBufvecAdvance(struct fuse_bufvec* buf_vec, size_t length);
  void FuseFillEntry(struct fuse_entry_out* arg, const struct fuse_entry_param* entry_param);
  size_t FuseBufSize(const struct fuse_bufvec* buf_vec);
  size_t FuseAddDirentryPlus(char* buf, size_t buf_size, const char* name,
                             const struct fuse_entry_param* entry_param, off_t off);
  size_t FuseAddDirentry(char* buf, size_t buf_size, const char* name, const struct stat* stbuf, off_t off);
  ssize_t FuseBufWrite(const struct fuse_buf* dst_buf, size_t dst_offset,
                       const struct fuse_buf* src_buf, size_t src_offset, size_t length);
  ssize_t FuseBufRead(const struct fuse_buf* dst_buf, size_t dst_offset,
                      const struct fuse_buf* src_buf, size_t src_offset, size_t length);
  ssize_t FuseBufCopyOne(const struct fuse_buf* dst, size_t dst_offset, const struct fuse_buf* src,
                         size_t src_offset, size_t length, enum fuse_buf_copy_flags flags);

 public:
  static Fuse* GetInstance(std::string& mount_path) {
    // no need to share
    if (instance_ == nullptr) {
      instance_ = new Fuse(mount_path);
    }
    return instance_;
  }
  virtual ~Fuse();

  int LowLevelFd(fuse_ino_t ino);
  int LowLevelLookup(fuse_ino_t parent, const char* name, struct fuse_entry_param* entry_param);
  int MakeNodeWrapper(int dirfd, const char* path, const char* link, int mode, dev_t rdev);

  void UnrefInode(struct lo_data* lo, struct lo_inode* inode, uint64_t n);
  void ConvertStat(const struct stat* stbuf, struct fuse_attr* attr);
  void ConvertStatfs(const struct statvfs* stbuf, struct fuse_kstatfs* kstatfs);
  void LowLevelForgetOne(fuse_ino_t ino, uint64_t nlookup);

  bool LowLevelReadDir(fuse_in_header* in, char* result_buf, uint32_t* remain_size, bool is_plus);
  ssize_t FuseBufCopy(struct fuse_bufvec* dstv, struct fuse_bufvec* srcv, enum fuse_buf_copy_flags flags);
  struct lo_inode* GetLowLevelInode(fuse_ino_t inode);

  inline struct lo_data* get_lo_data() { return lo_data_; }

  inline const struct fuse_buf* FuseBufvecCurrent(struct fuse_bufvec* buf_vec) {
    if (buf_vec->idx < buf_vec->count) {
      return &buf_vec->buf[buf_vec->idx];
    } else {
      return nullptr;
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

  inline int IsDotOrDotdot(const char* name) {
    return name[0] == '.' && (name[1] == '\0' || (name[1] == '.' && name[2] == '\0'));
  }

  inline void MakeResponse(fuse_out_header* response, uint32_t len, int32_t error,
                           uint64_t unique) {
    MV_ASSERT(response != nullptr && len >= 0);
    response->error = error;
    response->unique = unique;
    response->len = sizeof(fuse_out_header) + len;
  }
};

#endif  // _MVISOR_FUSE_H
