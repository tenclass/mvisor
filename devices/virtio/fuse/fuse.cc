/*
 * MVisor
 * fuse.cc is based on libfuse3
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

#include "fuse.h"

Fuse::Fuse(std::string& mount_path, uint64_t disk_size_limit, uint64_t inode_count_limit) {
  // start fuse init process
  user_config_.timeout = 1.0;

  // set mount path from config
  user_config_.source = new char[mount_path.length() + 1];
  strcpy(user_config_.source, mount_path.c_str());

  // check mount path must be director
  struct stat stat;
  MV_ASSERT(lstat(user_config_.source, &stat) != -1);
  MV_ASSERT(S_ISDIR(stat.st_mode));

  // new root dir
  user_config_.root = new Inode;
  MV_ASSERT(user_config_.root != nullptr);

  // in case vm start from snapshot
  user_config_.root->refcount = 2;
  user_config_.root->ino = stat.st_ino;
  user_config_.root->dev = stat.st_dev;
  user_config_.root->fd = open(user_config_.source, O_PATH);
  MV_ASSERT(user_config_.root->fd != -1);
  inode_list_.push_front(user_config_.root);

  // init disk size
  disk_info_.size_limit = disk_size_limit;
  disk_info_.inode_limit = inode_count_limit;
  UpdateDiskInfo();
  MV_ASSERT(disk_info_.size_limit >= disk_info_.size_used && disk_info_.inode_limit >= disk_info_.inode_count);
}

Fuse::~Fuse() {
  if (user_config_.source) {
    delete[] user_config_.source;
  }
  
  ClearInodeList(true);
}

void Fuse::UpdateDiskInfo() {
  disk_info_.size_used = 0;
  disk_info_.inode_count = 0;
  for (auto& entry : std::filesystem::recursive_directory_iterator(user_config_.source)) {
    if (entry.is_regular_file()) {
      disk_info_.size_used += entry.file_size();
    }
    disk_info_.inode_count++;
  }
}

struct Inode* Fuse::GetInode(fuse_ino_t inode) {
  if (inode == FUSE_ROOT_ID) {
    return user_config_.root;
  }
  auto iter = std::find(inode_list_.begin(), inode_list_.end(), (struct Inode*)(uintptr_t)inode);
  if (iter != inode_list_.end()) {
    return *iter;
  } else {
    return nullptr;
  }
}

int Fuse::GetFdFromInode(fuse_ino_t ino) {
  Inode* inode = GetInode(ino);
  if (inode == nullptr) {
    return -1;
  }
  return inode->fd;
}

Inode* Fuse::GetInodeFromFd(int fd) {
  auto inode = std::find_if(inode_list_.begin(), inode_list_.end(), [=](struct Inode* inode) -> bool {
    return inode->fd == fd;
  });

  if (inode == inode_list_.end()) {
    return nullptr;
  } else {
    return *inode;
  }
}

struct Inode* Fuse::GetInodeFromStat(struct stat* stat) {
  auto inode = std::find_if(inode_list_.begin(), inode_list_.end(), [=](struct Inode* inode) -> bool {
    return inode->dev == stat->st_dev && inode->ino == stat->st_ino;
  });

  if (inode == inode_list_.end()) {
    return nullptr;
  } else {
    return *inode;
  }
}

void Fuse::ModifyDiskInformationToVm(struct statvfs* new_stat_vfs) {
  // we need to update disk info in case that the host files in mount path was updated
  UpdateDiskInfo();

  // the host can write in the mount path shared with vm
  if (disk_info_.size_used > disk_info_.size_limit) {
    disk_info_.size_limit = disk_info_.size_used;
  }
  if (disk_info_.inode_count > disk_info_.inode_limit) {
    disk_info_.inode_limit = disk_info_.inode_count;
  }

  new_stat_vfs->f_files = disk_info_.inode_limit;
  new_stat_vfs->f_blocks = disk_info_.size_limit / BLOCK_SIZE;
  new_stat_vfs->f_bfree = new_stat_vfs->f_bavail = (disk_info_.size_limit - disk_info_.size_used) / BLOCK_SIZE;
  new_stat_vfs->f_ffree = new_stat_vfs->f_favail = disk_info_.inode_limit - inode_list_.size();
}

void Fuse::ClearInodeList(bool clear_root) {
  for (auto it = inode_list_.begin(); it != inode_list_.end();) {
    if(!clear_root && (*it)->fd == user_config_.root->fd) {
      it++;
    } else {
      close((*it)->fd);
      delete *it;
      it = inode_list_.erase(it);
    }
  }
}

bool Fuse::FilePathSafeCheck(int fd) {
  // get fd file path
  char file_path[PATH_MAX] = {0};
  auto fd_path = "/proc/self/fd/" + std::to_string(fd);
  auto len = readlink(fd_path.c_str(), file_path, PATH_MAX);
  if (len == -1) {
    MV_ERROR("readlink failed fd=%d errno=%d", fd, -errno);
    return false;
  }

  std::string mount_path = user_config_.source;
  auto compare_len = mount_path.size();
  if (len < (ssize_t)compare_len) {
    MV_ERROR("file path too short len=%d", len);
    return false;
  }

  // fd path must start with mount_path_
  std::string file_path_str = file_path;
  if (0 != file_path_str.compare(0, compare_len, mount_path, 0, compare_len)) {
    MV_PANIC("guest attempt to get out of share root path=%s", file_path);
    return false;
  }

  return true;
}

Inode* Fuse::CreateInodeFromFd(int fd, struct stat* stat, int refcount) {
  if (!FilePathSafeCheck(fd)) {
    return nullptr;
  }

  Inode* inode = new Inode{
    .fd = fd,
    .refcount = refcount,
    .ino = stat->st_ino,
    .dev = stat->st_dev
  };
  MV_ASSERT(inode != nullptr);
  inode_list_.push_front(inode);
  return inode;
}

bool Fuse::Lookup(fuse_ino_t parent, const char* name, struct fuse_entry_param* entry_param) {
  entry_param->generation = 0;
  entry_param->attr_timeout = user_config_.timeout;
  entry_param->entry_timeout = user_config_.timeout;

  auto parent_fd = GetFdFromInode(parent);
  MV_ASSERT(parent_fd != -1);
  
  auto ret = fstatat(parent_fd, name, &entry_param->attr, 0);
  if (ret != 0) {
    return false;
  }

  Inode* inode = GetInodeFromStat(&entry_param->attr);
  if (!inode) {
    auto open_flags = S_ISREG(entry_param->attr.st_mode) ? O_RDWR : O_PATH | O_NOFOLLOW;
    int new_fd = openat(parent_fd, name, open_flags);
    if (new_fd == -1) {
      MV_ERROR("lookup openat err name=%s open_flags=%d", name, open_flags);
      return false;
    }

    inode = CreateInodeFromFd(new_fd, &entry_param->attr, 1);
    MV_ASSERT(inode != nullptr);
  } else {
    inode->refcount++;
  }
  
  entry_param->ino = (uintptr_t)inode;
  return true;
}

void Fuse::ConvertStatToFuseAttr(const struct stat* stbuf, struct fuse_attr* attr) {
  attr->ino = stbuf->st_ino;
  attr->mode = stbuf->st_mode;
  attr->nlink = stbuf->st_nlink;
  attr->uid = stbuf->st_uid;
  attr->gid = stbuf->st_gid;
  attr->rdev = stbuf->st_rdev;
  attr->size = stbuf->st_size;
  attr->blksize = stbuf->st_blksize;
  attr->blocks = stbuf->st_blocks;
  attr->atime = stbuf->st_atime;
  attr->mtime = stbuf->st_mtime;
  attr->ctime = stbuf->st_ctime;
  attr->atimensec = 0;
  attr->mtimensec = 0;
  attr->ctimensec = 0;
}

void Fuse::CopyStatFs(const struct statvfs* stbuf, struct fuse_kstatfs* kstatfs) {
  kstatfs->bsize = stbuf->f_bsize;
  kstatfs->frsize = stbuf->f_frsize;
  kstatfs->blocks = stbuf->f_blocks;
  kstatfs->bfree = stbuf->f_bfree;
  kstatfs->bavail = stbuf->f_bavail;
  kstatfs->files = stbuf->f_files;
  kstatfs->ffree = stbuf->f_ffree;
  kstatfs->namelen = stbuf->f_namemax;
}

void Fuse::UnrefInode(struct Inode* inode, uint64_t refcount) {
  inode->refcount -= refcount;
  MV_ASSERT(inode->refcount >= 0);

  if (inode->refcount == 0) {
    close(inode->fd);
    inode_list_.remove(inode);
    delete inode;
  }
}

size_t Fuse::AddDirentryPlus(char* buf, size_t buf_size, const char* name,
                                 const struct fuse_entry_param* entry_param, off_t off) {
  size_t name_length = strlen(name);
  size_t entry_length = FUSE_NAME_OFFSET_DIRENTPLUS + name_length;
  size_t entry_length_padded = FUSE_DIRENT_ALIGN(entry_length);
  if (entry_length_padded > buf_size) {
    return 0;
  }

  struct fuse_direntplus* dp = (struct fuse_direntplus*)buf;
  bzero(&dp->entry_out, sizeof(dp->entry_out));
  
  dp->entry_out.nodeid = entry_param->ino;
  dp->entry_out.generation = entry_param->generation;
  dp->entry_out.entry_valid = CalcTimeoutSecond(entry_param->entry_timeout);
  dp->entry_out.entry_valid_nsec = CalcTimeoutNsecond(entry_param->entry_timeout);
  dp->entry_out.attr_valid = CalcTimeoutSecond(entry_param->attr_timeout);
  dp->entry_out.attr_valid_nsec = CalcTimeoutNsecond(entry_param->attr_timeout);
  ConvertStatToFuseAttr(&entry_param->attr, &dp->entry_out.attr);

  dp->dirent.ino = entry_param->attr.st_ino;
  dp->dirent.off = off;
  dp->dirent.namelen = name_length;
  dp->dirent.type = (entry_param->attr.st_mode & S_IFMT) >> 12;
  memcpy(dp->dirent.name, name, name_length);
  bzero(dp->dirent.name + name_length, entry_length_padded - entry_length);
  return entry_length_padded;
}

size_t Fuse::AddDirentry(char* buf, size_t buf_size, const char* name, const struct stat* stat, off_t off) {
  size_t name_length = strlen(name);
  size_t entry_length = FUSE_NAME_OFFSET + name_length;
  size_t enttry_length_padded = FUSE_DIRENT_ALIGN(entry_length);
  if (enttry_length_padded > buf_size) {
    return 0;
  }

  struct fuse_dirent* dirent;
  dirent = (struct fuse_dirent*)buf;
  dirent->ino = stat->st_ino;
  dirent->off = off;
  dirent->namelen = name_length;
  dirent->type = (stat->st_mode & S_IFMT) >> 12;
  memcpy(dirent->name, name, name_length);
  bzero(dirent->name + name_length, enttry_length_padded - entry_length);
  return enttry_length_padded;
}

bool Fuse::ReadDirectory(fuse_read_in* read_in, uint64_t nodeid, char* result_buf, uint32_t* result_buf_len, bool is_plus) {
  auto dirp = (Directory*)read_in->fh;
  if (read_in->offset != (uint64_t)dirp->offset) {
    seekdir(dirp->dp, read_in->offset);
    dirp->entry = nullptr;
  }

  // get result in loop
  bool lookup_err = false;
  auto remain_buf_len = read_in->size;
  auto result_buf_pointer = result_buf;
  while (true) {
    if (!dirp->entry) {
      dirp->entry = readdir(dirp->dp);
      if (!dirp->entry) {
        break;
      }
    }

    fuse_ino_t entry_ino = 0;
    struct fuse_entry_param entry_param;
    bzero(&entry_param, sizeof(entry_param));

    auto name = dirp->entry->d_name;
    auto next_off = dirp->entry->d_off;
    if (!is_plus || IsDotOrDotdot(name)) {
      entry_param.attr.st_ino = dirp->entry->d_ino;
      entry_param.attr.st_mode = dirp->entry->d_type << 12;
    } else {
      if (!Lookup(nodeid, name, &entry_param)) {
          lookup_err = true;
          break;
      }
      entry_ino = entry_param.ino;
    }

    size_t entry_size;
    if (is_plus) {
      entry_size = AddDirentryPlus(result_buf_pointer, remain_buf_len, name, &entry_param, next_off);
    } else {
      entry_size = AddDirentry(result_buf_pointer, remain_buf_len, name, &entry_param.attr, next_off);
    }

    if (!entry_size) {
      if (entry_ino) {
        UnrefInode((Inode*)entry_ino, 1);
      }
      MV_ERROR("FuseAddDirentry/FuseAddDirentryPlus error name=%s is_plus=%d errno=%d", name, is_plus, -errno);
      lookup_err = true;
      break;
    }

    MV_ASSERT(remain_buf_len > entry_size);
    remain_buf_len -= entry_size;
    result_buf_pointer += entry_size;
    dirp->entry = nullptr;
  }

  *result_buf_len = read_in->size - remain_buf_len;
  return !lookup_err;
}

DataBuffer Fuse::GetDataBufferFromIovec(std::deque<struct iovec>& iovec, size_t size) {
  // get request from guest
  MV_ASSERT(!iovec.empty());

  auto& front = iovec.front();
  MV_ASSERT(front.iov_len >= size);

  DataBuffer buffer = {
    .address = front.iov_base,
    .size = size == 0 ? front.iov_len : size
  };

  if(buffer.size == front.iov_len) {
    // run out of this iov
    iovec.pop_front();
  } else {
    front.iov_len -= buffer.size;
    front.iov_base = (uint8_t*)front.iov_base + buffer.size;
  }

  return buffer;
}