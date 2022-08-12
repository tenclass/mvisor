/*
 * MVisor VirtIO-FS Device
 * based on libfuse3 passthrough_ll.c
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

#include <linux/virtio_fs.h>
#include <sys/stat.h>
#include <sys/xattr.h>

#include <filesystem>

#include "device_interface.h"
#include "fuse.h"
#include "logger.h"
#include "virtio_pci.h"

// size < 1024 may cause fuse copy file crash
// https://github.com/virtio-win/kvm-guest-drivers-windows/issues/760
#define DEFAULT_QUEUE_SIZE 1024

class VirtioFs : public VirtioPci, public VirtioFsInterface {
 private:
  Fuse* fuse_ = nullptr;
  std::string mount_path_;
  virtio_fs_config fs_config_;
  std::unordered_set<uint64_t> dirp_pointer_set_;
  std::vector<VirtioFsListener> virtio_fs_listeners_;

 public:
  VirtioFs() {
    pci_header_.class_code = 0x018000;
    pci_header_.vendor_id = 0x1AF4;
    pci_header_.subsys_id = 0x1100;
    pci_header_.device_id = 0x105A;
    pci_header_.revision_id = 0x01;

    AddPciBar(1, 0x1000, kIoResourceTypeMmio);
    device_features_ |= 0;
  }

  virtual void Disconnect() {
    Reset();

    // free dir handle
    ClearDirectoryPointerSet();

    // delete Fuse object
    if (fuse_) {
      delete fuse_;
      fuse_ = nullptr;
    }

    VirtioPci::Disconnect();
  }

  virtual void Connect() {
    if (has_key("path")) {
      // init fuse
      mount_path_ = std::get<std::string>(key_values_["path"]);
      MV_ASSERT(std::filesystem::exists(mount_path_));
      MV_ASSERT(std::filesystem::is_directory(mount_path_));

      // set disk space
      auto disk_size_limit = (uint64_t)2 * 1024 * 1024 * 1024;
      if (has_key("disk_size")) {
        std::string disk_size = std::get<std::string>(key_values_["disk_size"]);
        if (disk_size.back() != 'G') {
          MV_PANIC("disk size must be aligned with 1GB");
        }
        long value = atol(disk_size.substr(0, disk_size.length() - 1).c_str());
        disk_size_limit = (1UL << 30) * value;
      }

      // set inode count limit
      uint64_t inode_count_limit = 200;
      if (has_key("inode_count")) {
        inode_count_limit = std::get<uint64_t>(key_values_["inode_count"]);
      }

      fuse_ = new Fuse(mount_path_, disk_size_limit, inode_count_limit);
      MV_ASSERT(fuse_ != nullptr);

      // set fs config
      // num_request_queues > 1 cause windows vm crash
      bzero(&fs_config_, sizeof(fs_config_));
      fs_config_.num_request_queues = 1;

      if (has_key("disk_name")) {
        std::string name = std::get<std::string>(key_values_["disk_name"]);
        MV_ASSERT(name.length() < sizeof(fs_config_.tag));
        strcpy((char*)fs_config_.tag, name.c_str());
      } 

      // connect
      VirtioPci::Connect();
    }
  }

  void RegisterVirtioFsListener(VirtioFsListener callback) { 
    virtio_fs_listeners_.push_back(callback); 
  }

  void NotifyVirtioFs() {
    // notify host to refresh share dir
    for (auto& callback : virtio_fs_listeners_) {
      callback();
    }
  }

  void Reset() {
    /* Reset all queues */
    VirtioPci::Reset();

    for (uint32_t i = 0; i < 1 + fs_config_.num_request_queues; ++i) {
      AddQueue(DEFAULT_QUEUE_SIZE, std::bind(&VirtioFs::OnOutput, this, i));
    }
  }

  void ReadDeviceConfig(uint64_t offset, uint8_t* data, uint32_t size) {
    MV_ASSERT(offset + size <= sizeof(fs_config_));
    memcpy(data, (uint8_t*)&fs_config_ + offset, size);
  }

  void OnOutput(int queue_index) {
    if (fuse_ == nullptr) {
      return;
    }

    auto& vq = queues_[queue_index];
    while (auto element = PopQueue(vq)) {
      HandleCommand(element);
      PushQueue(vq, element);
    }
    NotifyQueue(vq);
  }

  void ClearDirectoryPointerSet() {
    for (auto dirp_pointer : dirp_pointer_set_) {
      closedir(((Directory*)dirp_pointer)->dp);
      delete (Directory*)dirp_pointer;
    }
    dirp_pointer_set_.clear();
  }

  void FuseInit(VirtElement* element, fuse_in_header* request) {
    fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_init_in));
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;
    auto out = (fuse_init_out*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_init_out)).address;

    // set out params
    out->major = FUSE_KERNEL_VERSION;
    out->minor = FUSE_KERNEL_MINOR_VERSION;
    out->flags = FUSE_BIG_WRITES | FUSE_DO_READDIRPLUS;
    out->max_readahead = UINT32_MAX;
    out->max_write = UINT32_MAX;
    fuse_->MakeResponse(response, sizeof(fuse_init_out), 0, request->unique);
    element->length = response->len;
  }

  void FuseStatFs(VirtElement* element, fuse_in_header* request) {
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;
    auto out = (fuse_statfs_out*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_statfs_out)).address;

    int fd = fuse_->GetFdFromInode(request->nodeid);
    MV_ASSERT(fd != -1);

    struct statvfs new_stat_vfs = {0};
    MV_ASSERT(fstatvfs(fd, &new_stat_vfs) != -1);

    // modify disk information to vm
    fuse_->ModifyDiskInformationToVm(&new_stat_vfs);

    fuse_->CopyStatFs(&new_stat_vfs, &out->st);
    fuse_->MakeResponse(response, sizeof(fuse_statfs_out), 0, request->unique);
    element->length = response->len;
  }

  void FuseLookup(VirtElement* element, fuse_in_header* request) {
    auto name = (const char*)fuse_->GetDataBufferFromIovec(element->vector, request->len - sizeof(fuse_in_header)).address;
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;
    auto out = (fuse_entry_out*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_entry_out)).address;

    struct fuse_entry_param entry_param = {0};
    if (!fuse_->Lookup(request->nodeid, name, &entry_param)) {
      fuse_->MakeResponse(response, 0, -errno, request->unique);
      element->length = response->len;
      return;
    } 

    out->nodeid = entry_param.ino;
    out->generation = entry_param.generation;
    out->entry_valid = fuse_->CalcTimeoutSecond(entry_param.entry_timeout);
    out->entry_valid_nsec = fuse_->CalcTimeoutNsecond(entry_param.entry_timeout);
    out->attr_valid = fuse_->CalcTimeoutSecond(entry_param.attr_timeout);
    out->attr_valid_nsec = fuse_->CalcTimeoutNsecond(entry_param.attr_timeout);
    fuse_->ConvertStatToFuseAttr(&entry_param.attr, &out->attr);
    fuse_->MakeResponse(response, sizeof(fuse_entry_out), 0, request->unique);
    element->length = response->len;
  }

  void FuseOpen(VirtElement* element, fuse_in_header* request) {
    fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_open_in));
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;
    auto out = (fuse_open_out*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_open_out)).address;

    int fd = fuse_->GetFdFromInode(request->nodeid);
    if (fd == -1) {
      fuse_->MakeResponse(response, 0, -errno, request->unique);
      element->length = response->len;
      return;
    }

    out->fh = fd;
    fuse_->MakeResponse(response, sizeof(fuse_open_out), 0, request->unique);
    element->length = response->len;
  }

  void FuseReadLink(VirtElement* element, fuse_in_header* request) {
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;
    auto out_len = request->len - sizeof(fuse_in_header);
    auto out = (char*)fuse_->GetDataBufferFromIovec(element->vector, out_len).address;

    int fd = fuse_->GetFdFromInode(request->nodeid);
    MV_ASSERT(fd != -1);

    auto ret = readlinkat(fd, "", out, out_len);
    MV_ASSERT(ret != -1);
    fuse_->MakeResponse(response, ret, 0, request->unique);
    element->length = response->len;
  }

  void FuseRelease(VirtElement* element, fuse_in_header* request) {
    auto in = (fuse_release_in*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_release_in)).address;
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;
    auto inode = fuse_->GetInodeFromFd(in->fh);
    fuse_->MakeResponse(response, 0, inode != nullptr ? 0 : -2, request->unique);
    element->length = response->len;
  }

  void FuseOpenDir(VirtElement* element, fuse_in_header* request) {
    fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_open_in));
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;
    auto out = (fuse_open_out*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_open_out)).address;

    int fd = fuse_->GetFdFromInode(request->nodeid);
    if (fd == -1) {
      fuse_->MakeResponse(response, 0, -2, request->unique);
      element->length = response->len;
      return;
    }

    // the fd from node list is opened with flag "O_PATH" which can not be used in read dir
    auto fd_copy = openat(fd, ".", O_RDONLY);
    if (fd_copy == -1) {
      fuse_->MakeResponse(response, 0, -errno, request->unique);
      element->length = response->len;
      return;
    }

    // we need to remember directory pointer to release in FuseReleaseDir
    auto dirp = new Directory{ 
      .dp = fdopendir(fd_copy),
      .entry = nullptr,
      .offset = 0
    };
    MV_ASSERT(dirp != nullptr && dirp->dp != nullptr);
    dirp_pointer_set_.insert((uint64_t)dirp);

    out->fh = (uint64_t)dirp;
    fuse_->MakeResponse(response, sizeof(fuse_open_out), 0, request->unique);
    element->length = response->len;
  }

  void FuseReleaseDir(VirtElement* element, fuse_in_header* request) {
    auto in = (fuse_release_in*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_release_in)).address;
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;

    if (dirp_pointer_set_.erase(in->fh)) {
      auto dirp = (Directory*)in->fh;
      closedir(dirp->dp);
      delete dirp;
    }

    fuse_->MakeResponse(response, 0, 0, request->unique);
    element->length = response->len;
  }

  void FuseReadDir(VirtElement* element, fuse_in_header* request) {
    auto in = (fuse_read_in*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_read_in)).address;
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;
    std::string result_buf(in->size, '\0');

    // get result buf length
    uint32_t result_buf_len = 0;

    // set result_buf_pointer to the head of result_buf
    auto result_buf_pointer = result_buf.data();

    if (!fuse_->ReadDirectory(in, request->nodeid, result_buf_pointer, &result_buf_len, request->opcode == FUSE_READDIRPLUS)) {
      fuse_->MakeResponse(response, 0, -errno, request->unique);
      element->length = response->len;
      return;
    }

    // init out header base size
    bzero(response, sizeof(fuse_out_header));
    response->unique = request->unique;
    response->len = sizeof(fuse_out_header);

    // loop
    while (result_buf_len > 0) {
      auto buffer = fuse_->GetDataBufferFromIovec(element->vector, 0);
      if (result_buf_len > buffer.size) {
        memcpy(buffer.address, result_buf_pointer, buffer.size);
        result_buf_len -= buffer.size;
        response->len += buffer.size;
        result_buf_pointer += buffer.size;
      } else {
        memcpy(buffer.address, result_buf_pointer, result_buf_len);
        response->len += result_buf_len;
        break;
      }
    }

    element->length = response->len;
  }

  void FuseUnlink(VirtElement* element, fuse_in_header* request) {
    auto name = (char*)fuse_->GetDataBufferFromIovec(element->vector, request->len - sizeof(fuse_in_header)).address;
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;

    auto parent_fd = fuse_->GetFdFromInode(request->nodeid);
    MV_ASSERT(parent_fd != -1);

    // get file information
    struct stat stat = {0};
    MV_ASSERT(fstatat(parent_fd, name, &stat, 0) != -1);

    // release inode
    auto ret = -1;
    auto inode = fuse_->GetInodeFromStat(&stat);
    if(inode) {
      ret = unlinkat(parent_fd, name, 0);
    }

    // update disk info
    if (!ret) {
      fuse_->ReleaseDiskSpace(stat.st_size);
    }
    fuse_->MakeResponse(response, 0, (ret == 0 ? 0 : -errno), request->unique);
    element->length = response->len;
  }

  /*
  * The inode's lookup count increases by one for every call to lookup. 
  * The nlookup parameter indicates by how much the lookup count should be decreased.
  */
  void FuseForget(VirtElement* element, fuse_in_header* request) {
    auto in = (fuse_forget_in*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_forget_in)).address;
    auto inode = fuse_->GetInode(request->nodeid);
    if(inode) {
      fuse_->UnrefInode(inode, in->nlookup);
      if (debug_) {
        MV_LOG("call forget fd=%d refcount=%d nlookup=%d", inode->fd, inode->refcount, in->nlookup);
      }
    } else {
      MV_ERROR("can't find inode nodeid=%d", request->nodeid);
    }
  }

  void FuseFlush(VirtElement* element, fuse_in_header* request) {
    auto in = (fuse_flush_in*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_flush_in)).address;
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;
    MV_ASSERT(fuse_->GetInodeFromFd(in->fh) != nullptr);
    fuse_->MakeResponse(response, 0, close(dup(in->fh)) == -1 ? -errno : 0, request->unique);
    element->length = response->len;
  }

  void FuseWrite(VirtElement* element, fuse_in_header* request) {
    auto in = (fuse_write_in*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_write_in)).address;

    // check disk space
    if (!fuse_->IsDiskSpaceLeftEnough(in->size)) {
      auto response = (fuse_out_header*)element->vector.back().iov_base;
      fuse_->MakeResponse(response, 0, -28, request->unique);
      element->length = response->len;
      return;
    }

    // check the fd from vm is valid
    auto inode = fuse_->GetInodeFromFd(in->fh);
    if (inode == nullptr) {
      auto response = (fuse_out_header*)element->vector.back().iov_base;
      fuse_->MakeResponse(response, 0, -2, request->unique);
      element->length = response->len;
      return;
    }
    
    auto remain = in->size;
    off_t offset = in->offset;
    while (remain > 0) {
      auto write_in_data_buffer = fuse_->GetDataBufferFromIovec(element->vector, 0);
      auto write_size = std::min((size_t)remain, write_in_data_buffer.size);

      auto ret = pwrite(in->fh, write_in_data_buffer.address, write_size, offset);
      if (ret != (ssize_t )write_size) {
        MV_PANIC("failed to write count=%lu ret=%d", write_size, ret);
      }

      remain -= ret;
      offset += ret;
    }
    
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;
    auto out = (fuse_write_out*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_write_out)).address;

    bzero(out, sizeof(fuse_write_out));
    out->size = in->size - remain;
    fuse_->CostDiskSpace(out->size);
    fuse_->MakeResponse(response, sizeof(fuse_write_out), 0, request->unique);
    element->length = response->len;
  }

  void FuseRead(VirtElement* element, fuse_in_header* request) {
    auto in = (fuse_read_in*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_read_in)).address;
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;
    fuse_->MakeResponse(response, 0, 0, request->unique);

    auto remain = in->size;
    off_t offset = in->offset;
    while (remain > 0) {
      auto read_in_data_buffer = fuse_->GetDataBufferFromIovec(element->vector, 0);
      auto read_size = std::min((size_t)remain, read_in_data_buffer.size);

      auto ret = pread(in->fh, read_in_data_buffer.address, read_size, offset);
      if (ret < 0) {
        response->error = -errno;
        MV_ERROR("failed to read count=%lu error=%d", read_size, -errno);
        break;
      } else {
        remain -= ret;
        offset += ret;
        if (ret < (ssize_t)read_size) {
          // read finish
          break;
        }
      }
    }

    response->len += in->size - remain;
    element->length = response->len;
  }

  void FuseSetAttribute(VirtElement* element, fuse_in_header* request) {
    auto in = (fuse_setattr_in*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_setattr_in)).address;

    struct stat stat = {0};
    stat.st_mode = in->mode;
    stat.st_uid = in->uid;
    stat.st_gid = in->gid;
    stat.st_size = in->size;
    stat.st_atim.tv_sec = in->atime;
    stat.st_mtim.tv_sec = in->mtime;
    stat.st_ctim.tv_sec = in->ctime;
    stat.st_atim.tv_nsec = in->atimensec;
    stat.st_mtim.tv_nsec = in->mtimensec;
    stat.st_ctim.tv_nsec = in->ctimensec;

    if (in->valid & FATTR_FH) {
      in->valid &= ~FATTR_FH;
    }
    in->valid &= FUSE_SET_ATTR_MODE | FUSE_SET_ATTR_UID | FUSE_SET_ATTR_GID | FUSE_SET_ATTR_SIZE | FUSE_SET_ATTR_ATIME |
                 FUSE_SET_ATTR_MTIME | FUSE_SET_ATTR_ATIME_NOW | FUSE_SET_ATTR_MTIME_NOW | FUSE_SET_ATTR_CTIME;

    int fd = fuse_->GetFdFromInode(request->nodeid);
    MV_ASSERT(fd != -1);

    int ret = -1;
    char procname[64] = {0};
    if (in->valid & FUSE_SET_ATTR_MODE) {
      if (in->fh) {
        ret = fchmod(in->fh, stat.st_mode);
      } else {
        sprintf(procname, "/proc/self/fd/%i", fd);
        ret = chmod(procname, stat.st_mode);
      }
      MV_ASSERT(ret != -1);
    }
    if (in->valid & (FUSE_SET_ATTR_UID | FUSE_SET_ATTR_GID)) {
      uid_t uid = (in->valid & FUSE_SET_ATTR_UID) ? stat.st_uid : (uid_t)-1;
      gid_t gid = (in->valid & FUSE_SET_ATTR_GID) ? stat.st_gid : (gid_t)-1;

      ret = fchownat(fd, "", uid, gid, AT_EMPTY_PATH | AT_SYMLINK_NOFOLLOW);
      MV_ASSERT(ret != -1);
    }
    if (in->valid & FUSE_SET_ATTR_SIZE) {
      if (in->fh) {
        ret = ftruncate(in->fh, stat.st_size);
      } else {
        sprintf(procname, "/proc/self/fd/%i", fd);
        ret = truncate(procname, stat.st_size);
      }
      MV_ASSERT(ret != -1);
    }
    if (in->valid & (FUSE_SET_ATTR_ATIME | FUSE_SET_ATTR_MTIME)) {
      struct timespec tv[2] = {0};
      tv[0].tv_sec = 0;
      tv[1].tv_sec = 0;
      tv[0].tv_nsec = UTIME_OMIT;
      tv[1].tv_nsec = UTIME_OMIT;

      if (in->valid & FUSE_SET_ATTR_ATIME_NOW)
        tv[0].tv_nsec = UTIME_NOW;
      else if (in->valid & FUSE_SET_ATTR_ATIME)
        tv[0] = stat.st_atim;

      if (in->valid & FUSE_SET_ATTR_MTIME_NOW)
        tv[1].tv_nsec = UTIME_NOW;
      else if (in->valid & FUSE_SET_ATTR_MTIME)
        tv[1] = stat.st_mtim;

      if (in->fh) {
        ret = futimens(in->fh, tv);
      } else {
        sprintf(procname, "/proc/self/fd/%i", fd);
        ret = utimensat(AT_FDCWD, procname, tv, 0);
      }
      MV_ASSERT(ret != -1);
    }
  }

  void FuseGetAttribute(VirtElement* element, fuse_in_header* request) {
    if(request->opcode == FUSE_GETATTR) {
      fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_getattr_in));
    }
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;
    auto out = (fuse_attr_out*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_attr_out)).address;

    int fd = fuse_->GetFdFromInode(request->nodeid);
    MV_ASSERT(fd != -1);

    struct stat stat = {0};
    MV_ASSERT(fstatat(fd, "", &stat, AT_EMPTY_PATH | AT_SYMLINK_NOFOLLOW) != -1);

    auto user_config = fuse_->user_config();
    out->attr_valid = fuse_->CalcTimeoutSecond(user_config.timeout);
    out->attr_valid_nsec = fuse_->CalcTimeoutNsecond(user_config.timeout);
    fuse_->ConvertStatToFuseAttr(&stat, &out->attr);
    fuse_->MakeResponse(response, sizeof(fuse_attr_out), 0, request->unique);
    element->length = response->len;
  }

  void FuseFallocate(VirtElement* element, fuse_in_header* request) {
    auto in = (fuse_fallocate_in*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_fallocate_in)).address;
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;
    auto ret = fallocate(in->fh, FALLOC_FL_KEEP_SIZE, in->offset, in->length);
    fuse_->MakeResponse(response, 0, ret == -1 ? -errno : 0, request->unique);
    element->length = response->len;
  }

  void FuseCreate(VirtElement* element, fuse_in_header* request) {
    auto in = (fuse_create_in*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_create_in)).address;
    auto name = (char*)fuse_->GetDataBufferFromIovec(element->vector, request->len - sizeof(fuse_in_header) - sizeof(fuse_create_in)).address;
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;
    auto entry_out_arg = (fuse_entry_out*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_entry_out)).address;
    auto open_out_arg = (fuse_open_out*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_open_out)).address;

    if (fuse_->IsInodeListFull()) {
      fuse_->MakeResponse(response, 0, -24, request->unique);
      element->length = response->len;
      return;
    }

    auto parent_fd = fuse_->GetFdFromInode(request->nodeid);
    MV_ASSERT(parent_fd != -1);

    // create new empty file
    int fd = openat(parent_fd, name, (in->flags | O_CREAT) & ~O_NOFOLLOW, in->mode);
    if (fd == -1) {
      fuse_->MakeResponse(response, 0, -errno, request->unique);
      element->length = response->len;
      return;
    }

    // close fd after new empty file is created
    close(fd);

    // create new inode
    struct fuse_entry_param entry_param = {0};
    if (!fuse_->Lookup(request->nodeid, name, &entry_param)) {
      fuse_->MakeResponse(response, 0, -errno, request->unique);
      element->length = response->len;
      return;
    }

    entry_out_arg->nodeid = (uintptr_t)entry_param.ino;
    entry_out_arg->generation = entry_param.generation;
    entry_out_arg->entry_valid = fuse_->CalcTimeoutSecond(entry_param.entry_timeout);
    entry_out_arg->entry_valid_nsec = fuse_->CalcTimeoutNsecond(entry_param.entry_timeout);
    entry_out_arg->attr_valid = fuse_->CalcTimeoutSecond(entry_param.attr_timeout);
    entry_out_arg->attr_valid_nsec = fuse_->CalcTimeoutNsecond(entry_param.attr_timeout);
    fuse_->ConvertStatToFuseAttr(&entry_param.attr, &entry_out_arg->attr);

    auto out_fd = fuse_->GetFdFromInode(entry_param.ino);
    MV_ASSERT(out_fd != -1);
    open_out_arg->fh = out_fd;

    fuse_->MakeResponse(response, sizeof(fuse_entry_out) + sizeof(fuse_open_out), 0, request->unique);
    element->length = response->len;
  }

  void FuseRemoveDir(VirtElement* element, fuse_in_header* request) {
    auto name = (char*)fuse_->GetDataBufferFromIovec(element->vector, request->len - sizeof(fuse_in_header)).address;
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;

    auto parent_fd = fuse_->GetFdFromInode(request->nodeid);
    MV_ASSERT(parent_fd != -1);

    struct stat stat = {0};
    MV_ASSERT(fstatat(parent_fd, name, &stat, 0) != -1);

    auto ret = unlinkat(parent_fd, name, AT_REMOVEDIR);
    fuse_->MakeResponse(response, 0, ret == -1 ? -errno : 0, request->unique);
    element->length = response->len;
  }

  void FuseGetXAttr(VirtElement* element, fuse_in_header* request) {
    auto in = (fuse_getxattr_in*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_getxattr_in)).address;
    auto name = (char*)fuse_->GetDataBufferFromIovec(element->vector, request->len - sizeof(fuse_in_header) - sizeof(fuse_getxattr_in)).address;
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;
    auto out = (fuse_getxattr_out*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_getxattr_out)).address;

    char procname[64] = {0};
    auto fd = fuse_->GetFdFromInode(request->nodeid);
    MV_ASSERT(fd != -1);
    sprintf(procname, "/proc/self/fd/%i", fd);

    ssize_t ret = 0;
    if (in->size) {
      auto out_buf = fuse_->GetDataBufferFromIovec(element->vector, 0);
      ret = getxattr(procname, name, out_buf.address, out_buf.size);
    } else {
      ret = getxattr(procname, name, nullptr, 0);
    }
    
    if(ret == -1) {
      fuse_->MakeResponse(response, 0, -errno, request->unique);
    } else {
      out->size = ret;
      out->padding = 0;
      fuse_->MakeResponse(response, ret + sizeof(fuse_getxattr_out), 0, request->unique);
    }
    element->length = response->len;
  } 

  void FuseSetXAttr(VirtElement* element, fuse_in_header* request) {
    auto in = (fuse_setxattr_in*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_setxattr_in)).address;
    auto name_value = (char*)fuse_->GetDataBufferFromIovec(element->vector, request->len - sizeof(fuse_in_header) - sizeof(fuse_setxattr_in)).address;
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;

	  char procname[64] = {0};
    auto fd = fuse_->GetFdFromInode(request->nodeid);
    MV_ASSERT(fd != -1);
    sprintf(procname, "/proc/self/fd/%i", fd);

    auto name = name_value;
    auto value = name_value + strlen(name) + 1;
	  auto ret = setxattr(procname, name, value, in->size, in->flags);
    fuse_->MakeResponse(response, 0, ret == -1 ? -errno : 0, request->unique);
    element->length = response->len;
  }
  
  void FuseFSync(VirtElement* element, fuse_in_header* request) {
    auto in = (fuse_fsync_in*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_fsync_in)).address;
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;

    int ret = -1;
    if(in->fsync_flags & 1) {
      ret = fdatasync(in->fh);
    } else {
      ret = fsync(in->fh);
    }

    fuse_->MakeResponse(response, 0, ret == -1 ? -errno : 0, request->unique);
    element->length = response->len;
  }

  void FuseRename2(VirtElement* element, fuse_in_header* request) {
    auto in = (fuse_rename2_in*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_rename2_in)).address;
    auto old_name = (char*)fuse_->GetDataBufferFromIovec(element->vector, request->len - sizeof(fuse_in_header) - sizeof(fuse_rename2_in)).address;
    auto new_name = old_name + strlen(old_name) + 1;
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;

    auto old_fd = fuse_->GetFdFromInode(request->nodeid);
    MV_ASSERT(old_fd != -1);
    auto new_fd = fuse_->GetFdFromInode(in->newdir);
    MV_ASSERT(new_fd != -1);

    auto ret = renameat(old_fd, old_name, new_fd, new_name);
    fuse_->MakeResponse(response, 0, ret == -1 ? -errno : 0, request->unique);
    element->length = response->len;
  }

  void FuseDestroy(VirtElement* element, fuse_in_header* request) {
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;

    // free dir handle
    ClearDirectoryPointerSet();

    // free inode list
    fuse_->ClearInodeList(false);

    fuse_->MakeResponse(response, 0, 0, request->unique);
    element->length = response->len;
  }

  void FuseMakeDir(VirtElement* element, fuse_in_header* request) {
    auto in = (fuse_mkdir_in*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_mkdir_in)).address;
    auto name = (char*)fuse_->GetDataBufferFromIovec(element->vector, request->len - sizeof(fuse_in_header) - sizeof(fuse_mkdir_in)).address;
    auto response = (fuse_out_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_out_header)).address;
    auto out = (fuse_entry_out*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_entry_out)).address;

    if (fuse_->IsInodeListFull()) {
      fuse_->MakeResponse(response, 0, -24, request->unique);
      element->length = response->len;
      return;
    }

    struct fuse_entry_param entry_param = {0};
    auto parent_fd = fuse_->GetFdFromInode(request->nodeid);
    MV_ASSERT(parent_fd != -1);

    auto ret = mkdirat(parent_fd, name, S_IFDIR | in->mode);
    if (ret != 0 || !fuse_->Lookup(request->nodeid, name, &entry_param)) {
      fuse_->MakeResponse(response, 0, -errno, request->unique);
      element->length = response->len;
      return;
    }

    bzero(out, sizeof(fuse_entry_out));
    out->nodeid = entry_param.ino;
    out->generation = entry_param.generation;
    out->entry_valid = fuse_->CalcTimeoutSecond(entry_param.entry_timeout);
    out->entry_valid_nsec = fuse_->CalcTimeoutNsecond(entry_param.entry_timeout);
    out->attr_valid = fuse_->CalcTimeoutSecond(entry_param.attr_timeout);
    out->attr_valid_nsec = fuse_->CalcTimeoutNsecond(entry_param.attr_timeout);
    fuse_->ConvertStatToFuseAttr(&entry_param.attr, &out->attr);
    fuse_->MakeResponse(response, sizeof(fuse_entry_out), 0, request->unique);
    element->length = response->len;
  }

  void HandleCommand(VirtElement* element) {
    auto request = (fuse_in_header*)fuse_->GetDataBufferFromIovec(element->vector, sizeof(fuse_in_header)).address;

    if (debug_) {
      MV_LOG(
          "HandleCommand len=%d gid=%d opcode=%d padding=%d pid=%d uid=%d nodeid=%d unique=%d",
          request->len, request->gid, request->opcode, request->padding, request->pid, request->uid,
          request->nodeid, request->unique);
    }

    // handle command
    switch (request->opcode) {
      case FUSE_INIT:
        FuseInit(element, request);
        break;
      case FUSE_STATFS:
        FuseStatFs(element, request);
        break;
      case FUSE_LOOKUP:
        FuseLookup(element, request);
        break;
      case FUSE_READLINK:
        FuseReadLink(element, request);
        break;
      case FUSE_OPEN:
        FuseOpen(element, request);
        break;
      case FUSE_RELEASE:
        FuseRelease(element, request);
        break;
      case FUSE_OPENDIR:
        FuseOpenDir(element, request);
        break;
      case FUSE_RELEASEDIR:
        FuseReleaseDir(element, request);
        break;
      case FUSE_READDIR:
      case FUSE_READDIRPLUS:
        FuseReadDir(element, request);
        break;
      case FUSE_READ:
        FuseRead(element, request);
        break;
      case FUSE_UNLINK:
        FuseUnlink(element, request);
        NotifyVirtioFs();
        break;
      case FUSE_FORGET:
        FuseForget(element, request);
        NotifyVirtioFs();
        break;
      case FUSE_FLUSH:
        FuseFlush(element, request);
        break;
      case FUSE_WRITE:
        FuseWrite(element, request);
        break;
      case FUSE_SETATTR:
        FuseSetAttribute(element, request);
        NotifyVirtioFs();
        /* [[fallthrough]] */
      case FUSE_GETATTR:
        FuseGetAttribute(element, request);
        break;
      case FUSE_FALLOCATE:
        FuseFallocate(element, request);
        break;
      case FUSE_CREATE:
        FuseCreate(element, request);
        NotifyVirtioFs();
        break;
      case FUSE_RMDIR:
        FuseRemoveDir(element, request);
        NotifyVirtioFs();
        break;
      case FUSE_RENAME:
      case FUSE_RENAME2:
        FuseRename2(element, request);
        NotifyVirtioFs();
        break;
      case FUSE_MKDIR:
        FuseMakeDir(element, request);
        NotifyVirtioFs();
        break;
      case FUSE_GETXATTR:
        FuseGetXAttr(element, request);
        break;
      case FUSE_SETXATTR:
        FuseSetXAttr(element, request);
        break;
      case FUSE_FSYNC:
        FuseFSync(element, request);
        break;
      case FUSE_DESTROY:
        FuseDestroy(element, request);
        break;
      default:
        MV_ERROR("no implement opcode=%d", request->opcode);
        auto response = (fuse_out_header*)element->vector.front().iov_base;
        fuse_->MakeResponse(response, 0, -EACCES, request->unique);
        element->length = response->len;
        break;
    }
  }
};

DECLARE_DEVICE(VirtioFs);
