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

      std::string name;
      if (has_key("disk_name")) {
        name = std::get<std::string>(key_values_["disk_name"]);
      } else {
        name = "mvisor-fs";
      }
      strncpy((char*)fs_config_.tag, name.c_str(), sizeof(fs_config_.tag) - 1);

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

    for (uint32_t i = 0; i < fs_config_.num_request_queues; ++i) {
      AddQueue(DEFAULT_QUEUE_SIZE, std::bind(&VirtioFs::OnOutput, this, i));
    }

    // free dir handle
    for (auto dirp_pointer : dirp_pointer_set_) {
      closedir(((lo_dirp*)dirp_pointer)->dp);
      free((void*)dirp_pointer);
    }
    dirp_pointer_set_.clear();
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

  void FuseInit(VirtElement* element, fuse_in_header* request) {
    auto& vector = element->vector;
    auto response = (fuse_out_header*)vector.front().iov_base;
    auto out = (fuse_init_out*)OFFSET(response);
    vector.pop_front();

    // set out params
    out->major = FUSE_KERNEL_VERSION;
    out->minor = FUSE_KERNEL_MINOR_VERSION;
    out->flags |= FUSE_BIG_WRITES | FUSE_DO_READDIRPLUS;
    out->max_readahead = UINT32_MAX;
    out->max_write = UINT32_MAX;
    fuse_->MakeResponse(response, sizeof(fuse_init_out), 0, request->unique);
    element->length = response->len;
  }

  void FuseStatfs(VirtElement* element, fuse_in_header* request) {
    auto& vector = element->vector;
    auto response = (fuse_out_header*)vector.front().iov_base;
    vector.pop_front();

    int fd = fuse_->GetFdFromInode(request->nodeid);
    MV_ASSERT(fd != -1);

    struct statvfs new_stat_vfs = {0};
    MV_ASSERT(fstatvfs(fd, &new_stat_vfs) != -1);

    // modify disk information to vm
    fuse_->ModifyDiskInformationToVm(&new_stat_vfs);

    auto out = (fuse_statfs_out*)OFFSET(response);
    fuse_->ConvertStatfs(&new_stat_vfs, &out->st);
    fuse_->MakeResponse(response, sizeof(fuse_statfs_out), 0, request->unique);
    element->length = response->len;
  }

  void FuseLookup(VirtElement* element, fuse_in_header* request) {
    auto& vector = element->vector;
    auto response = (fuse_out_header*)vector.front().iov_base;
    vector.pop_front();
    auto name = (const char*)OFFSET(request);
    struct fuse_entry_param entry_param = {0};
    if (!fuse_->LowLevelLookup(request->nodeid, name, &entry_param)) {
      fuse_->MakeResponse(response, 0, -errno, request->unique);
      element->length = response->len;
      return;
    } 

    auto out = (fuse_entry_out*)OFFSET(response);
    out->nodeid = entry_param.ino;
    out->generation = entry_param.generation;
    out->entry_valid = fuse_->CalcTimeoutSecond(entry_param.entry_timeout);
    out->entry_valid_nsec = fuse_->CalcTimeoutNsecond(entry_param.entry_timeout);
    out->attr_valid = fuse_->CalcTimeoutSecond(entry_param.attr_timeout);
    out->attr_valid_nsec = fuse_->CalcTimeoutNsecond(entry_param.attr_timeout);
    fuse_->ConvertStat(&entry_param.attr, &out->attr);
    fuse_->MakeResponse(response, sizeof(fuse_entry_out), 0, request->unique);
    element->length = response->len;
  }

  void FuseOpen(VirtElement* element, fuse_in_header* request) {
    auto& vector = element->vector;
    auto response = (fuse_out_header*)vector.front().iov_base;
    vector.pop_front();

    int fd = fuse_->GetFdFromInode(request->nodeid);
    if (fd == -1) {
      fuse_->MakeResponse(response, 0, -errno, request->unique);
      element->length = response->len;
      return;
    }

    // add reference count
    ((lo_inode*)request->nodeid)->refcount++;

    fuse_open_out* out = (fuse_open_out*)OFFSET(response);
    out->fh = fd;
    fuse_->MakeResponse(response, sizeof(fuse_open_out), 0, request->unique);
    element->length = response->len;
  }

  void FuseReadLink(VirtElement* element, fuse_in_header* request) {
    auto& vector = element->vector;
    auto response = (fuse_out_header*)vector.front().iov_base;
    auto iov_len = vector.back().iov_len;
    vector.pop_front();
    auto out = (char*)OFFSET(response);

    int fd = fuse_->GetFdFromInode(request->nodeid);
    MV_ASSERT(fd != -1);

    auto ret = readlinkat(fd, "", out, iov_len);
    MV_ASSERT(ret != -1 && ret <= (ssize_t)iov_len);
    fuse_->MakeResponse(response, ret, 0, request->unique);
    element->length = response->len;
  }

  void FuseRelease(VirtElement* element, fuse_in_header* request) {
    auto& vector = element->vector;
    auto response = (fuse_out_header*)vector.front().iov_base;
    vector.pop_front();

    auto in = (fuse_release_in*)OFFSET(request);
    auto inode = fuse_->GetInodeFromFd(in->fh);
    if (inode != nullptr) {
      fuse_->LowLevelForget(inode, 1);
    }
    fuse_->MakeResponse(response, 0, 0, request->unique);
    element->length = response->len;
  }

  void FuseOpenDir(VirtElement* element, fuse_in_header* request) {
    auto& vector = element->vector;
    auto response = (fuse_out_header*)vector.front().iov_base;
    vector.pop_front();

    int fd = fuse_->GetFdFromInode(request->nodeid);
    if (fd == -1) {
      fuse_->MakeResponse(response, 0, -errno, request->unique);
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

    auto dirp = (struct lo_dirp*)malloc(sizeof(struct lo_dirp));
    MV_ASSERT(dirp != nullptr);
    dirp->dp = fdopendir(fd_copy);
    MV_ASSERT(dirp->dp != nullptr);
    dirp->offset = 0;
    dirp->entry = nullptr;

    // add reference count
    ((lo_inode*)request->nodeid)->refcount++;
    dirp_pointer_set_.insert((uint64_t)dirp);

    auto out = (fuse_open_out*)OFFSET(response);
    out->fh = (uint64_t)dirp;
    fuse_->MakeResponse(response, sizeof(fuse_open_out), 0, request->unique);
    element->length = response->len;
  }

  void FuseReleaseDir(VirtElement* element, fuse_in_header* request) {
    auto& vector = element->vector;
    auto response = (fuse_out_header*)vector.front().iov_base;
    vector.pop_front();

    auto in = (fuse_release_in*)OFFSET(request);
    if (dirp_pointer_set_.erase(in->fh)) {
      auto dirp = (lo_dirp*)in->fh;
      MV_ASSERT(dirp != nullptr);
      closedir(dirp->dp);
      free(dirp);
      fuse_->LowLevelForget((lo_inode*)request->nodeid, 1);
    }

    fuse_->MakeResponse(response, 0, 0, request->unique);
    element->length = response->len;
  }

  void FuseReadDir(VirtElement* element, fuse_in_header* request) {
    auto& vector = element->vector;
    auto in = (fuse_read_in*)OFFSET(request);
    auto response = (fuse_out_header*)vector.front().iov_base;
    std::string result_buf(in->size, '\0');
    uint32_t remain_size = 0;
    if (!fuse_->LowLevelReadDir(request, (char*)result_buf.data(), &remain_size, request->opcode == FUSE_READDIRPLUS)) {
      fuse_->MakeResponse(response, 0, -errno, request->unique);
      element->length = response->len;
      return;
    }

    // set result_buf_pointer to the head of result_buf
    auto result_buf_pointer = result_buf.data();

    // get result buf length
    auto res_len = in->size - remain_size;

    // put result_buf into return vector
    auto tmp_buf_len = vector.front().iov_len - sizeof(fuse_out_header);
    auto tmp_out_buf = OFFSET(response);
    vector.pop_front();

    // init out header base size
    bzero(response, sizeof(fuse_out_header));
    response->unique = request->unique;
    response->len = sizeof(fuse_out_header);

    // loop
    while (res_len > 0) {
      if (res_len > tmp_buf_len) {
        memcpy(tmp_out_buf, result_buf_pointer, tmp_buf_len);
        res_len -= tmp_buf_len;
        response->len += tmp_buf_len;
        result_buf_pointer += tmp_buf_len;
      } else {
        memcpy(tmp_out_buf, result_buf_pointer, res_len);
        response->len += res_len;
        break;
      }

      // impossible to be empty here
      MV_ASSERT(!vector.empty());

      tmp_out_buf = (char*)vector.front().iov_base;
      tmp_buf_len = vector.front().iov_len;
      vector.pop_front();
    }

    element->length = response->len;
  }

  void FuseRead(VirtElement* element, fuse_in_header* request) {
    auto& vector = element->vector;
    auto in = (fuse_read_in*)OFFSET(request);
    uint32_t read_in_size = in->size;
    MV_ASSERT(read_in_size > 0);

    // init src_buf from file fd
    struct fuse_bufvec src_buf = FUSE_BUFVEC_INIT(read_in_size);
    src_buf.buf[0].flags = fuse_buf_flags(FUSE_BUF_IS_FD | FUSE_BUF_FD_SEEK);
    src_buf.buf[0].fd = in->fh;
    src_buf.buf[0].pos = in->offset;

    // read operation only have one fuse_out_header!
    auto read_out = (fuse_out_header*)vector.front().iov_base;
    fuse_->MakeResponse(read_out, 0, 0, request->unique);

    // read file loop
    auto out_buf = (uint8_t*)read_out + sizeof(fuse_out_header);
    uint32_t out_buf_len = vector.front().iov_len - sizeof(fuse_out_header);
    while (vector.size() > 0) {
      struct fuse_bufvec dst_mem_buf = FUSE_BUFVEC_INIT(out_buf_len);
      dst_mem_buf.buf[0].mem = out_buf;
      bzero(out_buf, out_buf_len);

      // copy bytes from file to dst_mem_buf
      auto ret = fuse_->FuseBufCopy(&dst_mem_buf, &src_buf, (fuse_buf_copy_flags)0);
      if (ret < 0) {
        read_out->error = -errno;
        break;
      }

      // finish read
      if (ret == 0) {
        break;
      }

      read_in_size -= ret;
      read_out->len += ret;
      vector.pop_front();

      // finish read
      if (read_in_size == 0) {
        break;
      }

      // next
      if (!vector.empty()) {
        out_buf_len = vector.front().iov_len;
        out_buf = (uint8_t*)vector.front().iov_base;
      }
    }

    element->length += read_out->len;
  }

  void FuseUnlink(VirtElement* element, fuse_in_header* request) {
    auto& vector = element->vector;
    fuse_out_header* response = (fuse_out_header*)vector.front().iov_base;
    vector.pop_front();
    auto parent_fd = fuse_->GetFdFromInode(request->nodeid);
    MV_ASSERT(parent_fd != -1);

    // get file information
    struct stat stat = {0};
    auto name = (char*)OFFSET(request);
    MV_ASSERT(fstatat(parent_fd, name, &stat, 0) != -1);

    // release disk space
    fuse_->ReleaseDiskSpace(stat.st_size);

    // release inode
    auto inode = fuse_->LowLevelFind(&stat);
    MV_ASSERT(inode != nullptr);
    fuse_->LowLevelForget(inode, 1);

    // unlink
    auto ret = unlinkat(parent_fd, name, 0);
    fuse_->MakeResponse(response, 0, (ret == -1 ? -errno : 0), request->unique);
    element->length = response->len;
  }

  void FuseForget(VirtElement* element, fuse_in_header* request) {
    auto& vector = element->vector;
    fuse_out_header* response = (fuse_out_header*)vector.front().iov_base;
    vector.pop_front();

    MV_ASSERT(fuse_->GetFdFromInode(request->nodeid) != -1);
    auto* in = (fuse_forget_in*)OFFSET(request);
    fuse_->LowLevelForget((lo_inode*)request->nodeid, in->nlookup);
    fuse_->MakeResponse(response, 0, 0, request->unique);
    element->length = response->len;
  }

  void FuseFlush(VirtElement* element, fuse_in_header* request) {
    auto& vector = element->vector;
    auto response = (fuse_out_header*)vector.front().iov_base;
    vector.pop_front();

    auto in = (fuse_flush_in*)OFFSET(request);
    auto inode = fuse_->GetInodeFromFd(in->fh);
    MV_ASSERT(inode != nullptr);

    auto ret = close(dup(inode->fd));
    fuse_->MakeResponse(response, 0, ret == -1 ? -errno : 0, request->unique);
    element->length = response->len;
  }

  void FuseWrite(VirtElement* element, fuse_in_header* request) {
    auto& vector = element->vector;
    auto response = (fuse_out_header*)vector.back().iov_base;
    vector.pop_back();

    auto in = (fuse_write_in*)OFFSET(request);
    uint64_t in_byte_buf_len = in->size;
    if (!fuse_->IsDiskSpaceLeftEnough(in_byte_buf_len)) {
      fuse_->MakeResponse(response, 0, -28, request->unique);
      element->length = response->len;
      return;
    }

    // check the fd from vm is valid
    auto inode = fuse_->GetInodeFromFd(in->fh);
    if (inode == nullptr) {
      fuse_->MakeResponse(response, 0, -2, request->unique);
      element->length = response->len;
    }

    struct fuse_bufvec out_buf = FUSE_BUFVEC_INIT(in_byte_buf_len);
    out_buf.buf[0].flags = fuse_buf_flags(FUSE_BUF_IS_FD | FUSE_BUF_FD_SEEK);
    out_buf.buf[0].fd = in->fh;
    out_buf.buf[0].pos = in->offset;

    auto in_byte_buf = (char*)((uint8_t*)in + sizeof(fuse_write_in));
    uint64_t iov_len = request->len - sizeof(fuse_in_header) - sizeof(fuse_write_in);
    while (in_byte_buf_len > 0) {
      struct fuse_bufvec in_buf = FUSE_BUFVEC_INIT(MIN(in_byte_buf_len, iov_len));
      in_buf.buf[0].mem = in_byte_buf;

      auto ret = fuse_->FuseBufCopy(&out_buf, &in_buf, (fuse_buf_copy_flags)0);
      MV_ASSERT(ret == (ssize_t)in_buf.buf[0].size);

      out_buf.buf[0].pos += ret;
      in_byte_buf_len -= ret;
      if (in_byte_buf_len == 0) {
        break;
      }

      MV_ASSERT(!vector.empty());
      in_byte_buf = (char*)vector.front().iov_base;
      iov_len = vector.front().iov_len;
      vector.pop_front();
    }

    auto out = (fuse_write_out*)OFFSET(response);
    bzero(out, sizeof(fuse_write_out));
    out->size = in->size - in_byte_buf_len;
    fuse_->CostDiskSpace(out->size);
    fuse_->MakeResponse(response, sizeof(fuse_write_out), 0, request->unique);
    element->length = response->len;
  }

  void FuseSetAttribute(fuse_in_header* request) {
    auto in = (fuse_setattr_in*)OFFSET(request);
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
      struct timespec tv[2];

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
    auto& vector = element->vector;
    auto response = (fuse_out_header*)vector.front().iov_base;
    vector.pop_front();

    int fd = fuse_->GetFdFromInode(request->nodeid);
    MV_ASSERT(fd != -1);

    struct stat stat = {0};
    MV_ASSERT(fstatat(fd, "", &stat, AT_EMPTY_PATH | AT_SYMLINK_NOFOLLOW) != -1);

    auto lo_data = fuse_->get_lo_data();
    auto out = (struct fuse_attr_out*)((uint8_t*)response + sizeof(fuse_out_header));
    out->attr_valid = fuse_->CalcTimeoutSecond(lo_data->timeout);
    out->attr_valid_nsec = fuse_->CalcTimeoutNsecond(lo_data->timeout);
    fuse_->ConvertStat(&stat, &out->attr);
    fuse_->MakeResponse(response, sizeof(fuse_attr_out), 0, request->unique);
    element->length = response->len;
  }

  void FuseFallocate(VirtElement* element, fuse_in_header* request) {
    auto& vector = element->vector;
    auto response = (fuse_out_header*)vector.front().iov_base;
    vector.pop_front();

    auto in = (fuse_fallocate_in*)OFFSET(request);
    int ret = fallocate(in->fh, FALLOC_FL_KEEP_SIZE, in->offset, in->length);
    fuse_->MakeResponse(response, 0, ret == -1 ? -errno : 0, request->unique);
    element->length = response->len;
  }

  void FuseCreate(VirtElement* element, fuse_in_header* request) {
    auto& vector = element->vector;
    auto response = (fuse_out_header*)vector.back().iov_base;
    vector.pop_back();

    if (fuse_->IsInodeListFull()) {
      fuse_->MakeResponse(response, 0, -24, request->unique);
      element->length = response->len;
      return;
    }

    auto parent_fd = fuse_->GetFdFromInode(request->nodeid);
    MV_ASSERT(parent_fd != -1);

    auto in = (fuse_create_in*)OFFSET(request);
    auto name = OFFSET(in);

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
    if (!fuse_->LowLevelLookup(request->nodeid, name, &entry_param)) {
      fuse_->MakeResponse(response, 0, -errno, request->unique);
      element->length = response->len;
      return;
    }

    auto entry_out_arg = (struct fuse_entry_out*)OFFSET(response);
    entry_out_arg->nodeid = (uintptr_t)entry_param.ino;
    entry_out_arg->generation = entry_param.generation;
    entry_out_arg->entry_valid = fuse_->CalcTimeoutSecond(entry_param.entry_timeout);
    entry_out_arg->entry_valid_nsec = fuse_->CalcTimeoutNsecond(entry_param.entry_timeout);
    entry_out_arg->attr_valid = fuse_->CalcTimeoutSecond(entry_param.attr_timeout);
    entry_out_arg->attr_valid_nsec = fuse_->CalcTimeoutNsecond(entry_param.attr_timeout);
    fuse_->ConvertStat(&entry_param.attr, &entry_out_arg->attr);

    auto open_out_arg = (struct fuse_open_out*)OFFSET(entry_out_arg);
    open_out_arg->fh = ((lo_inode*)entry_param.ino)->fd;

    fuse_->MakeResponse(response, sizeof(fuse_entry_out) + sizeof(fuse_open_out), 0, request->unique);
    element->length = response->len;
  }

  void FuseRemoveDir(VirtElement* element, fuse_in_header* request) {
    auto& vector = element->vector;
    auto response = (fuse_out_header*)vector.front().iov_base;
    vector.pop_front();

    auto ret = -1;
    auto parent_fd = fuse_->GetFdFromInode(request->nodeid);
    if (parent_fd != -1) {
      auto name = (const char*)OFFSET(request);

      struct stat stat = {0};
      MV_ASSERT(fstatat(parent_fd, name, &stat, 0) != -1);

      auto inode = fuse_->LowLevelFind(&stat);
      MV_ASSERT(inode != nullptr);
      fuse_->LowLevelForget(inode, 1);

      ret = unlinkat(parent_fd, name, AT_REMOVEDIR);
    }

    fuse_->MakeResponse(response, 0, ret == -1 ? -errno : 0, request->unique);
    element->length = response->len;
  }

  void FuseRename2(VirtElement* element, fuse_in_header* request) {
    auto& vector = element->vector;
    auto in = (fuse_rename2_in*)OFFSET(request);
    auto old_name = (char*)((uint8_t*)in + sizeof(fuse_rename2_in));
    auto new_name = old_name + strlen(old_name) + 1;

    auto old_fd = fuse_->GetFdFromInode(request->nodeid);
    MV_ASSERT(old_fd != -1);
    auto new_fd = fuse_->GetFdFromInode(in->newdir);
    MV_ASSERT(new_fd != -1);

    auto ret = renameat(old_fd, old_name, new_fd, new_name);
    auto response = (fuse_out_header*)vector.front().iov_base;
    vector.pop_front();
    fuse_->MakeResponse(response, 0, ret == -1 ? -errno : 0, request->unique);
    element->length = response->len;
  }

  void FuseMakeDir(VirtElement* element, fuse_in_header* request) {
    auto& vector = element->vector;
    auto response = (fuse_out_header*)vector.front().iov_base;
    vector.pop_front();

    if (fuse_->IsInodeListFull()) {
      fuse_->MakeResponse(response, 0, -24, request->unique);
      element->length = response->len;
      return;
    }

    struct fuse_entry_param entry_param = {0};
    auto parent_fd = fuse_->GetFdFromInode(request->nodeid);
    MV_ASSERT(parent_fd != -1);

    auto in = (fuse_mkdir_in*)OFFSET(request);
    auto name = (char*)((uint8_t*)in + sizeof(fuse_mkdir_in));
    auto ret = fuse_->MakeNodeWrapper(parent_fd, name, NULL, S_IFDIR | in->mode, 0);
    if (ret != 0 || !fuse_->LowLevelLookup(request->nodeid, name, &entry_param)) {
      fuse_->MakeResponse(response, 0, -errno, request->unique);
      element->length = response->len;
      return;
    }

    auto out = (struct fuse_entry_out*)OFFSET(response);
    bzero(out, sizeof(fuse_entry_out));
    out->nodeid = entry_param.ino;
    out->generation = entry_param.generation;
    out->entry_valid = fuse_->CalcTimeoutSecond(entry_param.entry_timeout);
    out->entry_valid_nsec = fuse_->CalcTimeoutNsecond(entry_param.entry_timeout);
    out->attr_valid = fuse_->CalcTimeoutSecond(entry_param.attr_timeout);
    out->attr_valid_nsec = fuse_->CalcTimeoutNsecond(entry_param.attr_timeout);
    fuse_->ConvertStat(&entry_param.attr, &out->attr);
    fuse_->MakeResponse(response, sizeof(fuse_entry_out), 0, request->unique);
    element->length = response->len;
  }

  void HandleCommand(VirtElement* element) {
    // get request from guest
    auto& vector = element->vector;
    auto request = (fuse_in_header*)vector.front().iov_base;
    vector.pop_front();

    if (debug_) {
      MV_LOG(
          "HandleCommand len=%d gid=%d opcode=%d padding=%d pid=%d uid=%d "
          "nodeid=%d "
          "unique=%d",
          request->len, request->gid, request->opcode, request->padding, request->pid, request->uid, request->nodeid,
          request->unique);
    }

    // handle command
    switch (request->opcode) {
      case FUSE_INIT: {
        FuseInit(element, request);
        break;
      }

      case FUSE_STATFS: {
        FuseStatfs(element, request);
        break;
      }

      case FUSE_LOOKUP: {
        FuseLookup(element, request);
        break;
      }

      case FUSE_READLINK: {
        FuseReadLink(element, request);
        break;
      }

      case FUSE_OPEN: {
        FuseOpen(element, request);
        break;
      }

      case FUSE_RELEASE: {
        FuseRelease(element, request);
        break;
      }

      case FUSE_OPENDIR: {
        FuseOpenDir(element, request);
        break;
      }

      case FUSE_RELEASEDIR: {
        FuseReleaseDir(element, request);
        break;
      }

      case FUSE_READDIR:
      case FUSE_READDIRPLUS: {
        FuseReadDir(element, request);
        break;
      }

      case FUSE_READ: {
        FuseRead(element, request);
        break;
      }

      case FUSE_UNLINK: {
        FuseUnlink(element, request);
        NotifyVirtioFs();
        break;
      }

      case FUSE_FORGET: {
        FuseForget(element, request);
        NotifyVirtioFs();
        break;
      }

      case FUSE_FLUSH: {
        FuseFlush(element, request);
        break;
      }

      case FUSE_WRITE: {
        FuseWrite(element, request);
        NotifyVirtioFs();
        break;
      }

      case FUSE_SETATTR: {
        FuseSetAttribute(request);
        // no break go on to FUSE_GETATTR
      }

      case FUSE_GETATTR: {
        FuseGetAttribute(element, request);
        break;
      }

      case FUSE_FALLOCATE: {
        FuseFallocate(element, request);
        break;
      }

      case FUSE_CREATE: {
        FuseCreate(element, request);
        NotifyVirtioFs();
        break;
      }

      case FUSE_RMDIR: {
        FuseRemoveDir(element, request);
        NotifyVirtioFs();
        break;
      }

      case FUSE_RENAME2: {
        FuseRename2(element, request);
        NotifyVirtioFs();
        break;
      }

      case FUSE_MKDIR: {
        FuseMakeDir(element, request);
        NotifyVirtioFs();
        break;
      }

      default:
        MV_LOG("no implement opcode=%d", request->opcode);
        auto response = (fuse_out_header*)vector.front().iov_base;
        fuse_->MakeResponse(response, 0, -EACCES, request->unique);
        element->length = response->len;
        break;
    }
  }
};

DECLARE_DEVICE(VirtioFs);
