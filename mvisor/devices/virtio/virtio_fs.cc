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

  // File Handles are created by the open, opendir, and create methods and closed
  // by the release and releasedir methods.
  // save file handle for releasing handle safely
  std::unordered_set<uint64_t> file_pointer_set_;
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
      fuse_ = Fuse::GetInstance(mount_path_);
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

    // free file handle
    for (auto file_pointer : file_pointer_set_) {
      close(file_pointer);
    }
    file_pointer_set_.clear();

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
      HandleCommand(vq, element);
      PushQueue(vq, element);
    }
    NotifyQueue(vq);
  }

  bool FilePathSafeCheck(int fd) {
    // get fd file path
    char file_path[PATH_MAX] = {0};
    auto fd_path = "/proc/self/fd/" + std::to_string(fd);
    auto len = readlink(fd_path.c_str(), file_path, PATH_MAX);
    if (len == -1) {
      MV_LOG("readlink failed fd=%d", fd);
      return false;
    }

    // ignore '\0' in path_ end
    auto compare_len = mount_path_.size() - 1;
    if (len < (ssize_t)compare_len) {
      MV_LOG("file path too short len=%d", len);
      return false;
    }

    // fd path must start with mount_path_
    std::string file_path_str = file_path;
    if (0 != file_path_str.compare(0, compare_len, mount_path_, 0, compare_len)) {
      MV_PANIC("guest attempt to get out of share root path=%s", file_path);
      return false;
    }

    return true;
  }

  void HandleCommand(VirtQueue& vq, VirtElement* element) {
    auto& vector = element->vector;
    element->length = 0;

    // get request from guest
    auto request = (fuse_in_header*)vector.front().iov_base;
    uint64_t request_iov_len = vector.front().iov_len;
    vector.pop_front();

    if (debug_) {
      MV_LOG(
          "HandleCommand len=%d gid=%d opcode=%d padding=%d pid=%d uid=%d nodeid=%d "
          "unique=%d vector_size=%d",
          request->len, request->gid, request->opcode, request->padding, request->pid, request->uid,
          request->nodeid, request->unique, vector.size());
    }

    // handle command
    switch (request->opcode) {
      case FUSE_INIT: {
        auto in = (fuse_init_in*)OFFSET(request);
        auto response = (fuse_out_header*)vector.front().iov_base;
        auto out = (fuse_init_out*)OFFSET(response);
        vector.pop_front();

        if (debug_) {
          MV_LOG(
              "init major=%d minor=%d max_readahead=%d flags=%d "
              "flags2=%d",
              in->major, in->minor, in->max_readahead, in->flags, in->flags2);
        }

        // set out params
        out->major = FUSE_KERNEL_VERSION;
        out->minor = FUSE_KERNEL_MINOR_VERSION;
        out->flags |= FUSE_BIG_WRITES | FUSE_DO_READDIRPLUS | FUSE_WRITEBACK_CACHE;
        out->max_readahead = UINT32_MAX;
        out->max_write = UINT32_MAX;
        fuse_->MakeResponse(response, sizeof(fuse_init_out), 0, request->unique);
        element->length = response->len;
        break;
      }

      case FUSE_STATFS: {
        auto response = (fuse_out_header*)vector.front().iov_base;
        vector.pop_front();
        auto out = (fuse_statfs_out*)OFFSET(response);

        int fd = fuse_->LowLevelFd(request->nodeid);
        MV_ASSERT(fd != -1);

        struct statvfs stat_buf = {0};
        MV_ASSERT(fstatvfs(fd, &stat_buf) != -1);

        fuse_->ConvertStatfs(&stat_buf, &out->st);
        fuse_->MakeResponse(response, sizeof(fuse_statfs_out), 0, request->unique);
        element->length = response->len;
        break;
      }

      case FUSE_LOOKUP: {
        auto response = (fuse_out_header*)vector.front().iov_base;
        vector.pop_front();
        auto name = (const char*)OFFSET(request);
        struct fuse_entry_param entry_param = {0};
        auto error = fuse_->LowLevelLookup(request->nodeid, name, &entry_param);
        if (error != 0) {
          fuse_->MakeResponse(response, 0, -error, request->unique);
          element->length = response->len;
        } else {
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
        break;
      }

      case FUSE_READLINK: {
        auto response = (fuse_out_header*)vector.front().iov_base;
        auto iov_len = vector.back().iov_len;
        vector.pop_front();
        auto out = (char*)OFFSET(response);

        int fd = fuse_->LowLevelFd(request->nodeid);
        MV_ASSERT(fd != -1);

        auto ret = readlinkat(fd, "", out, iov_len);
        MV_ASSERT(ret != -1 && ret <= (ssize_t)iov_len);
        fuse_->MakeResponse(response, ret, 0, request->unique);
        element->length = response->len;
        break;
      }

      case FUSE_OPEN: {
        auto in = (fuse_open_in*)OFFSET(request);
        auto response = (fuse_out_header*)vector.front().iov_base;
        vector.pop_front();

        struct fuse_file_info file_info = {0};
        file_info.flags = in->flags;

        auto lo_data = fuse_->get_lo_data();
        /* With writeback cache, kernel may send read requests even
           when userspace opened write-only */
        if (lo_data->writeback && (file_info.flags & O_ACCMODE) == O_WRONLY) {
          file_info.flags &= ~O_ACCMODE;
          file_info.flags |= O_RDWR;
        }

        /* With writeback cache, O_APPEND is handled by the kernel.
           This breaks atomicity (since the file may change in the
           underlying filesystem, so that the kernel's idea of the
           end of the file isn't accurate anymore). In this example,
           we just accept that. A more rigorous filesystem may want
           to return an error here */
        if (lo_data->writeback && (file_info.flags & O_APPEND)) {
          file_info.flags &= ~O_APPEND;
        }

        int src_fd = fuse_->LowLevelFd(request->nodeid);
        if (src_fd == -1) {
          fuse_->MakeResponse(response, 0, -errno, request->unique);
          element->length = response->len;
          break;
        }

        auto self_path = "/proc/self/fd/" + std::to_string(src_fd);
        auto fd = open(self_path.c_str(), file_info.flags & ~O_NOFOLLOW);
        if (fd == -1) {
          fuse_->MakeResponse(response, 0, -errno, request->unique);
          element->length = response->len;
          break;
        }
        if (!FilePathSafeCheck(fd)) {
          close(fd);
          fuse_->MakeResponse(response, 0, -errno, request->unique);
          element->length = response->len;
          break;
        }

        fuse_open_out* out = (fuse_open_out*)OFFSET(response);
        out->fh = fd;
        file_pointer_set_.insert(fd);
        fuse_->MakeResponse(response, sizeof(fuse_open_out), 0, request->unique);
        element->length = response->len;
        break;
      }

      case FUSE_RELEASE: {
        auto in = (fuse_release_in*)OFFSET(request);
        auto response = (fuse_out_header*)vector.front().iov_base;
        vector.pop_front();

        if (file_pointer_set_.erase(in->fh)) {
          close(in->fh);
        }
        fuse_->MakeResponse(response, 0, 0, request->unique);
        element->length = response->len;
        break;
      }

      case FUSE_OPENDIR: {
        auto response = (fuse_out_header*)vector.front().iov_base;
        vector.pop_front();

        int src_fd = fuse_->LowLevelFd(request->nodeid);
        if (src_fd == -1) {
          fuse_->MakeResponse(response, 0, -errno, request->unique);
          element->length = response->len;
          break;
        }

        auto fd = openat(src_fd, ".", O_RDONLY);
        if (fd == -1) {
          fuse_->MakeResponse(response, 0, -errno, request->unique);
          element->length = response->len;
          break;
        }

        if (!FilePathSafeCheck(fd)) {
          close(fd);
          fuse_->MakeResponse(response, 0, -errno, request->unique);
          element->length = response->len;
          break;
        }

        auto dirp = (struct lo_dirp*)calloc(1, sizeof(struct lo_dirp));
        MV_ASSERT(dirp != NULL);
        dirp->dp = fdopendir(fd);
        if (dirp->dp == nullptr) {
          close(fd);
          free(dirp);
          fuse_->MakeResponse(response, 0, -errno, request->unique);
          element->length = response->len;
          break;
        }

        dirp->offset = 0;
        dirp->entry = nullptr;
        auto out = (fuse_open_out*)OFFSET(response);
        out->fh = (uint64_t)dirp;
        dirp_pointer_set_.insert(out->fh);
        fuse_->MakeResponse(response, sizeof(fuse_open_out), 0, request->unique);
        element->length = response->len;
        break;
      }

      case FUSE_RELEASEDIR: {
        auto in = (fuse_release_in*)OFFSET(request);
        auto response = (fuse_out_header*)vector.front().iov_base;
        vector.pop_front();

        if (dirp_pointer_set_.erase(in->fh)) {
          auto dirp = (lo_dirp*)in->fh;
          if (dirp) {
            closedir(dirp->dp);
            free(dirp);
          }
        }
        fuse_->MakeResponse(response, 0, 0, request->unique);
        element->length = response->len;
        break;
      }

      case FUSE_READDIR:
      case FUSE_READDIRPLUS: {
        auto in = (fuse_read_in*)OFFSET(request);
        std::string result_buf(in->size, '\0');
        uint32_t remain_size = 0;
        if (!fuse_->LowLevelReadDir(request, (char*)result_buf.data(), &remain_size,
                                    request->opcode == FUSE_READDIRPLUS)) {
          auto response = (fuse_out_header*)vector.front().iov_base;
          fuse_->MakeResponse(response, 0, -errno, request->unique);
          element->length = response->len;
          break;
        }

        // set result_buf_pointer to the head of result_buf
        auto result_buf_pointer = result_buf.data();

        // put result_buf into return vector
        char* tmp_out_buf = nullptr;
        fuse_out_header* tmp_out_header = nullptr;
        uint32_t res_len = in->size - remain_size;
        uint32_t tmp_buf_len = 0;
        while (res_len > 0 && vector.size() > 0) {
          tmp_out_header = (fuse_out_header*)vector.front().iov_base;
          tmp_out_buf = (char*)((uint8_t*)tmp_out_header + sizeof(fuse_out_header));
          tmp_buf_len = vector.front().iov_len - sizeof(fuse_out_header);
          bzero(tmp_out_header, sizeof(fuse_out_header));
          vector.pop_front();

          fuse_->MakeResponse(tmp_out_header, 0, 0, request->unique);
          if (res_len > tmp_buf_len) {
            memcpy(tmp_out_buf, result_buf_pointer, tmp_buf_len);
            tmp_out_header->len += tmp_buf_len;
            element->length += tmp_out_header->len;
            res_len -= tmp_buf_len;
            result_buf_pointer += tmp_buf_len;
          } else {
            memcpy(tmp_out_buf, result_buf_pointer, res_len);
            tmp_out_header->len += res_len;
            element->length += tmp_out_header->len;
            break;
          }
        }

        if (request->opcode == FUSE_READDIRPLUS) {
          // one item left to sen end iov
          MV_ASSERT(vector.size() >= 1);

          // set end iov
          tmp_out_header = (fuse_out_header*)vector.front().iov_base;
          vector.pop_front();
          fuse_->MakeResponse(tmp_out_header, 0, 0, request->unique);
          element->length += tmp_out_header->len;
        }
        break;
      }

      case FUSE_READ: {
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
          int64_t ret = fuse_->FuseBufCopy(&dst_mem_buf, &src_buf, (fuse_buf_copy_flags)0);
          MV_ASSERT(ret >= 0);

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
        break;
      }

      case FUSE_UNLINK: {
        fuse_out_header* response = (fuse_out_header*)vector.front().iov_base;
        vector.pop_front();
        auto name = (char*)OFFSET(request);
        auto fd = fuse_->LowLevelFd(request->nodeid);
        if (fd == -1) {
          fuse_->MakeResponse(response, 0, -errno, request->unique);
          element->length = response->len;
          break;
        }

        auto ret = unlinkat(fd, name, 0);
        fuse_->MakeResponse(response, 0, (ret == -1 ? errno : 0), request->unique);
        element->length = response->len;
        NotifyVirtioFs();
        break;
      }

      case FUSE_FORGET: {
        fuse_out_header* response = (fuse_out_header*)vector.front().iov_base;
        vector.pop_front();
        auto in = (fuse_forget_in*)OFFSET(request);
        auto lo_data = fuse_->get_lo_data();
        auto inode = fuse_->GetLowLevelInode(request->nodeid);
        if (inode == nullptr) {
          fuse_->MakeResponse(response, 0, -errno, request->unique);
          element->length = response->len;
          break;
        }

        fuse_->UnrefInode(lo_data, inode, in->nlookup);
        fuse_->MakeResponse(response, 0, 0, request->unique);
        element->length = response->len;
        NotifyVirtioFs();
        break;
      }

      case FUSE_FLUSH: {
        auto in = (fuse_flush_in*)OFFSET(request);
        auto ret = -1;
        if (file_pointer_set_.erase(in->fh)) {
          ret = close(dup(in->fh));
        }
        auto response = (fuse_out_header*)vector.front().iov_base;
        vector.pop_front();
        fuse_->MakeResponse(response, 0, ret == -1 ? -errno : 0, request->unique);
        element->length = response->len;
        break;
      }

      case FUSE_WRITE: {
        auto in = (fuse_write_in*)OFFSET(request);
        auto response = (fuse_out_header*)vector.back().iov_base;
        vector.pop_back();

        if (file_pointer_set_.find(in->fh) == file_pointer_set_.end()) {
          fuse_->MakeResponse(response, 0, -2, request->unique);
          element->length = response->len;
          break;
        }

        auto in_byte_buf = (char*)((uint8_t*)in + sizeof(fuse_write_in));
        uint64_t in_byte_buf_len = in->size;

        struct fuse_bufvec out_buf = FUSE_BUFVEC_INIT(in_byte_buf_len);
        out_buf.buf[0].flags = fuse_buf_flags(FUSE_BUF_IS_FD | FUSE_BUF_FD_SEEK);
        out_buf.buf[0].fd = in->fh;
        out_buf.buf[0].pos = in->offset;

        uint64_t iov_len = request_iov_len - sizeof(fuse_in_header) - sizeof(fuse_write_in);
        ssize_t ret = 0;
        while (in_byte_buf_len > 0) {
          struct fuse_bufvec in_buf = FUSE_BUFVEC_INIT(MIN(in_byte_buf_len, iov_len));
          in_buf.buf[0].mem = in_byte_buf;
          ret = fuse_->FuseBufCopy(&out_buf, &in_buf, (fuse_buf_copy_flags)0);
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
        fuse_->MakeResponse(response, sizeof(fuse_write_out), 0, request->unique);
        element->length = response->len;
        NotifyVirtioFs();
        break;
      }

      case FUSE_SETATTR: {
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
        in->valid &= FUSE_SET_ATTR_MODE | FUSE_SET_ATTR_UID | FUSE_SET_ATTR_GID |
                     FUSE_SET_ATTR_SIZE | FUSE_SET_ATTR_ATIME | FUSE_SET_ATTR_MTIME |
                     FUSE_SET_ATTR_ATIME_NOW | FUSE_SET_ATTR_MTIME_NOW | FUSE_SET_ATTR_CTIME;

        auto inode = fuse_->GetLowLevelInode(request->nodeid);
        MV_ASSERT(inode != nullptr);

        int ret = -1;
        char procname[64] = {0};
        if (in->valid & FUSE_SET_ATTR_MODE) {
          if (in->fh) {
            ret = fchmod(in->fh, stat.st_mode);
          } else {
            sprintf(procname, "/proc/self/fd/%i", inode->fd);
            ret = chmod(procname, stat.st_mode);
          }
          MV_ASSERT(ret != -1);
        }
        if (in->valid & (FUSE_SET_ATTR_UID | FUSE_SET_ATTR_GID)) {
          uid_t uid = (in->valid & FUSE_SET_ATTR_UID) ? stat.st_uid : (uid_t)-1;
          gid_t gid = (in->valid & FUSE_SET_ATTR_GID) ? stat.st_gid : (gid_t)-1;

          ret = fchownat(inode->fd, "", uid, gid, AT_EMPTY_PATH | AT_SYMLINK_NOFOLLOW);
          MV_ASSERT(ret != -1);
        }
        if (in->valid & FUSE_SET_ATTR_SIZE) {
          if (in->fh) {
            ret = ftruncate(in->fh, stat.st_size);
          } else {
            sprintf(procname, "/proc/self/fd/%i", inode->fd);
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
            sprintf(procname, "/proc/self/fd/%i", inode->fd);
            ret = utimensat(AT_FDCWD, procname, tv, 0);
          }
          MV_ASSERT(ret != -1);
        }
        // no break go on to FUSE_GETATTR
      }

      case FUSE_GETATTR: {
        auto response = (fuse_out_header*)vector.front().iov_base;
        vector.pop_front();

        int fd = fuse_->LowLevelFd(request->nodeid);
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
        break;
      }

      case FUSE_FALLOCATE: {
        auto in = (fuse_fallocate_in*)OFFSET(request);
        int ret = posix_fallocate(in->fh, in->offset, in->length);
        MV_ASSERT(ret == 0);
        auto response = (fuse_out_header*)vector.front().iov_base;
        vector.pop_front();
        fuse_->MakeResponse(response, 0, 0, request->unique);
        element->length = response->len;
        break;
      }

      case FUSE_CREATE: {
        auto in = (fuse_create_in*)OFFSET(request);
        auto name = (char*)((uint8_t*)in + sizeof(fuse_create_in));
        auto response = (fuse_out_header*)vector.back().iov_base;
        vector.pop_back();

        int src_fd = fuse_->LowLevelFd(request->nodeid);
        if (src_fd == -1) {
          fuse_->MakeResponse(response, 0, -errno, request->unique);
          element->length = response->len;
          break;
        }

        int fd = openat(src_fd, name, (in->flags | O_CREAT) & ~O_NOFOLLOW, in->mode);
        if (fd == -1) {
          fuse_->MakeResponse(response, 0, -errno, request->unique);
          element->length = response->len;
          break;
        }

        if (!FilePathSafeCheck(fd)) {
          close(fd);
          fuse_->MakeResponse(response, 0, -errno, request->unique);
          element->length = response->len;
          break;
        }

        struct fuse_entry_param entry_param = {0};
        auto ret = fuse_->LowLevelLookup(request->nodeid, name, &entry_param);
        if (ret == -1) {
          close(fd);
          fuse_->MakeResponse(response, 0, -errno, request->unique);
          element->length = response->len;
          break;
        }

        auto entry_out_arg = (struct fuse_entry_out*)OFFSET(response);
        entry_out_arg->nodeid = entry_param.ino;
        entry_out_arg->generation = entry_param.generation;
        entry_out_arg->entry_valid = fuse_->CalcTimeoutSecond(entry_param.entry_timeout);
        entry_out_arg->entry_valid_nsec = fuse_->CalcTimeoutNsecond(entry_param.entry_timeout);
        entry_out_arg->attr_valid = fuse_->CalcTimeoutSecond(entry_param.attr_timeout);
        entry_out_arg->attr_valid_nsec = fuse_->CalcTimeoutNsecond(entry_param.attr_timeout);
        fuse_->ConvertStat(&entry_param.attr, &entry_out_arg->attr);

        auto open_out_arg = (struct fuse_open_out*)OFFSET(entry_out_arg);
        open_out_arg->fh = fd;

        file_pointer_set_.insert(fd);
        fuse_->MakeResponse(response, sizeof(fuse_entry_out) + sizeof(fuse_open_out), 0,
                            request->unique);
        element->length = response->len;
        NotifyVirtioFs();
        break;
      }

      case FUSE_RMDIR: {
        auto ret = -1;
        auto fd = fuse_->LowLevelFd(request->nodeid);
        if (fd != -1) {
          auto name = (const char*)OFFSET(request);
          ret = unlinkat(fd, name, AT_REMOVEDIR);
        }
        auto response = (fuse_out_header*)vector.front().iov_base;
        vector.pop_front();
        fuse_->MakeResponse(response, 0, ret == -1 ? -errno : 0, request->unique);
        element->length = response->len;
        NotifyVirtioFs();
        break;
      }

      case FUSE_RENAME2: {
        auto in = (fuse_rename2_in*)OFFSET(request);
        auto old_name = (char*)((uint8_t*)in + sizeof(fuse_rename2_in));
        auto new_name = old_name + strlen(old_name) + 1;

        auto old_fd = fuse_->LowLevelFd(request->nodeid);
        MV_ASSERT(old_fd != -1);
        auto new_fd = fuse_->LowLevelFd(in->newdir);
        MV_ASSERT(new_fd != -1);

        auto ret = renameat(old_fd, old_name, new_fd, new_name);
        auto response = (fuse_out_header*)vector.front().iov_base;
        vector.pop_front();
        fuse_->MakeResponse(response, 0, ret == -1 ? -errno : 0, request->unique);
        element->length = response->len;
        NotifyVirtioFs();
        break;
      }

      case FUSE_MKDIR: {
        auto response = (fuse_out_header*)vector.front().iov_base;
        auto in = (fuse_mkdir_in*)OFFSET(request);
        auto name = (char*)((uint8_t*)in + sizeof(fuse_mkdir_in));

        struct fuse_entry_param entry_param = {0};
        auto dir_node = fuse_->GetLowLevelInode(request->nodeid);
        MV_ASSERT(dir_node != nullptr);

        auto ret = fuse_->MakeNodeWrapper(dir_node->fd, name, NULL, S_IFDIR | in->mode, 0);
        if (ret != -1) {
          ret = fuse_->LowLevelLookup(request->nodeid, name, &entry_param);
        }

        if (ret != 0) {
          fuse_->MakeResponse(response, 0, -errno, request->unique);
          element->length = response->len;
          break;
        }

        auto out = (struct fuse_entry_out*)((uint8_t*)response + sizeof(fuse_out_header));
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
