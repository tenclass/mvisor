/* 
 * MVisor
 * Copyright (C) 2021 Terrence <terrence@tenclass.com>
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

#include "disk_image.h"
#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>
#include <libgen.h>
#include <sys/stat.h>
#include <ctime>
#include <cstring>
#include <vector>
#include "lru_cache.h"
#include "logger.h"
#include "device_manager.h"

#define QCOW2_OFLAG_COPIED        (1UL << 63)
#define QCOW2_OFLAG_COMPRESSED    (1UL << 62)
#define QCOW2_OFLAGS_MASK         (QCOW2_OFLAG_COPIED | QCOW2_OFLAG_COMPRESSED)
#define QCOW2_OFFSET_MASK         (~QCOW2_OFLAGS_MASK)

#define REFCOUNT_CACHE_ITEMS      128
#define L2_CACHE_ITEMS            128

inline void be32_to_cpus(uint32_t* x) {
  *x = be32toh(*x);
}
inline void be64_to_cpus(uint64_t* x) {
  *x = be64toh(*x);
}

struct Qcow2Header {
  uint32_t magic;
  uint32_t version;

  uint64_t backing_file_offset;
  uint32_t backing_file_size;

  uint32_t cluster_bits;
  uint64_t size;
  uint32_t crypt_method;

  uint32_t l1_size;
  uint64_t l1_table_offset;

  uint64_t refcount_table_offset;
  uint32_t refcount_table_clusters;

  uint32_t nb_snapshots;
  uint64_t snapshots_offset;
} __attribute__ ((packed));

struct L2Table {
  uint64_t    offset_in_file;
  bool        dirty;
  uint64_t    entries[];
};

struct RefcountBlock {
  uint64_t    offset_in_file;
  bool        dirty;
  uint16_t    entries[];
};

/* Reference: https://git.qemu.org/?p=qemu.git;a=blob;f=docs/interop/qcow2.txt
 * All numbers in Qcow2 are stored in Big Endian byte order
 * Take an example created with the following commandline:
 * # qemu-img create -f qcow2 -F qcow2 -b win10.qcow2 hd.qcow2
 * 
 * The file size of hd.qcow2 is 196KB (approximately 3 clsuters)
 * cluster_bits = 16, then cluster_size = 0x10000 (64KB)
 * refcount_table starts at 0x10000
 * The first refcount_block starts at 0x20000
 * l1_table starts at 0x30000
 * 
 * A hex view of the file data at 0x10000 to the end of the newly created file
 * # hexdump -C hd.qcow2  -s 0x10000
 * 00010000  00 00 00 00 00 02 00 00  00 00 00 00 00 00 00 00  |................|
 * 00010010  00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |................|
 *
 * 00020000  00 01 00 01 00 01 00 01  00 00 00 00 00 00 00 00  |................|
 * 00020010  00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |................|
 *
 * 00030780
 * 
 * Assume you are reading at the first sector of the disk:
 * 1. Look for the l1_table at 0x30000 with l1_index = 0, get l1_entry = 0
 * 2. If l1_entry has no COPIED flag, try to read the backing file
 * 3. If l1_entry has COPIED flag, read from current file
 * 
 * refcount_table is used when allocating a cluster for l2_table or refcount block
 * or data cluster 
 */

typedef std::function<void()> VoidCallback;

class Qcow2Image : public DiskImage {
 private:
  int fd_ = -1;
  size_t block_size_shift_ = 9; // block size = 4KB
  size_t total_blocks_ = 0;
  size_t image_size_ = 0;
  size_t cluster_size_;

  size_t l2_entries_;
  size_t rfb_entries_;
  size_t refcount_bits_;

  uint64_t free_cluster_index_ = 0;
  uint8_t* copied_cluster_ = nullptr;

  std::vector<uint64_t> l1_table_;
  std::vector<uint64_t> refcount_table_;
  bool l1_table_dirty_ = false;
  bool refcount_table_dirty_ = false;

  SimpleLRUCache<uint64_t, L2Table*> l2_cache_;
  SimpleLRUCache<uint64_t, RefcountBlock*> rfb_cache_;

  Qcow2Header image_header_;
  std::string backing_filepath_;
  Qcow2Image* backing_file_ = nullptr;

  IoThread* io_ = nullptr;

  ImageInformation information() {
    return ImageInformation {
      .block_size = 1UL << block_size_shift_,
      .total_blocks = total_blocks_
    };
  }

  ~Qcow2Image() {
    /* Flush caches if dirty */
    l2_cache_.Clear();
    rfb_cache_.Clear();

    if (fd_ != -1) {
      Flush([=](long ret){
        close(fd_);
      });
    }

    if (copied_cluster_) {
      delete[] copied_cluster_;
    }

    if (backing_file_) {
      delete backing_file_;
    }
  }

  void Initialize(const std::string& path, bool readonly) {
    MV_ASSERT(device_);
    io_ = device_->manager()->io();
    readonly_ = readonly;

    if (readonly) {
      fd_ = open(path.c_str(), O_RDONLY);
    } else {
      fd_ = open(path.c_str(), O_RDWR);
    }
    if (fd_ < 0)
      MV_PANIC("disk file not found: %s", path.c_str());

    struct stat st;
    fstat(fd_, &st);
    image_size_ = st.st_size;
  
    InitializeQcow2Header();
    InitializeL1Table();
    InitializeRefcountTable();
    InitializeLruCache();
    
    /* Setup backing file READONLY if valid */
    if (image_header_.backing_file_offset && image_header_.backing_file_size < 1024) {
      char filename[1024] = { 0 };
      ReadFile(filename, image_header_.backing_file_size, image_header_.backing_file_offset);
      if (filename[0] == '/') {
        backing_filepath_ = filename;
      } else {
        char temp[1024] = {  0 };
        strncpy(temp, path.c_str(), sizeof(temp) - 1);
        backing_filepath_ = std::string(dirname(temp)) + "/" + filename;
      }
      backing_file_ = new Qcow2Image();
      backing_file_->device_ = device_;
      backing_file_->Initialize(backing_filepath_, true);
    }
    // MV_LOG("open qcow2 %s file size=%ld", path.c_str(), image_size_);
  }

  void InitializeQcow2Header() {
    /* Read the image header at offset 0 */
    ReadFile(&image_header_, sizeof(image_header_), 0);
    /* Bigendian to host */
    be32_to_cpus(&image_header_.magic);
    be32_to_cpus(&image_header_.version);
    be64_to_cpus(&image_header_.backing_file_offset);
    be32_to_cpus(&image_header_.backing_file_size);
    be32_to_cpus(&image_header_.cluster_bits);
    be64_to_cpus(&image_header_.size);
    be32_to_cpus(&image_header_.crypt_method);
    be32_to_cpus(&image_header_.l1_size);
    be64_to_cpus(&image_header_.l1_table_offset);
    be64_to_cpus(&image_header_.refcount_table_offset);
    be32_to_cpus(&image_header_.refcount_table_clusters);
    be32_to_cpus(&image_header_.nb_snapshots);
    be64_to_cpus(&image_header_.snapshots_offset);
    if (image_header_.version > 3) {
      MV_PANIC("Qcow2 file version=0x%x not supported", image_header_.version);
    }

    total_blocks_ = image_header_.size >> block_size_shift_;
    cluster_size_ = 1 << image_header_.cluster_bits;
    l2_entries_ = cluster_size_ / sizeof(uint64_t);
    copied_cluster_ = new uint8_t[cluster_size_];
  
    /* For version 2, refcount bits is always 16 */
    refcount_bits_ = 16;
    rfb_entries_ = cluster_size_ * 8 / refcount_bits_;

    /* If there is no snapshot, we assume all refcounts <= 1 */
    if (image_header_.nb_snapshots) {
      MV_PANIC("Qcow2 file with snapshots is not supported yet");
    }
  }

  /* FIXME: should we call pwrite for multiple times to write all data ??? */
  ssize_t WriteFile(void* buffer, size_t length, off_t offset) {
    if (offset >= (ssize_t)image_header_.size) {
      MV_LOG("write overflow length=0x%lx, offset=0x%lx", length, offset);
      return 0;
    }
    ssize_t ret = pwrite(fd_, buffer, length, offset);
    MV_ASSERT(ret == (ssize_t)length);
    return ret;
  }

  /* FIXME: should we call pread for multiple times to read all data ??? */
  ssize_t ReadFile(void* buffer, size_t length, off_t offset) {
    ssize_t ret = pread(fd_, buffer, length, offset);
    return ret;
  }

  void WriteFileAsync(void* buffer, size_t length, off_t offset, IoCallback callback) {
    io_->Write(fd_, buffer, length, offset, callback);
  }

  void ReadFileAsync(void* buffer, size_t length, off_t offset, IoCallback callback) {
    io_->Read(fd_, buffer, length, offset, callback);
  }

  void InitializeL1Table() {
    l1_table_.resize(image_header_.l1_size);
    ReadFile(l1_table_.data(), sizeof(uint64_t) * image_header_.l1_size,
      image_header_.l1_table_offset
    );
  }

  void InitializeRefcountTable () {
    size_t refcount_bytes = image_header_.refcount_table_clusters * cluster_size_;
    refcount_table_.resize(refcount_bytes  / sizeof(uint64_t));
    ReadFile(refcount_table_.data(), refcount_bytes, image_header_.refcount_table_offset);
  }

  void WriteL1Table() {
    WriteFile(l1_table_.data(), l1_table_.size() * sizeof(uint64_t),
      image_header_.l1_table_offset);
    l1_table_dirty_ = false;
  }

  void WriteRefcountTable() {
    WriteFile(refcount_table_.data(), refcount_table_.size() * sizeof(uint64_t),
      image_header_.refcount_table_offset);
    refcount_table_dirty_ = false;
  }

  void WriteL2Table(L2Table* l2_table) {
    WriteFile(l2_table->entries, l2_entries_ * sizeof(uint64_t),
      l2_table->offset_in_file);
    l2_table->dirty = false;
  }

  void WriteRefcountBlock(RefcountBlock* rfb) {
    WriteFile(rfb->entries, rfb_entries_ * sizeof(uint16_t), rfb->offset_in_file);
    rfb->dirty = false;
  }

  void InitializeLruCache() {
    rfb_cache_.Initialize(REFCOUNT_CACHE_ITEMS, [this](auto offset_in_file, auto rfb) {
      if (rfb->dirty) {
        WriteRefcountBlock(rfb);
      }
      // MV_LOG("free rfb 0x%lx", rfb->offset_in_file);
      free(rfb);
    });
    l2_cache_.Initialize(L2_CACHE_ITEMS, [this](auto offset_in_file, auto l2_table) {
      if (l2_table->dirty) {
        WriteL2Table(l2_table);
      }
      // MV_LOG("free l2_table 0x%lx", l2_table->offset_in_file);
      free(l2_table);
    });
  }

  RefcountBlock* NewRefcountBlock(uint64_t block_offset) {
    size_t size = sizeof(RefcountBlock) + rfb_entries_ * sizeof(uint16_t);
    RefcountBlock* block = (RefcountBlock*)malloc(size);
    block->dirty = false;
    block->offset_in_file = block_offset;
    return block;
  }

  /*
   * FIXME: This is not asynchronous yet
   */
  RefcountBlock* GetRefcountBlock(uint64_t cluster_index, uint64_t* rfb_index, bool allocate) {
    uint64_t rft_index = cluster_index / rfb_entries_;
    *rfb_index = cluster_index % rfb_entries_;
    
    if (rft_index >= refcount_table_.size()) {
      return nullptr;
    }
    RefcountBlock* rfb;
    uint64_t block_offset = be64toh(refcount_table_[rft_index]);
    if (!block_offset) {
      /* If refcount block is not allocated, this cluster must be free and will be used
       * as a refcount block. Right???
       */
      if (!allocate) {
        return nullptr;
      }
      block_offset = cluster_index * cluster_size_;
      rfb = NewRefcountBlock(block_offset);
      bzero(rfb->entries, rfb_entries_ * sizeof(uint16_t));
      rfb_cache_.Put(block_offset, rfb);
      rfb->entries[*rfb_index] = htobe16(1);
      rfb->dirty = true;

      refcount_table_[rft_index] = htobe64(block_offset);
      refcount_table_dirty_ = true;
      return rfb;
    } else {
      if (rfb_cache_.Get(block_offset, rfb)) {
        return rfb;
      }

      rfb = NewRefcountBlock(block_offset);
      ReadFile(rfb->entries, rfb_entries_ * sizeof(uint16_t), rfb->offset_in_file);
      rfb_cache_.Put(rfb->offset_in_file, rfb);
      return rfb;
    }
  }

  void FreeCluster(uint64_t start) {
    uint64_t rfb_index;
    uint64_t cluster_index = start / cluster_size_;
    RefcountBlock* rfb = GetRefcountBlock(cluster_index, &rfb_index, false);
    if (rfb == nullptr) {
      MV_PANIC("Try to free an unallocated cluster 0x%lx", start);
      return;
    }

    rfb->entries[rfb_index] = htobe16(0); // Set refcount to 0
    rfb->dirty = true;
    if (cluster_index < free_cluster_index_) {
      free_cluster_index_ = cluster_index;
    }
  }

  /* free_cluster_index_ is initialized to zero and record last position.
   * FIXME: This is not asynchronous yet
   */
  uint64_t AllocateCluster() {
    uint64_t rfb_index;
    uint64_t cluster_index = free_cluster_index_++;
    RefcountBlock* rfb = GetRefcountBlock(cluster_index, &rfb_index, true);
    while (true) {
      if (rfb == nullptr) {
        return 0; // Error occurred
      }
      uint16_t refcount = be16toh(rfb->entries[rfb_index]);
      if (refcount == 0) {
        break;
      }
      cluster_index = free_cluster_index_++;
      if (++rfb_index >= rfb_entries_) {
        rfb = GetRefcountBlock(cluster_index, &rfb_index, true);
      }
    }

    // update refcount and set dirty
    rfb->entries[rfb_index] = htobe16(1);
    rfb->dirty = true;
    return cluster_index * cluster_size_;
  }

  L2Table* NewL2Table(uint64_t l2_offset) {
    size_t size = sizeof(L2Table) + l2_entries_ * sizeof(uint64_t);
    L2Table* table = (L2Table*)malloc(size);
    table->dirty = false;
    table->offset_in_file = l2_offset;
    return table;
  }

  void ReadL2Table(uint64_t l2_offset, std::function<void (L2Table*)> callback) {
    L2Table* table;
    if (l2_cache_.Get(l2_offset, table)) {
      return callback(table);
    }

    table = NewL2Table(l2_offset);
    ReadFileAsync(table->entries, l2_entries_ * sizeof(uint64_t), table->offset_in_file, [=](ssize_t ret) {
      L2Table* t = table;
      l2_cache_.Put(table->offset_in_file, t);
      callback(table);
    });
  }

  void GetL2Table(bool is_write, off_t pos, uint64_t length,
    std::function<void (L2Table* l2_table, uint64_t l2_index, uint64_t offset_in_cluster, uint64_t length)> callback) {
    uint64_t offset_in_cluster, l2_index;
    offset_in_cluster = pos % cluster_size_;
    if (length > cluster_size_ - offset_in_cluster) {
      length = cluster_size_ - offset_in_cluster;
    }
  
    uint64_t cluster_index = pos / cluster_size_;
    uint64_t l1_index = cluster_index / l2_entries_;
    l2_index = cluster_index % l2_entries_;
    
    uint64_t l2_offset = be64toh(l1_table_[l1_index]);
    if (l2_offset & QCOW2_OFLAG_COPIED) { /* L2 already allocated, read from current file */
      l2_offset &= ~QCOW2_OFLAG_COPIED;
      if (!l2_offset) { /* copied l1 entry must be valid */
        MV_PANIC("l2_offset is not valid");
      }
      ReadL2Table(l2_offset, [=](auto l2_table) {
        callback(l2_table, l2_index, offset_in_cluster, length);
      });
    } else if (is_write) { /* L2 not allocated, but is a write operation */
      MV_ASSERT(l2_offset == 0); /* @XX: l2_offset != 0 if nb_snapshots > 0 ??? */
      l2_offset = AllocateCluster();
      MV_ASSERT(l2_offset);
      
      L2Table* l2_table = NewL2Table(l2_offset);
      bzero(l2_table->entries, l2_entries_ * sizeof(uint64_t));
      l2_cache_.Put(l2_offset, l2_table);
      l2_table->dirty = true;

      l1_table_[l1_index] = htobe64(l2_offset | QCOW2_OFLAG_COPIED);
      l1_table_dirty_ = true;
      callback(l2_table, l2_index, offset_in_cluster, length);
    } else { /* L2 not allocated, but should read from backing file if possible */
      callback(nullptr, l2_index, offset_in_cluster, length);
    }
  }
  
  /* The return value is always less than or equal to cluster size */
  void ReadCluster(void* buffer, off_t pos, size_t length0, bool no_zero, IoCallback callback) {
    GetL2Table(false, pos, length0, [=] (auto l2_table, uint64_t l2_index, uint64_t offset_in_cluster, uint64_t length) {
      if (l2_table == nullptr) {
        if (backing_file_ == nullptr) { /* Reading at unallocated space always return zero??? */
          if (no_zero)
            return callback(-2);
          bzero(buffer, length);
          return callback(length);
        }
        return backing_file_->ReadCluster(buffer, pos, length, false, callback);
      }

      uint64_t cluster_start = be64toh(l2_table->entries[l2_index]);
      if (cluster_start & QCOW2_OFLAG_COMPRESSED) {
        MV_PANIC("not supported compressed pos=0x%lx cluster=0x%lx", pos, cluster_start);
      } else {
        cluster_start &= QCOW2_OFFSET_MASK;
        if (cluster_start == 0) {
          if (backing_file_ == nullptr) { /* unallocated means zero */
            if (no_zero)
              return callback(-2);
            bzero(buffer, length);
            return callback(length);
          }
          return backing_file_->ReadCluster(buffer, pos, length, false, callback);
        }

        ReadFileAsync(buffer, length, cluster_start + offset_in_cluster, [=](ssize_t bytes_read) {
          if (bytes_read < 0) {
            return callback(bytes_read);
          }
          if ((size_t)bytes_read < length) {
            /* Reach the end of file??? */
            bzero((uint8_t*)buffer + bytes_read, length - bytes_read);
          }
          callback(length);
        });
      }
    });
  }

  /* The return value is always less than or equal to cluster size */
  void WriteCluster(void* buffer, off_t pos, size_t length0, IoCallback callback) {
    GetL2Table(true, pos, length0, [=] (auto l2_table, uint64_t l2_index, uint64_t offset_in_cluster, uint64_t length) {
      MV_ASSERT(l2_table);

      uint64_t cluster_start = be64toh(l2_table->entries[l2_index]);
      uint64_t cluster_flags = cluster_start & QCOW2_OFLAGS_MASK;
      cluster_start &= QCOW2_OFFSET_MASK;

      if (cluster_flags & QCOW2_OFLAG_COPIED) {
        WriteFileAsync(buffer, length, cluster_start + offset_in_cluster, [=](ssize_t ret) {
          MV_ASSERT(ret == (ssize_t)length);
          callback(length);
        });
      } else {
        if (cluster_start) {
          MV_PANIC("writing to images with snapshots is not supported yet");
        }
        cluster_start = AllocateCluster();
        if (cluster_start == 0) {
          MV_LOG("failed to allocate cluster");
          return callback(-1);
        }
    
        l2_table->entries[l2_index] = htobe64(cluster_start | QCOW2_OFLAG_COPIED);
        l2_table->dirty = true;
    
        /* If not writing the whole cluster, we should read the original data from the backing file
        * Always read the whole cluster without zeroing data if cluster exists in backing file
        */
        if (backing_file_ && !(offset_in_cluster == 0 && length == cluster_size_)) {
          uint8_t* copied_cluster = new uint8_t[cluster_size_];
          MV_ASSERT(copied_cluster);
          backing_file_->ReadCluster(copied_cluster, pos - offset_in_cluster, cluster_size_, true, [=](auto bytes) {
            if (bytes > 0) { // Check if exists
              MV_ASSERT(bytes == (ssize_t)cluster_size_); // Make sure we have a whole cluster
              memcpy(copied_cluster + offset_in_cluster, buffer, length);
              WriteFileAsync(copied_cluster, cluster_size_, cluster_start, [=](ssize_t ret) {
                MV_ASSERT(ret == (ssize_t)cluster_size_);
                delete copied_cluster;
                callback(length);
              });
            } else {
              WriteFileAsync(buffer, length, cluster_start + offset_in_cluster, [=](ssize_t ret) {
                MV_ASSERT(ret == (ssize_t)length);
                delete copied_cluster;
                callback(length);
              });
            }
          });
        } else {
          WriteFileAsync(buffer, length, cluster_start + offset_in_cluster, [=](ssize_t ret) {
            MV_ASSERT(ret == (ssize_t)length);
            callback(length);
          });
        }
      }
    });
  }

  void HandleIoAsync(void *buffer, off_t position, size_t total_length, ssize_t bytes_rw, bool is_write, IoCallback callback) {
    uint8_t *ptr = (uint8_t*)buffer;
  
    if ((uint64_t)position >= image_header_.size) {
      return callback(bytes_rw);
    }

    auto io_complete = [=](ssize_t ret) {
      if (ret <= 0) {
        return callback(ret);
      }
      if (bytes_rw + ret < (ssize_t)total_length) {
        return HandleIoAsync((uint8_t*)buffer + ret, position + ret, total_length, bytes_rw + ret, is_write, callback);
      } else {
        return callback(bytes_rw + ret);
      }
    };

    if (is_write) {
      WriteCluster(ptr, position, total_length - bytes_rw, io_complete);
    } else {
      ReadCluster(ptr, position, total_length - bytes_rw, false, io_complete);
    }
  }

  void Write(void *buffer, off_t position, size_t length, IoCallback callback) {
    HandleIoAsync(buffer, position, length, 0, true, callback);
  }

  void Read(void *buffer, off_t position, size_t length, IoCallback callback) {
    HandleIoAsync(buffer, position, length, 0, false, callback);
  }

  void FlushL2Tables () {
    int l2_table_dirty_count = 0;
    auto map = l2_cache_.map();
    for (auto it = map.begin(); it != map.end(); it++) {
      auto l2_table = it->second->second;
      if (l2_table->dirty) {
        WriteL2Table(l2_table);
        ++l2_table_dirty_count;
      }
    }
  }

  void FlushRefcountBlocks() {
    int refcount_block_dirty_count = 0;
    auto map = rfb_cache_.map();
    for (auto it = map.begin(); it != map.end(); it++) {
      auto rfb = it->second->second;
      if (rfb->dirty) {
        WriteRefcountBlock(rfb);
        ++refcount_block_dirty_count;
      }
    }
  }

  void Flush(IoCallback callback) {
    if (readonly_) {
      callback(0);
    }

    FlushL2Tables();
    FlushRefcountBlocks();
    if (l1_table_dirty_) {
      WriteL1Table();
    }
    if (refcount_table_dirty_) {
      WriteRefcountTable();
    }

    io_->FSync(fd_, 0, [=](long ret) {
      if (ret < 0) {
        MV_PANIC("failed to sync disk image, ret=%d", ret);
      }
      callback(ret);
    });
  }

  // /* The return value is always less than or equal to cluster size */
  // ssize_t TrimCluster(off_t pos, size_t length) {
  //   uint64_t offset_in_cluster, l2_index;
  //   auto l2_table = GetL2Table(false, pos, &offset_in_cluster, &l2_index, &length);
  //   if (l2_table == nullptr) {
  //     return length;
  //   }

  //   uint64_t cluster_start = be64toh(l2_table->entries[l2_index]);
  //   if (cluster_start & QCOW2_OFLAG_COMPRESSED) {
  //     MV_PANIC("not supported compressed pos=0x%lx cluster=0x%lx", pos, cluster_start);
  //   } else {
  //     cluster_start &= QCOW2_OFFSET_MASK;
  //     if (cluster_start == 0) {
  //       return length;
  //     }

  //     FreeCluster(cluster_start); // Set refcount to 0
  //     l2_table->entries[l2_index] = be64toh(0);
  //     l2_table->dirty = true;
  //   }
  //   return length;
  // }

  // /* The OS use TRIM command to inform us some disk regions are freed
  //  * To recycle these regions, clear the L2 table entry, and set the refcount to 0
  //  */
  // void Trim(off_t position, size_t length, IoCallback callback) {
  //   /* FIXME: this method crashes the file system */
  //   return;

  //   size_t bytes_trimed = 0;
  
  //   while (bytes_trimed < length) {
  //     if (readonly_ || (uint64_t)position >= image_header_.size) {
  //       return;
  //     }

  //     ssize_t ret = TrimCluster(position, length - bytes_trimed);
  //     if (ret <= 0) {
  //       return;
  //     }

  //     bytes_trimed += ret;
  //     position += ret;
  //   }
  // }

};

DECLARE_DISK_IMAGE(Qcow2Image);
