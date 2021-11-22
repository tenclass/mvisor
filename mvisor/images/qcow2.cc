#include "disk_image.h"
#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <ctime>
#include <cstring>
#include <vector>
#include "lru_cache.h"
#include "logger.h"

#define QCOW2_OFLAG_COPIED        (1UL << 63)
#define QCOW2_OFLAG_COMPRESSED    (1UL << 62)
#define QCOW2_OFLAGS_MASK	        (QCOW2_OFLAG_COPIED | QCOW2_OFLAG_COMPRESSED)
#define QCOW2_OFFSET_MASK	        (~QCOW2_OFLAGS_MASK)

inline void be32_to_cpus(uint32_t* x) {
  *x = be32toh(*x);
}
inline void be64_to_cpus(uint64_t* x) {
  *x = be64toh(*x);
}

/* All numbers in qcow2 are stored in Big Endian byte order */
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

class Qcow2 : public DiskImage {
 private:
  int fd_ = -1;
  bool readonly_ = true;
  size_t block_size_shift_ = 9; // 512 block size
  size_t total_blocks_ = 0;
  size_t image_size_ = 0;
  size_t cluster_size_;
  size_t l2_table_entries_;
  size_t refcount_block_entries_;
  size_t refcount_bits;
  uint64_t free_cluster_index_ = 0;

  std::vector<uint64_t> l1_table_;
  std::vector<uint64_t> refcount_table_;
  bool l1_table_dirty_ = false;
  bool refcount_table_dirty_ = false;

  SimpleLRUCache<uint64_t, L2Table*> l2_table_cache_;
  SimpleLRUCache<uint64_t, RefcountBlock*> refcount_block_cache_;

  Qcow2Header image_header_;

  ImageInformation information() {
    return ImageInformation {
      .block_size = 1UL << block_size_shift_,
      .total_blocks = total_blocks_
    };
  }

  void Initialize(const std::string& path, bool readonly) {
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
    l2_table_entries_ = cluster_size_ / sizeof(uint64_t);
    /* For version 2, refcount bits is always 16 */
    refcount_bits = 16;
    refcount_block_entries_ = cluster_size_ * 8 / refcount_bits;

    /* If there is no snapshot, we assume all refcounts <= 1 */
    if (image_header_.nb_snapshots) {
      MV_PANIC("Qcow2 file with snapshots is not supported yet");
    }
  }

  ssize_t WriteFile(void* buffer, size_t length, off_t offset) {
    if (offset >= (ssize_t)image_header_.size) {
      MV_LOG("write overflow length=0x%lx, offset=0x%lx", length, offset);
      int a = 0;
      return 4 / a;
    }
    return pwrite(fd_, buffer, length, offset);
  }

  ssize_t ReadFile(void* buffer, size_t length, off_t offset) {
    return pread(fd_, buffer, length, offset);
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
    WriteFile(l2_table->entries, l2_table_entries_ * sizeof(uint64_t),
      l2_table->offset_in_file);
    l2_table->dirty = false;
  }

  void WriteRefcountBlock(RefcountBlock* rfb) {
    WriteFile(rfb->entries, refcount_block_entries_ * sizeof(uint16_t), rfb->offset_in_file);
    rfb->dirty = false;
  }

  void InitializeLruCache() {
    refcount_block_cache_.Initialize(128, [this](auto offset_in_file, auto rfb) {
      if (rfb->dirty) {
        WriteRefcountBlock(rfb);
      }
      MV_LOG("free rfb 0x%lx", rfb->offset_in_file);
      free(rfb);
    });
    l2_table_cache_.Initialize(128, [this](auto offset_in_file, auto l2_table) {
      if (l2_table->dirty) {
        WriteL2Table(l2_table);
      }
      MV_LOG("free l2_table 0x%lx", l2_table->offset_in_file);
      free(l2_table);
    });
  }

  RefcountBlock* NewRefcountBlock(uint64_t block_offset) {
    size_t size = sizeof(RefcountBlock) + refcount_block_entries_ * sizeof(uint16_t);
    RefcountBlock* block = (RefcountBlock*)malloc(size);
    block->dirty = false;
    block->offset_in_file = block_offset;
    return block;
  }

  RefcountBlock* GetRefcountBlock(uint64_t cluster_index, uint64_t* rfb_index) {
    uint64_t rft_index = cluster_index / refcount_block_entries_;
    *rfb_index = cluster_index % refcount_block_entries_;
    
    if (rft_index >= refcount_table_.size()) {
      return nullptr;
    }
    RefcountBlock* rfb;
    uint64_t block_offset = be64toh(refcount_table_[rft_index]);
    if (!block_offset) {
      /* If refcount block is not allocated, this cluster must be free and will be used
       * as a refcount block. Right???
       */
      block_offset = cluster_index * cluster_size_;
      rfb = NewRefcountBlock(block_offset);
      bzero(rfb->entries, refcount_block_entries_ * sizeof(uint16_t));
      refcount_block_cache_.Put(block_offset, rfb);
      rfb->entries[*rfb_index] = htobe16(1);
      rfb->dirty = true;

      refcount_table_[rft_index] = htobe64(block_offset);
      refcount_table_dirty_ = true;
      return rfb;
    } else {
      if (refcount_block_cache_.Get(block_offset, rfb)) {
        return rfb;
      }

      rfb = NewRefcountBlock(block_offset);
      ReadFile(rfb->entries, refcount_block_entries_ * sizeof(uint16_t), rfb->offset_in_file);
      refcount_block_cache_.Put(rfb->offset_in_file, rfb);
      return rfb;
    }
  }

  /* free_cluster_index_ is initialized to zero and record last position. */
  uint64_t AllocateCluster() {
    uint64_t rfb_index;
    uint64_t cluster_index = free_cluster_index_++;
    RefcountBlock* rfb = GetRefcountBlock(cluster_index, &rfb_index);
    while (true) {
      if (rfb == nullptr) {
        return 0; // Error occurred
      }
      uint16_t refcount = be16toh(rfb->entries[rfb_index]);
      if (refcount == 0) {
        break;
      }
      cluster_index = free_cluster_index_++;
      if (++rfb_index >= refcount_block_entries_) {
        rfb = GetRefcountBlock(cluster_index, &rfb_index);
      }
    }

    // update refcount and set dirty
    rfb->entries[rfb_index] = htobe16(1);
    rfb->dirty = true;
    return cluster_index * cluster_size_;
  }

  L2Table* NewL2Table(uint64_t l2_table_offset) {
    size_t size = sizeof(L2Table) + l2_table_entries_ * sizeof(uint64_t);
    L2Table* table = (L2Table*)malloc(size);
    table->dirty = false;
    table->offset_in_file = l2_table_offset;
    return table;
  }

  L2Table* ReadL2Table(uint64_t l2_table_offset) {
    L2Table* table;
    if (l2_table_cache_.Get(l2_table_offset, table)) {
      return table;
    }

    table = NewL2Table(l2_table_offset);
    ReadFile(table->entries, l2_table_entries_ * sizeof(uint64_t), table->offset_in_file);

    l2_table_cache_.Put(table->offset_in_file, table);
    return table;
  }

  L2Table* GetL2Table(bool is_write, off_t offset, uint64_t* offset_in_cluster, uint64_t* l2_index, size_t* length) {
    *offset_in_cluster = offset % cluster_size_;
    if (*length > cluster_size_ - *offset_in_cluster) {
      *length = cluster_size_ - *offset_in_cluster;
    }
  
    uint64_t cluster_index = offset / cluster_size_;
    uint64_t l1_index = cluster_index / l2_table_entries_;
    *l2_index = cluster_index % l2_table_entries_;
    
    uint64_t l2_table_offset = be64toh(l1_table_[l1_index]);
    if (l2_table_offset & QCOW2_OFLAG_COPIED) {
      l2_table_offset &= ~QCOW2_OFLAG_COPIED;
      if (!l2_table_offset) { /* copied l1 entry must be valid */
        MV_PANIC("l2_table_offset is not valid");
      }
      return ReadL2Table(l2_table_offset);
    } else if (is_write) {
      if (l2_table_offset != 0) {
        MV_PANIC("l2_table_offset should be 0, now is 0x%lx", l2_table_offset);
      }

      l2_table_offset = AllocateCluster();
      MV_ASSERT(l2_table_offset);
      
      L2Table* l2_table = NewL2Table(l2_table_offset);
      bzero(l2_table->entries, l2_table_entries_ * sizeof(uint64_t));
      l2_table_cache_.Put(l2_table_offset, l2_table);
      l2_table->dirty = true;

      l1_table_[l1_index] = htobe64(l2_table_offset | QCOW2_OFLAG_COPIED);
      l1_table_dirty_ = true;
      return l2_table;
    }
    return nullptr;
  }

  ssize_t Read(void *buffer, off_t block, size_t block_count) {
    size_t bytes_read = 0;
    size_t total_bytes = block_count << block_size_shift_;
    uint8_t *ptr = (uint8_t*)buffer;
    size_t offset = block << block_size_shift_;
  
    while (bytes_read < total_bytes) {
      if (offset >= image_header_.size) {
        return bytes_read;
      }

      ssize_t ret = ReadCluster(ptr, offset, total_bytes - bytes_read);
      if (ret <= 0) {
        return ret;
      }

      bytes_read += ret;
      ptr += ret;
      offset += ret;
    }
    return bytes_read;
  }
  
  ssize_t ReadCluster(void* buffer, off_t offset, size_t length) {
    uint64_t offset_in_cluster, l2_index;
    auto l2_table = GetL2Table(false, offset, &offset_in_cluster, &l2_index, &length);
    if (nullptr == l2_table) {
      bzero(buffer, length);
      return length;
    }

    uint64_t cluster_start = be64toh(l2_table->entries[l2_index]);
    if (cluster_start & QCOW2_OFLAG_COMPRESSED) {
      MV_PANIC("not supported compressed offset=0x%lx cluster=0x%lx", offset, cluster_start);
    } else {
      cluster_start &= QCOW2_OFFSET_MASK;
      if (!cluster_start) { /* unallocated means zero */
        bzero(buffer, length);
        return length;
      }

      if (ReadFile(buffer, length, cluster_start + offset_in_cluster) != (ssize_t)length) {
        return -1;
      }
    }
    return length;
  }

  ssize_t WriteCluster(void* buffer, off_t offset, size_t length) {
    uint64_t offset_in_cluster, l2_index;
    L2Table* l2_table = GetL2Table(true, offset, &offset_in_cluster, &l2_index, &length);
    MV_ASSERT(l2_table);

    uint64_t cluster_start = be64toh(l2_table->entries[l2_index]);
    uint64_t cluster_flags = cluster_start & QCOW2_OFLAGS_MASK;
    cluster_start &= QCOW2_OFFSET_MASK;

    if (cluster_flags & QCOW2_OFLAG_COPIED) {
      if (WriteFile(buffer, length, cluster_start + offset_in_cluster) != (ssize_t)length) {
        return -1;
      }
    } else {
      if (cluster_start) {
        MV_PANIC("writing to images with snapshots is not supported yet");
      }
      cluster_start = AllocateCluster();
      
      if (cluster_start == 0) {
        MV_PANIC("failed to allocate cluster");
        return -1;
      }
  
      l2_table->entries[l2_index] = htobe64(cluster_start | QCOW2_OFLAG_COPIED);
      l2_table->dirty = true;
  
      /* When should read the backend file ??? */
      if (WriteFile(buffer, length, cluster_start + offset_in_cluster) != (ssize_t)length) {
        MV_PANIC("failed to write image file at 0x%lx", cluster_start);
        return -1;
      }
    }

    return length;
  }

  ssize_t Write(void *buffer, off_t block, size_t block_count) {
    size_t bytes_written = 0;
    size_t total_bytes = block_count << block_size_shift_;
    size_t offset = block << block_size_shift_;
    uint8_t *ptr = (uint8_t*)buffer;
  
    while (bytes_written < total_bytes) {
      if (readonly_ || offset >= image_header_.size) {
        return bytes_written;
      }

      ssize_t ret = WriteCluster(ptr, offset, total_bytes - bytes_written);
      if (ret <= 0) {
        return ret;
      }

      bytes_written += ret;
      ptr += ret;
      offset += ret;
    }
    return bytes_written;
  }

  void FlushL2Tables () {
    int l2_table_dirty_count = 0;
    auto map = l2_table_cache_.map();
    for (auto it = map.begin(); it != map.end(); it++) {
      auto l2_table = it->second->second;
      if (l2_table->dirty) {
        WriteL2Table(l2_table);
        ++l2_table_dirty_count;
      }
    }
    if (l2_table_dirty_count) {
      MV_LOG("flush %lu L2 tables", l2_table_dirty_count);
    }
  }

  void FlushRefcountBlocks() {
    int refcount_block_dirty_count = 0;
    auto map = refcount_block_cache_.map();
    for (auto it = map.begin(); it != map.end(); it++) {
      auto rfb = it->second->second;
      if (rfb->dirty) {
        WriteRefcountBlock(rfb);
        ++refcount_block_dirty_count;
      }
    }
    if (refcount_block_dirty_count) {
      MV_LOG("flush %lu refcount blocks", refcount_block_dirty_count);
    }
  }

  void Flush() {
    if (readonly_) {
      return;
    }

    FlushL2Tables();
    FlushRefcountBlocks();
    if (l1_table_dirty_) {
      WriteL1Table();
    }
    if (refcount_table_dirty_) {
      WriteRefcountTable();
    }

    int ret = fsync(fd_);
    if (ret < 0) {
      MV_PANIC("failed to sync disk image, ret=%d", ret);
    }
  }

};

DECLARE_DISK_IMAGE(Qcow2);
