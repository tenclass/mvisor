#include "disk_image.h"
#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <ctime>
#include <cstring>
#include <vector>
#include <map>
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
  uint64_t    offset;
  bool        dirty;
  time_t      last_access_time;
  uint64_t    entries[];
};

class Qcow2 : public DiskImage {
 private:
  int fd_ = -1;
  bool readonly_ = true;
  size_t block_size_shift_ = 9; // 512 block size
  size_t total_blocks_ = 0;
  size_t image_size_ = 0;
  size_t cluster_size_;
  size_t l2_entries_;

  std::vector<uint64_t> l1_table_;
  std::vector<uint64_t> refcount_table_;
  std::map<uint64_t, L2Table*> l2_table_cache_;

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
    InitializeQcow2L1Table();
    InitializeQcow2RefcountTable();
  }

  void InitializeQcow2Header() {
    /* Read the image header at offset 0 */
    pread(fd_, &image_header_, sizeof(image_header_), 0);
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

    total_blocks_ = image_header_.size >> block_size_shift_;
    cluster_size_ = 1 << image_header_.cluster_bits;
    l2_entries_ = cluster_size_ / sizeof(uint64_t);
  }

  void InitializeQcow2L1Table() {
    l1_table_.resize(image_header_.l1_size);
    pread(fd_, l1_table_.data(),
      sizeof(uint64_t) * image_header_.l1_size,
      image_header_.l1_table_offset
    );
  }

  void InitializeQcow2RefcountTable () {
    size_t refcount_bytes = image_header_.refcount_table_clusters * cluster_size_;
    refcount_table_.resize(refcount_bytes  / sizeof(uint64_t));
    pread(fd_, refcount_table_.data(), refcount_bytes, image_header_.refcount_table_offset);
  }


  L2Table* NewL2Table(uint64_t l2_table_offset) {
    size_t size = sizeof(L2Table) + l2_entries_ * sizeof(uint64_t);
    L2Table* table = (L2Table*)malloc(size);
    table->dirty = false;
    table->last_access_time = time(nullptr);
    table->offset = l2_table_offset;
    return table;
  }

  L2Table* ReadL2Table(uint64_t l2_table_offset) {
    auto it = l2_table_cache_.find(l2_table_offset);
    if (it != l2_table_cache_.end()) {
      return it->second;
    }

    L2Table* table = NewL2Table(l2_table_offset);
    pread(fd_, table->entries, l2_entries_ * sizeof(uint64_t), l2_table_offset);

    l2_table_cache_[l2_table_offset] = table;
    return table;
  }
  
  ssize_t ReadCluster(void* buffer, off_t offset, size_t length) {
    off_t l1_index = (offset / cluster_size_) / l2_entries_;
    off_t l2_index = (offset / cluster_size_) % l2_entries_;
    off_t offset_in_cluster = offset % cluster_size_;
    if (length > cluster_size_ - offset_in_cluster) {
      length = cluster_size_ - offset_in_cluster;
    }

    uint64_t l2_table_offset = be64toh(l1_table_[l1_index]);
    l2_table_offset &= ~QCOW2_OFLAG_COPIED;
    if (!l2_table_offset) { /*unallocated means zero */
      bzero(buffer, length);
      return length;
    }

    L2Table* l2_table = ReadL2Table(l2_table_offset);
    uint64_t cluster = be64toh(l2_table->entries[l2_index]);
    if (cluster & QCOW2_OFLAG_COMPRESSED) {
      MV_PANIC("not supported compressed offset=0x%lx cluster=0x%lx", offset, cluster);
    } else {
      cluster &= QCOW2_OFFSET_MASK;
      if (!cluster) { /*unallocated means zero */
        bzero(buffer, length);
        return length;
      }

      if (pread(fd_, buffer, length, cluster + offset_in_cluster) < 0) {
        return -1;
      }
    }
    return length;
  }

  ssize_t Read(void *buffer, off_t block, size_t block_count) {
    size_t bytes_read = 0;
    size_t total_bytes = block_count << block_size_shift_;
    uint8_t *ptr = (uint8_t*)buffer;
  
    while (bytes_read < total_bytes) {
      size_t offset = block << block_size_shift_;
      if (offset >= image_header_.size) {
        return -1;
      }

      ssize_t ret = ReadCluster(ptr, offset, total_bytes - bytes_read);
      if (ret <= 0) {
        return -1;
      }

      bytes_read += ret;
      ptr += ret;
      block += ret >> block_size_shift_;
    }
    return block_count;
  }

  ssize_t Write(void *buffer, off_t block, size_t block_count) {
    return 0;
  }

  void Flush() {
    int ret = fsync(fd_);
    if (ret < 0) {
      MV_PANIC("failed to sync disk image, ret=%d", ret);
    }
  }

};

DECLARE_DISK_IMAGE(Qcow2);
