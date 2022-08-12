/* 
 * MVisor QCOW2 Disk Image
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

#include "qcow2.h"
#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>
#include <libgen.h>
#include <sys/stat.h>
#include <ctime>
#include <cstring>
#include <vector>
#include "logger.h"

#define REFCOUNT_CACHE_ITEMS        128
#define L2_CACHE_ITEMS              128
#define CLUSTER_CACHE_ITEMS         128

Qcow2Image::~Qcow2Image() {
  /* Flush caches if dirty */
  l2_cache_.Clear();
  rfb_cache_.Clear();
  cluster_cache_.Clear();

  if (fd_ != -1) {
    Flush();
    safe_close(&fd_);
  }

  if (copied_cluster_) {
    delete[] copied_cluster_;
  }

  if (backing_file_) {
    delete backing_file_;
  }
}

void Qcow2Image::Initialize() {
  if (readonly_) {
    fd_ = open(filepath_.c_str(), O_RDONLY);
  } else {
    fd_ = open(filepath_.c_str(), O_RDWR);
  }
  if (fd_ < 0)
    MV_PANIC("disk file not found: %s", filepath_.c_str());

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
      strncpy(temp, filepath_.c_str(), sizeof(temp) - 1);
      backing_filepath_ = std::string(dirname(temp)) + "/" + filename;
    }
    backing_file_ = new Qcow2Image();
    backing_file_->is_backing_file_ = true;
    backing_file_->readonly_ = true;
    backing_file_->filepath_ = backing_filepath_;
    backing_file_->Initialize();
  }
  // MV_LOG("open qcow2 %s file size=%ld", path.c_str(), image_size_);
}

void Qcow2Image::InitializeQcow2Header() {
  bzero(&image_header_, sizeof(image_header_));
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
  /* Version 3 features */
  be64_to_cpus(&image_header_.incompatible_features);
  be64_to_cpus(&image_header_.compatible_features);
  be64_to_cpus(&image_header_.autoclear_features);
  be32_to_cpus(&image_header_.refcount_order);
  be32_to_cpus(&image_header_.header_length);

  if (image_header_.magic != 0x514649FB) {
    MV_PANIC("File %s is not QCOW2 format", filepath_.c_str());
  }
  if (image_header_.version > 3) {
    MV_PANIC("Qcow2 file version=0x%x not supported", image_header_.version);
  }

  total_blocks_ = image_header_.size >> block_size_shift_;
  cluster_bits_ = image_header_.cluster_bits;
  cluster_size_ = 1 << cluster_bits_;
  l2_entries_ = cluster_size_ / sizeof(uint64_t);
  copied_cluster_ = new uint8_t[cluster_size_];

  /* For version 2, refcount bits is always 16 */
  refcount_bits_ = 16;
  rfb_entries_ = cluster_size_ * 8 / refcount_bits_;

  /* If there is no snapshot, we assume all refcounts <= 1 */
  if (image_header_.nb_snapshots) {
    MV_PANIC("Qcow2 file with snapshots is not supported yet");
  }
  if (image_header_.version == 3 && image_header_.compression_type > 1) {
    MV_PANIC("Unsupportted compression type=%d", image_header_.compression_type);
  }
}

/* FIXME: should we call pwrite for multiple times to write all data ??? */
ssize_t Qcow2Image::WriteFile(void* buffer, size_t length, off_t offset) {
  if (offset >= (ssize_t)image_header_.size) {
    MV_LOG("write overflow length=0x%lx, offset=0x%lx", length, offset);
    return 0;
  }
  ssize_t ret = pwrite(fd_, buffer, length, offset);
  MV_ASSERT(ret == (ssize_t)length);
  return ret;
}

/* FIXME: should we call pread for multiple times to read all data ??? */
ssize_t Qcow2Image::ReadFile(void* buffer, size_t length, off_t offset) {
  ssize_t ret = pread(fd_, buffer, length, offset);
  return ret;
}

void Qcow2Image::InitializeL1Table() {
  l1_table_.resize(image_header_.l1_size);
  bzero(l1_table_.data(), sizeof(uint64_t) * image_header_.l1_size);
  ReadFile(l1_table_.data(), sizeof(uint64_t) * image_header_.l1_size,
    image_header_.l1_table_offset
  );
}

void Qcow2Image::InitializeRefcountTable() {
  size_t refcount_bytes = image_header_.refcount_table_clusters * cluster_size_;
  refcount_table_.resize(refcount_bytes  / sizeof(uint64_t));
  bzero(refcount_table_.data(), refcount_bytes);
  ReadFile(refcount_table_.data(), refcount_bytes, image_header_.refcount_table_offset);
}

void Qcow2Image::WriteL1Table() {
  WriteFile(l1_table_.data(), l1_table_.size() * sizeof(uint64_t),
    image_header_.l1_table_offset);
  l1_table_dirty_ = false;
}

void Qcow2Image::WriteRefcountTable() {
  WriteFile(refcount_table_.data(), refcount_table_.size() * sizeof(uint64_t),
    image_header_.refcount_table_offset);
  refcount_table_dirty_ = false;
}

void Qcow2Image::WriteL2Table(L2Table* l2_table) {
  WriteFile(l2_table->entries, l2_entries_ * sizeof(uint64_t),
    l2_table->offset_in_file);
  l2_table->dirty = false;
}

void Qcow2Image::WriteRefcountBlock(RefcountBlock* rfb) {
  WriteFile(rfb->entries, rfb_entries_ * sizeof(uint16_t), rfb->offset_in_file);
  rfb->dirty = false;
}

void Qcow2Image::InitializeLruCache() {
  rfb_cache_.Initialize(REFCOUNT_CACHE_ITEMS, [this](auto offset_in_file, auto rfb) {
    MV_UNUSED(offset_in_file);
    if (rfb->dirty) {
      WriteRefcountBlock(rfb);
    }
    // MV_LOG("free rfb 0x%lx", rfb->offset_in_file);
    free(rfb);
  });
  l2_cache_.Initialize(L2_CACHE_ITEMS, [this](auto offset_in_file, auto l2_table) {
    MV_UNUSED(offset_in_file);
    if (l2_table->dirty) {
      WriteL2Table(l2_table);
    }
    // MV_LOG("free l2_table 0x%lx", l2_table->offset_in_file);
    free(l2_table);
  });
  cluster_cache_.Initialize(CLUSTER_CACHE_ITEMS, [this](auto offset_in_file, auto data) {
    MV_UNUSED(offset_in_file);
    delete data;
  });
}

RefcountBlock* Qcow2Image::NewRefcountBlock(uint64_t block_offset) {
  size_t size = sizeof(RefcountBlock) + rfb_entries_ * sizeof(uint16_t);
  RefcountBlock* block = (RefcountBlock*)malloc(size);
  bzero(block, size);
  block->dirty = false;
  block->offset_in_file = block_offset;
  return block;
}

RefcountBlock* Qcow2Image::GetRefcountBlock(uint64_t cluster_index, uint64_t* rfb_index, bool allocate) {
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
    block_offset = cluster_index << cluster_bits_;
    rfb = NewRefcountBlock(block_offset);
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

void Qcow2Image::FreeCluster(uint64_t start) {
  uint64_t rfb_index;
  uint64_t cluster_index = start >> cluster_bits_;
  RefcountBlock* rfb = GetRefcountBlock(cluster_index, &rfb_index, false);
  if (rfb == nullptr) {
    MV_PANIC("Try to free an unallocated cluster 0x%lx", start);
    return;
  }

  rfb->entries[rfb_index] = htobe16(be16toh(rfb->entries[rfb_index]) - 1);
  rfb->dirty = true;
  if (rfb->entries[rfb_index] == 0 && cluster_index < free_cluster_index_) {
    free_cluster_index_ = cluster_index;
  }
}

/* free_cluster_index_ is initialized to zero and record last position. */
uint64_t Qcow2Image::AllocateCluster() {
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
  rfb->entries[rfb_index] = htobe16(be16toh(rfb->entries[rfb_index]) + 1);
  rfb->dirty = true;
  return cluster_index << cluster_bits_;
}

L2Table* Qcow2Image::NewL2Table(uint64_t l2_offset) {
  size_t size = sizeof(L2Table) + l2_entries_ * sizeof(uint64_t);
  L2Table* table = (L2Table*)malloc(size);
  bzero(table, size);
  table->dirty = false;
  table->offset_in_file = l2_offset;
  return table;
}

L2Table* Qcow2Image::ReadL2Table(uint64_t l2_offset) {
  L2Table* table;
  if (l2_cache_.Get(l2_offset, table)) {
    return table;
  }

  table = NewL2Table(l2_offset);
  ReadFile(table->entries, l2_entries_ * sizeof(uint64_t), table->offset_in_file);

  l2_cache_.Put(table->offset_in_file, table);
  return table;
}

L2Table* Qcow2Image::GetL2Table(bool is_write, off_t pos, uint64_t* offset_in_cluster, uint64_t* l2_index, size_t* length) {
  *offset_in_cluster = pos % cluster_size_;
  if (*length > cluster_size_ - *offset_in_cluster) {
    *length = cluster_size_ - *offset_in_cluster;
  }

  uint64_t cluster_index = pos >> cluster_bits_;
  uint64_t l1_index = cluster_index / l2_entries_;
  *l2_index = cluster_index % l2_entries_;
  
  uint64_t l2_offset = be64toh(l1_table_[l1_index]);
  if (l2_offset & QCOW2_OFLAG_COPIED) { /* L2 already allocated, read from current file */
    l2_offset &= ~QCOW2_OFLAG_COPIED;
    if (!l2_offset) { /* copied l1 entry must be valid */
      MV_PANIC("l2_offset is not valid");
    }
    return ReadL2Table(l2_offset);
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
    return l2_table;
  } else { /* L2 not allocated, but should read from backing file if possible */
    return nullptr;
  }
  return nullptr;
}

/* The return value is always less than or equal to cluster size */
ssize_t Qcow2Image::ReadCluster(void* buffer, off_t pos, size_t length, bool no_zero) {
  uint64_t offset_in_cluster, l2_index;
  auto l2_table = GetL2Table(false, pos, &offset_in_cluster, &l2_index, &length);
  if (l2_table == nullptr || !l2_table->entries[l2_index]) {
    if (backing_file_ == nullptr) { /* Reading at unallocated space always return zero??? */
      if (no_zero)
        return -2;
      bzero(buffer, length);
      return length;
    }
    return backing_file_->ReadCluster(buffer, pos, length);
  }

  uint64_t l2_entry = be64toh(l2_table->entries[l2_index]);
  uint64_t cluster_descriptor = l2_entry & QCOW2_DESCRIPTOR_MASK;

  if (l2_entry & QCOW2_OFLAG_COMPRESSED) { /* Compressed descriptor */
    MV_ASSERT(image_header_.compression_type == kCompressionTypeZstd);

    uint64_t x = (62 - (cluster_bits_ - 8));
    uint64_t mask = (1ULL << (cluster_bits_ - 8)) - 1;
    uint64_t sectors = ((cluster_descriptor >> x) & mask) + 1;
    uint64_t compressed_length = sectors * QCOW2_COMPRESSED_SECTOR_SIZE -
      (cluster_descriptor & ~QCOW2_COMPRESSED_SECTOR_MASK);
    uint64_t host_offset = cluster_descriptor & ((1ULL << x) - 1);

    uint8_t* decompressed = nullptr;
    if (!cluster_cache_.Get(host_offset, decompressed)) {
      if (compressed_.size() < compressed_length)
        compressed_.resize(compressed_length);
      ssize_t bytes_read = ReadFile(compressed_.data(), compressed_length, host_offset);
      if (bytes_read < 0) {
        return bytes_read;
      }

      decompressed = new uint8_t[cluster_size_];
      auto ret = zstd_decompress(compressed_.data(), compressed_length, decompressed, cluster_size_);
      if (ret < 0) {
        delete[] decompressed;
        MV_ERROR("failed to decompressed length=0x%x ret=%d", compressed_length, ret);
        return ret;
      }
      cluster_cache_.Put(host_offset, decompressed);
    }
    memcpy(buffer, decompressed + offset_in_cluster, length);
  } else { /* Standard descriptor */
    if (cluster_descriptor & 1) { // Bit 0 means zero
      bzero(buffer, length);
      return length;
    }

    uint64_t host_offset = cluster_descriptor & QCOW2_STANDARD_OFFSET_MASK;

    ssize_t bytes_read = ReadFile(buffer, length, host_offset + offset_in_cluster);
    if (bytes_read < 0) {
      return bytes_read;
    }
    if ((size_t)bytes_read < length) {
      /* Reach the end of file??? */
      bzero((uint8_t*)buffer + bytes_read, length - bytes_read);
    }
  }
  return length;
}

/* The return value is always less than or equal to cluster size */
ssize_t Qcow2Image::WriteCluster(void* buffer, off_t pos, size_t length) {
  uint64_t offset_in_cluster, l2_index;
  L2Table* l2_table = GetL2Table(true, pos, &offset_in_cluster, &l2_index, &length);
  MV_ASSERT(l2_table);

  uint64_t l2_entry = be64toh(l2_table->entries[l2_index]);
  uint64_t cluster_descriptor = l2_entry & QCOW2_DESCRIPTOR_MASK;

  /* Writing compressed cluster is not supported */
  MV_ASSERT(!(l2_entry & QCOW2_OFLAG_COMPRESSED));
  uint64_t host_offset = cluster_descriptor & QCOW2_STANDARD_OFFSET_MASK;

  if (l2_entry & QCOW2_OFLAG_COPIED) {
    if (WriteFile(buffer, length, host_offset + offset_in_cluster) != (ssize_t)length) {
      return -1;
    }
  } else {
    if (host_offset) {
      MV_PANIC("writing to images with snapshots is not supported yet");
    }
    host_offset = AllocateCluster();
    if (host_offset == 0) {
      MV_ERROR("failed to allocate cluster");
      return -1;
    }

    l2_table->entries[l2_index] = htobe64(host_offset | QCOW2_OFLAG_COPIED);
    l2_table->dirty = true;

    /* If not writing the whole cluster, we should read the original data from the backing file
      * Always read the whole cluster without zeroing data if cluster exists in backing file
      */
    if (backing_file_ && !(offset_in_cluster == 0 && length == cluster_size_)) {
      auto bytes = backing_file_->ReadCluster(copied_cluster_, pos - offset_in_cluster, cluster_size_, true);
      if (bytes > 0) { // Check if exists
        MV_ASSERT(bytes == (ssize_t)cluster_size_); // Make sure we have a whole cluster
        memcpy(copied_cluster_ + offset_in_cluster, buffer, length);
        if (WriteFile(copied_cluster_, cluster_size_, host_offset) != (ssize_t)cluster_size_) {
          MV_PANIC("failed to copy cluster at pos=0x%lx length=0x%lx", pos, length);
        }
        return length; // Always return length of dirty data
      }
    }

    if (WriteFile(buffer, length, host_offset + offset_in_cluster) != (ssize_t)length) {
      MV_PANIC("failed to write image file pos=0x%lx host_offset=0x%lx offset=0x%lx length=0x%lx",
        pos, host_offset, offset_in_cluster, length);
      return -1;
    }
  }

  return length;
}

/* The OS use DISCARD command to inform us some disk regions are freed
  * To recycle these regions, clear the L2 table entry, and set the refcount to 0
  * The return value is always less than or equal to cluster size */
ssize_t Qcow2Image::DiscardCluster(off_t pos, size_t length) {
  uint64_t offset_in_cluster, l2_index;
  auto l2_table = GetL2Table(false, pos, &offset_in_cluster, &l2_index, &length);
  if (l2_table == nullptr) {
    return length;
  }

  if (offset_in_cluster > 0 || length < cluster_size_) {
    // MV_LOG("do nothing, not aligned to 64k pos=0x%lx length=0x%lx", pos, length);
    return length;
  }

  uint64_t l2_entry = be64toh(l2_table->entries[l2_index]);
  uint64_t cluster_descriptor = l2_entry & QCOW2_DESCRIPTOR_MASK;

  /* Writing compressed cluster is not supported */
  MV_ASSERT(!(l2_entry & QCOW2_OFLAG_COMPRESSED));
  uint64_t host_offset = cluster_descriptor & QCOW2_STANDARD_OFFSET_MASK;

  if ((cluster_descriptor & 1) || host_offset == 0) {
    return length;
  }

  FreeCluster(host_offset);
  l2_table->entries[l2_index] = be64toh(0);
  l2_table->dirty = true;
  return length;
}

ssize_t Qcow2Image::BlockIo(void *buffer, off_t position, size_t length, ImageIoType type) {
  size_t offset = 0;
  uint8_t *ptr = (uint8_t*)buffer;

  while (offset < length) {
    if ((uint64_t)position >= image_header_.size) {
      return offset;
    }

    ssize_t ret;
    switch (type) {
    case kImageIoRead:
      ret = ReadCluster(ptr, position, length - offset);
      break;
    case kImageIoWrite:
      ret = WriteCluster(ptr, position, length - offset);
      break;
    case kImageIoDiscard:
    case kImageIoWriteZeros:
      ret = DiscardCluster(position, length - offset);
      break;
    default:
      ret = -EINVAL;
      MV_PANIC("invalid type=%d", type);
    }
    if (ret <= 0) {
      return ret;
    }

    offset += ret;
    ptr += ret;
    position += ret;
  }
  return offset;
}

void Qcow2Image::FlushL2Tables () {
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

void Qcow2Image::FlushRefcountBlocks() {
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

ssize_t Qcow2Image::Read(void *buffer, off_t position, size_t length) {
  return BlockIo(buffer, position, length, kImageIoRead);
}

ssize_t Qcow2Image::Write(void *buffer, off_t position, size_t length) {
  return BlockIo(buffer, position, length, kImageIoWrite);
}

ssize_t Qcow2Image::Discard(off_t position, size_t length, bool write_zeros) {
  return BlockIo(nullptr, position, length, write_zeros ? kImageIoWriteZeros : kImageIoDiscard);
}

ssize_t Qcow2Image::Flush() {
  if (readonly_) {
    return 0;
  }

  FlushL2Tables();
  FlushRefcountBlocks();
  if (l1_table_dirty_) {
    WriteL1Table();
  }
  if (refcount_table_dirty_) {
    WriteRefcountTable();
  }

  return fsync(fd_);
}

DECLARE_DISK_IMAGE(Qcow2Image);
