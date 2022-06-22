/* 
 * MVisor QCOW2 Disk Image
 * Copyright (C) 2021-2022 Terrence <terrence@tenclass.com>
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

#ifndef _MVISOR_IMAGES_QCOW2_H
#define _MVISOR_IMAGES_QCOW2_H

#include "disk_image.h"
#include "lru_cache.h"


#define QCOW2_OFLAG_COPIED            (1UL << 63)
#define QCOW2_OFLAG_COMPRESSED        (1UL << 62)
#define QCOW2_OFLAGS_MASK             (QCOW2_OFLAG_COPIED | QCOW2_OFLAG_COMPRESSED)
#define QCOW2_OFFSET_MASK             (~QCOW2_OFLAGS_MASK)
#define QCOW2_COMPRESSED_SECTOR_SIZE  512
#define QCOW2_COMPRESSED_SECTOR_MASK  (~(QCOW2_COMPRESSED_SECTOR_SIZE - 1LL))


static inline void be32_to_cpus(uint32_t* x) {
  *x = be32toh(*x);
}
static inline void be64_to_cpus(uint64_t* x) {
  *x = be64toh(*x);
}

enum Qcow2CompressionType {
  kCompressionTypeZlib = 0,
  kCompressionTypeZstd = 1
};

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

  /* Version 3 */
  uint64_t incompatible_features;
  uint64_t compatible_features;
  uint64_t autoclear_features;
  uint32_t refcount_order;
  uint32_t header_length;
  uint8_t  compression_type;
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

class Qcow2Image : public DiskImage {
 private:
  int fd_ = -1;
  size_t block_size_shift_ = 9; // block size = 4KB
  size_t total_blocks_ = 0;
  size_t image_size_ = 0;
  size_t cluster_size_;
  size_t cluster_bits_;

  size_t l2_entries_;
  size_t rfb_entries_;
  size_t refcount_bits_;

  uint64_t    free_cluster_index_ = 0;
  uint8_t*    copied_cluster_ = nullptr;
  std::string compressed_;

  std::vector<uint64_t> l1_table_;
  std::vector<uint64_t> refcount_table_;
  bool l1_table_dirty_ = false;
  bool refcount_table_dirty_ = false;

  SimpleLRUCache<uint64_t, L2Table*>        l2_cache_;
  SimpleLRUCache<uint64_t, RefcountBlock*>  rfb_cache_;
  SimpleLRUCache<uint64_t, uint8_t*>        cluster_cache_;

  Qcow2Header image_header_;
  std::string backing_filepath_;
  Qcow2Image* backing_file_ = nullptr;
  bool        is_backing_file_ = false;

 private:
  void InitializeQcow2Header();
  ssize_t WriteFile(void* buffer, size_t length, off_t offset);
  ssize_t ReadFile(void* buffer, size_t length, off_t offset);
  void InitializeL1Table();
  void InitializeRefcountTable ();
  void WriteL1Table();
  void WriteRefcountTable();
  void WriteL2Table(L2Table* l2_table);
  void WriteRefcountBlock(RefcountBlock* rfb);
  void InitializeLruCache();
  RefcountBlock* NewRefcountBlock(uint64_t block_offset);
  RefcountBlock* GetRefcountBlock(uint64_t cluster_index, uint64_t* rfb_index, bool allocate);
  void FreeCluster(uint64_t start);
  uint64_t AllocateCluster();
  L2Table* NewL2Table(uint64_t l2_offset);
  L2Table* ReadL2Table(uint64_t l2_offset);
  L2Table* GetL2Table(bool is_write, off_t pos, uint64_t* offset_in_cluster, uint64_t* l2_index, size_t* length);
  ssize_t ReadCluster(void* buffer, off_t pos, size_t length, bool no_zero = false);
  ssize_t WriteCluster(void* buffer, off_t pos, size_t length);
  ssize_t DiscardCluster(off_t pos, size_t length);
  ssize_t BlockIo(void *buffer, off_t position, size_t length, ImageIoType type);
  void FlushL2Tables ();
  void FlushRefcountBlocks();

 public:
  virtual ~Qcow2Image();
  virtual void Initialize();
  virtual ssize_t Read(void *buffer, off_t position, size_t length);
  virtual ssize_t Write(void *buffer, off_t position, size_t length);
  virtual ssize_t Discard(off_t position, size_t length, bool write_zeros);
  virtual ssize_t Flush();

  virtual ImageInformation information() {
    return ImageInformation {
      .block_size = 1UL << block_size_shift_,
      .total_blocks = total_blocks_
    };
  }

  static void CreateEmptyImage(std::string path, size_t disk_size);
  static void CreateImageWithBackingFile(std::string path, std::string backing_path);
};

#endif // _MVISOR_IMAGES_QCOW2_H
