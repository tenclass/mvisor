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
#include <cstdio>
#include <cstring>
#include "logger.h"

struct HeaderExtension {
  uint32_t  type;
  uint32_t  length;
  uint8_t   data[0];
} __attribute__((packed));

struct FeatureName {
  uint8_t   type;
  uint8_t   bit;
  char      name[46];
} __attribute__((packed));

static FeatureName default_features[] = {
  { 0, 0, "dirty bit" },
  { 0, 1, "corrupt bit" },
  { 0, 2, "external data file" },
  { 0, 3, "compression type" },
  { 0, 4, "extended L2 entries" },
  { 1, 0, "lazy refcounts" },
  { 2, 0, "bitmaps" },
  { 2, 1, "raw external data" }
};


void Qcow2Image::CreateEmptyImage(std::string path, size_t disk_size) {
  uint cluster_bits = 0x10;
  size_t cluster_size = 1 << cluster_bits;
  FILE* fp = fopen(path.c_str(), "wb");
  if (fp == nullptr) {
    MV_PANIC("failed to create image file: %s", path.c_str());
  }

  /* Build header */
  Qcow2Header header;
  bzero(&header, sizeof(header));
  header.magic = htobe32(0x514649FB);
  header.version = htobe32(3);
  header.cluster_bits = htobe32(cluster_bits);
  header.size = htobe64(disk_size);
  header.l1_size = htobe32(disk_size / cluster_size / (cluster_size / sizeof(uint64_t)));
  header.l1_table_offset = htobe64(cluster_size * 3);
  header.refcount_table_offset = htobe64(cluster_size * 1);
  header.refcount_table_clusters = htobe32(1);
  header.refcount_order = htobe32(4);
  header.header_length = htobe32(0x70);
  fwrite(&header, sizeof(header), 1, fp);

  /* Seek to extensions */
  fseek(fp, 0x70, SEEK_SET);

  HeaderExtension feature_extension;
  feature_extension.type = htobe32(0x6803F857);
  feature_extension.length = htobe32(sizeof(default_features));

  fwrite(&feature_extension, sizeof(feature_extension), 1, fp);
  fwrite(default_features, sizeof(default_features), 1, fp);

  /* Seek to refcount table */
  fseek(fp, cluster_size * 1, SEEK_SET);
  uint64_t refcount_table_entry = htobe64(cluster_size * 2);
  fwrite(&refcount_table_entry, sizeof(refcount_table_entry), 1, fp);

  /* Seek to refcount block */
  fseek(fp, cluster_size * 2, SEEK_SET);
  /* We have allocated 4 clusters */
  for (int i = 0; i < 4; i++) {
    uint16_t refcount_block_entry = htobe16(1);
    fwrite(&refcount_block_entry, sizeof(refcount_block_entry), 1, fp);
  }

  /* Seek to L1 table */
  fseek(fp, cluster_size * 3, SEEK_SET);

  /* Done */
  fclose(fp);
}

void Qcow2Image::CreateImageWithBackingFile(std::string path, std::string backing_path) {
  FILE* fin = fopen(backing_path.c_str(), "rb");
  if (fin == nullptr) {
    MV_PANIC("failed to open QCOW2 file: %s", backing_path.c_str());
  }
  FILE* fout = fopen(path.c_str(), "wb");
  if (fout == nullptr) {
    MV_PANIC("failed to create QCOW2 file: %s", path.c_str());
  }

  /* Read header */
  Qcow2Header backing_header;
  MV_ASSERT(fread(&backing_header, sizeof(Qcow2Header), 1, fin) == 1);
  if (be32toh(backing_header.version) != 3) {
    MV_PANIC("file %s version %u is not supported", backing_path.c_str(), be32toh(backing_header.version));
  }
  uint32_t header_length = be32toh(backing_header.header_length);
  uint32_t cluster_size = 1 << be32toh(backing_header.cluster_bits);

  /* Copy header extensions */
  fseek(fin, header_length, SEEK_SET);
  fseek(fout, header_length, SEEK_SET);
  bool wrote_backing_format = false;
  while (true) {
    HeaderExtension extension;
    MV_ASSERT(fread(&extension, sizeof(extension), 1, fin) == 1);
    if (extension.type == 0)
      break;
    MV_ASSERT(fwrite(&extension, sizeof(extension), 1, fout) == 1);
    uint32_t data_length = be32toh(extension.length);
    if (data_length & 7)
      data_length += 8 - (data_length & 7);
    uint8_t data[data_length];
    MV_ASSERT(fread(data, data_length, 1, fin) == 1);
    MV_ASSERT(fwrite(data, data_length, 1, fout) == 1);
    if (be32toh(extension.type) == 0xE2792ACA) {
      wrote_backing_format = true;
    }
  }

  if (!wrote_backing_format) {
    struct {
      HeaderExtension extension;
      char data[8];
    } ext;
    bzero(&ext, sizeof(ext));
    ext.extension.type = htobe32(0xE2792ACA),
    ext.extension.length = htobe32(5);

    strcpy(ext.data, "qcow2");
    fwrite(&ext, sizeof(ext), 1, fout);
  }

  /* Write backing file path */
  off_t pos = ftell(fout);
  if (pos & 0xF) {
    pos += 16 - (pos & 0xF);
  }
  fseek(fout, pos, SEEK_SET);
  fwrite(backing_path.data(), backing_path.length(), 1, fout);
  
  /* Write image header */
  uint8_t buffer[header_length];
  fseek(fin, 0, SEEK_SET);
  fseek(fout, 0, SEEK_SET);
  MV_ASSERT(fread(buffer, header_length, 1, fin) == 1);
  auto header = (Qcow2Header*)buffer;
  header->backing_file_offset = htobe64(pos);
  header->backing_file_size = htobe32(backing_path.length());
  MV_ASSERT(fwrite(buffer, header_length, 1, fout) == 1);

  /* Seek to refcount table */
  fseek(fout, cluster_size * 1, SEEK_SET);
  uint64_t refcount_table_entry = htobe64(cluster_size * 2);
  MV_ASSERT(fwrite(&refcount_table_entry, sizeof(refcount_table_entry), 1, fout) == 1);

  /* Seek to refcount block */
  fseek(fout, cluster_size * 2, SEEK_SET);
  /* We have allocated 4 clusters */
  for (int i = 0; i < 4; i++) {
    uint16_t refcount_block_entry = htobe16(1);
    MV_ASSERT(fwrite(&refcount_block_entry, sizeof(refcount_block_entry), 1, fout) == 1);
  }

  /* Seek to L1 table */
  fseek(fout, cluster_size * 3, SEEK_SET);

  /* Done */
  fclose(fin);
  fclose(fout);
}
