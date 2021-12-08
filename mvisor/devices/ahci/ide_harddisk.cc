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

#include "ide_storage.h"
#include <cstring>
#include "logger.h"
#include "disk_image.h"
#include "ahci_port.h"
#include "ata_interval.h"

static void padstr(char *str, const char *src, int len)
{
  int i, v;
  for(i = 0; i < len; i++) {
    if (*src)
      v = *src++;
    else
      v = ' ';
    str[i^1] = v;
  }
}

/*
 * Reference: https://read.seas.harvard.edu/cs161/2019/pdf/ata-atapi-8.pdf
 * The implemenation of ATA does not fully comply with the specification above.
 * It is a simplified version that just works.
 */
Harddisk::Harddisk() {
  type_ = kIdeStorageTypeHarddisk;

  drive_info_.world_wide_name = rand();
  sprintf(drive_info_.serial, "TC%05ld", drive_info_.world_wide_name);
  sprintf(drive_info_.model, "TENCLASS HARDDISK");
  sprintf(drive_info_.version, "1.0");

  multiple_sectors_ = 16;
  
  ata_handlers_[0x06] = [=] () { // DATA SET MANAGEMENT
    if (regs_.feature0 == 1) { // TRIM (Reference 7.5.6.1)
      io_.lba_mode = kIdeLbaMode28;
      io_.dma_status = 1;
      ReadLba();
      Ata_Trim();
    } else {
      AbortCommand();
      MV_PANIC("unknown data set management command=0x%x", regs_.feature0);
    }
  };

  ata_handlers_[0x20] = [=] () { // ATA_CMD_READ_SECTORS
    MV_PANIC("ATA command without DMA is no longer supported");
  };

  ata_handlers_[0x30] = [=] () { // ATA_CMD_WRITE_SECTORS
    MV_PANIC("ATA command without DMA is no longer supported");
  };

  ata_handlers_[0x40] = [=] () { // VERIFY
  };

  ata_handlers_[0xA1] = [=] () { // ATA_CMD_IDENTIFY_PACKET_DEVICE
    AbortCommand();
  };

  ata_handlers_[0xB0] = [=] () { // SMART
    AbortCommand();
  };

  ata_handlers_[0xC8] = [=] () { // ATA_CMD_READ_DMA
    io_.lba_mode = kIdeLbaMode28;
    io_.dma_status = 1;
    ReadLba();
    // MV_LOG("read 0x%lx sectors at 0x%lx", io_.lba_count, io_.lba_block);
    MV_ASSERT(io_.lba_count);
    Ata_ReadWriteSectors(false);
  };

  ata_handlers_[0xCA] = [=] () { // ATA_CMD_WRITE_DMA
    io_.lba_mode = kIdeLbaMode28;
    io_.dma_status = 1;
    ReadLba();
    // MV_LOG("write 0x%lx sectors at 0x%lx", io_.lba_count, io_.lba_block);
    MV_ASSERT(io_.lba_count);
    Ata_ReadWriteSectors(true);
  };

  ata_handlers_[0xE0] = [=] () { // STANDBYNOW1
  };

  ata_handlers_[0xE7] =          // FLUSH_CACHE
  ata_handlers_[0xEA] = [=] () { // FLUSH_CACHE_EXT
    image_->Flush();
  };

  ata_handlers_[0xF5] = [=] () { // SECURITY_FREEZE_LOCK
    AbortCommand();
  };
}

void Harddisk::InitializeGeometry() {
  ImageInformation info = image_->information();
  geometry_.sector_size = info.block_size;
  geometry_.total_sectors = info.total_blocks;
  geometry_.heads = 16;
  geometry_.sectors_per_cylinder = 63;
  geometry_.cylinders_per_heads = geometry_.total_sectors / (geometry_.sectors_per_cylinder * geometry_.heads);
}

void Harddisk::Connect() {
  IdeStorageDevice::Connect();

  if (image_) {
    InitializeGeometry();
  }
}

void Harddisk::ReadLba() {
  size_t block = regs_.lba0;
  size_t count = 0;

  switch (io_.lba_mode)
  {
  case kIdeLbaMode28:
    block |= (size_t)regs_.lba1 << 8;
    block |= (size_t)regs_.lba2 << 16;
    block |= (size_t)(regs_.device & 0xF) << 24;
    count = regs_.count0 == 0 ? 0x100 : regs_.count0;
    break;
  case kIdeLbaMode48:
    block |= (size_t)regs_.lba1 << 8;
    block |= (size_t)regs_.lba2 << 16;
    block |= (size_t)regs_.lba3 << 24;
    block |= (size_t)regs_.lba4 << 32;
    block |= (size_t)regs_.lba5 << 40;
    if (regs_.count0 == 0 && regs_.count1 == 0) {
      count = 0x10000;
    } else {
      count = (regs_.count1 << 8) | regs_.count0;
    }
    break;
  default:
    MV_PANIC("unsupported lba mode %x", io_.lba_mode);
    break;
  }
  io_.lba_block = block;
  io_.lba_count = count;
}

void Harddisk::WriteLba() {
  size_t block = io_.lba_block;
  size_t count = io_.lba_count;

  switch (io_.lba_mode)
  {
  case kIdeLbaMode28:
    regs_.device = (regs_.device & 0xF0) | ((block >> 24) & 0xF);
    regs_.lba2 = (block >> 16) & 0xFF;
    regs_.lba1 = (block >> 8) & 0xFF;
    regs_.lba0 = (block >> 0) & 0xFF;
    regs_.count0 = count == 0x100 ? 0: count & 0xFF;
    break;
  case kIdeLbaMode48:
    regs_.lba5 = (block >> 40) & 0xFF;
    regs_.lba4 = (block >> 32) & 0xFF;
    regs_.lba3 = (block >> 24) & 0xFF;
    regs_.lba2 = (block >> 16) & 0xFF;
    regs_.lba1 = (block >> 8) & 0xFF;
    regs_.lba0 = (block >> 0) & 0xFF;
    if (count == 0x10000) {
      regs_.count0 = regs_.count1 = 0;
    } else {
      regs_.count0 = count & 0xFF;
      regs_.count1 = (count >> 8) & 0xFF;
    }
    break;
  default:
    MV_PANIC("unsupported lba mode %x", io_.lba_mode);
    break;
  }
}

void Harddisk::Ata_ReadWriteSectors(bool is_write) {
  size_t vec_index = 0;
  size_t position = io_.lba_block * geometry_.sector_size;
  size_t remain_bytes = io_.lba_count * geometry_.sector_size;
  while (remain_bytes > 0 && vec_index < io_.vector.size()) {
    auto iov = io_.vector[vec_index];
  
    auto length = remain_bytes < iov.iov_len ? remain_bytes : iov.iov_len;
    if (is_write) {
      image_->Write(iov.iov_base, position, length);
    } else {
      image_->Read(iov.iov_base, position, length);
    }
    position += length;
    remain_bytes -= length;
    io_.nbytes += length;
    ++vec_index;
  }
  WriteLba();
}

void Harddisk::Ata_Trim() {
  for (auto vec : io_.vector) {
    for (size_t i = 0; i < vec.iov_len / sizeof(uint64_t); i++) {
      uint64_t value = ((uint64_t*)vec.iov_base)[i];
      size_t block = value & 0x0000FFFFFFFFFFFF;
      size_t count = value >> 48;
      if (block + count >= geometry_.total_sectors) {
        AbortCommand();
        return;
      }
      if (count) {
        image_->Trim(block * geometry_.sector_size, count * geometry_.sector_size);
      }
    }
  }
}

void Harddisk::Ata_IdentifyDevice() {
  uint16_t p[256] = { 0 };

  p[0] = 0x0040;
  p[1] = geometry_.cylinders_per_heads;
  p[3] = geometry_.heads;
  p[4] = 512 * geometry_.sectors_per_cylinder;
  p[5] = 512; /* Can we use larger sector size? */
  p[6] = geometry_.sectors_per_cylinder;
  padstr((char*)(p + 10), drive_info_.serial, 20);

  p[20] = 3; /* XXX: retired, remove ? */
  p[21] = 512; /* cache size in sectors */
  p[22] = 4; /* ecc bytes */
  padstr((char *)(p + 23), drive_info_.version, 8); /* firmware version */
  padstr((char *)(p + 27), drive_info_.model, 40); /* model */

  p[47] = 0x8000 | 16; /* Max multi sectors */

  p[48] = 1; /* dword I/O */
  p[49] = (1 << 11) | (1 << 9) | (1 << 8); /* DMA and LBA supported */
  p[51] = 0x200; /* PIO transfer cycle */
  p[52] = 0x200; /* DMA transfer cycle */
  p[53] = 1 | (1 << 1) | (1 << 2); /* words 54-58,64-70,88 are valid */
  p[54] = geometry_.cylinders_per_heads;
  p[55] = geometry_.heads;
  p[56] = geometry_.sectors_per_cylinder;
  
  uint oldsize = geometry_.cylinders_per_heads * geometry_.heads * geometry_.sectors_per_cylinder;
  p[57] = oldsize;
  p[58] = oldsize >> 16;
  if (multiple_sectors_)
      p[59] = 0x100 | multiple_sectors_;
  /* *(p + 60) := nb_sectors       -- see ide_identify_size */
  /* *(p + 61) := nb_sectors >> 16 -- see ide_identify_size */
  p[62] = 0x07; /* single word dma0-2 supported */
  p[63] = 0x07; /* mdma0-2 supported */
  p[64] = 0x03; /* pio3-4 supported */
  p[65] = 120;
  p[66] = 120;
  p[67] = 120;
  p[68] = 120;
  p[69] = (1 << 14); /* determinate TRIM behavior */

  /* NCQ supported */
  p[75] = 32 - 1; /* NCQ queues */
  p[76] = (1 << 8);

  p[80] = 0xf0; /* ata3 -> ata6 supported */
  p[81] = 0x16; /* conforms to ata5 */
  /* 14=NOP supported, 5=WCACHE supported, 0=SMART supported */
  p[82] = (1 << 14) | (1 << 5) | 0;
  /* 13=flush_cache_ext,12=flush_cache,10=lba48 */
  p[83] = (1 << 14) | (1 << 13) | (1 <<12) | (1 << 10);
  /* 14=set to 1, 8=has WWN, 1=SMART self test, 0=SMART error logging */
  if (drive_info_.world_wide_name) {
      p[84] = (1 << 14) | (1 << 8) | 0;
  } else {
      p[84] = (1 << 14) | 0;
  }
  /* 14 = NOP supported, 5=WCACHE enabled, 0=SMART feature set enabled */
  if (write_cache_) {
      p[85] = (1 << 14) | (1 << 5) | 1;
  } else {
      p[85] = (1 << 14) | 1;
  }
  /* 13=flush_cache_ext, 12=flush_cache, 10=lba48 */
  p[86] = (1 << 13) | (1 <<12) | (1 << 10);
  /* 14=set to 1, 8=has WWN, 1=SMART self test, 0=SMART error logging */
  if (drive_info_.world_wide_name) {
      p[87] = (1 << 14) | (1 << 8) | 0;
  } else {
      p[87] = (1 << 14) | 0;
  }
  p[88] = 0x3f | (1 << 13); /* udma5 set and supported */
  p[93] = 1 | (1 << 14) | 0x2000;

  /* p[106] = 0x6000 | get_physical_block_exp; */

  if (drive_info_.world_wide_name) {
      /* LE 16-bit words 111-108 contain 64-bit World Wide Name */
      p[108] = drive_info_.world_wide_name >> 48;
      p[109] = drive_info_.world_wide_name >> 32;
      p[110] = drive_info_.world_wide_name >> 16;
      p[111] = drive_info_.world_wide_name;
  }
  p[169] = 1; /* TRIM support */
  p[217] = 0; /* Nominal media rotation rate */

  /* update size */
  p[60] = geometry_.total_sectors;
  p[61] = geometry_.total_sectors >> 16;
  p[100] = geometry_.total_sectors;
  p[101] = geometry_.total_sectors >> 16;
  p[102] = geometry_.total_sectors >> 32;
  p[103] = geometry_.total_sectors >> 48;
  
  io_.nbytes = io_.buffer_size < 512 ? io_.buffer_size : 512;
  memcpy(io_.buffer, p, io_.nbytes);
}

DECLARE_DEVICE(Harddisk);
