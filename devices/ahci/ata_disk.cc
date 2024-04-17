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

#include "ata_disk.h"

#include <cstring>

#include "logger.h"
#include "disk_image.h"
#include "ahci_port.h"
#include "ata_internal.h"


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
AtaDisk::AtaDisk() {
  type_ = kAtaStorageTypeDisk;

  drive_info_.world_wide_name = 0;
  sprintf(drive_info_.serial, "TC%05ld", drive_info_.world_wide_name);
  sprintf(drive_info_.model, "Tenclass Disk");
  sprintf(drive_info_.version, "1.1");
  
  ata_handlers_[0x06] = [=] () { // DATA SET MANAGEMENT
    if (task_file_->feature0 == 1) { // TRIM (Reference 7.5.6.1)
      WaitForDma([this]() { Ata_Trim(); });
    } else {
      AbortCommand();
      MV_PANIC("unknown data set management command=0x%x", task_file_->feature0);
    }
  };

  ata_handlers_[0x20] =
  ata_handlers_[0x29] =
  ata_handlers_[0xC4] = [=] () { // ATA_CMD_READ_SECTORS
    size_t sectors = task_file_->command == 0x20 ? 1 : multiple_sectors_;

    ReadLba(task_file_->command == 0x29 ? kLbaMode48 : kLbaMode28);
    if (debug_) {
      MV_LOG("read 0x%lx sectors at 0x%lx", io_.lba_count, io_.lba_block);
    }
    Ata_PioReadSectors(sectors);
  };

  ata_handlers_[0x30] =
  ata_handlers_[0x39] =
  ata_handlers_[0xC5] = [=] () { // ATA_CMD_WRITE_SECTORS
    size_t sectors = task_file_->command == 0x30 ? 1 : multiple_sectors_;

    ReadLba(task_file_->command == 0x39 ? kLbaMode48 : kLbaMode28);
    if (debug_) {
      MV_LOG("write 0x%lx sectors at 0x%lx", io_.lba_count, io_.lba_block);
    }
    Ata_PioWriteSectors(sectors);
  };

  ata_handlers_[0x40] = [=] () { // VERIFY
  };

  ata_handlers_[0x91] = [=] () { // set drive geometry translation, obsolete since ATA6
  };

  ata_handlers_[0xA1] = [=] () { // ATA_CMD_IDENTIFY_PACKET_DEVICE
    AbortCommand();
  };

  ata_handlers_[0xB0] = [=] () { // SMART
    AbortCommand();
  };

  ata_handlers_[0xC6] = [=] () { // multiple sectors
    multiple_sectors_ = task_file_->count0 & 0xFF;
    if (debug_) {
      MV_LOG("set multiple sectors %d", multiple_sectors_);
    }
    /* update pio buffer size */
    io_.pio_buffer_size = multiple_sectors_ * geometry_.sector_size;
  };
  
  ata_handlers_[0x25] =          // ATA_CMD_READ_DMA_EXT
  ata_handlers_[0xC8] = [=] () { // ATA_CMD_READ_DMA
    ReadLba(task_file_->command == 0x25 ? kLbaMode48 : kLbaMode28);
    if (debug_) {
      MV_LOG("DMA read 0x%lx sectors at 0x%lx", io_.lba_count, io_.lba_block);
    }
    WaitForDma([this]() { Ata_DmaReadWriteSectors(false, io_.lba_count); });
  };

  ata_handlers_[0x35] =          // ATA_CMD_WRITE_DMA_EXT
  ata_handlers_[0xCA] = [=] () { // ATA_CMD_WRITE_DMA
    ReadLba(task_file_->command == 0x35 ? kLbaMode48 : kLbaMode28);
    if (debug_) {
      MV_LOG("DMA write 0x%lx sectors at 0x%lx", io_.lba_count, io_.lba_block);
    }

    WaitForDma([this]() { Ata_DmaReadWriteSectors(true, io_.lba_count); });
  };

  ata_handlers_[0xE1] = [=] () { // IDLEIMMEDIATE
  };

  ata_handlers_[0xE7] =          // FLUSH_CACHE
  ata_handlers_[0xEA] = [=] () { // FLUSH_CACHE_EXT
    io_async_ = true;
    ImageIoRequest request = {
      .type = kImageIoFlush
    };
    image_->QueueIoRequest(request, [this](ssize_t ret) {
      if (ret < 0) {
        task_file_->status |= ATA_CB_STAT_ERR;
      }
      EndCommand();
    });
  };
  
  ata_handlers_[0xEC] = [=] () { // ATA_CMD_IDENTIFY_DEVICE
    if (image_) {
      Ata_IdentifyDevice();
    }
  };

  ata_handlers_[0xF5] = [=] () { // SECURITY_FREEZE_LOCK
    AbortCommand();
  };
}

void AtaDisk::Connect() {
  AtaStorageDevice::Connect();

  if (image_) {
    ImageInformation info = image_->information();
    geometry_.sector_size = info.block_size;
    geometry_.total_sectors = info.total_blocks;
    geometry_.heads = 16;
    geometry_.sectors_per_cylinder = 63;
    geometry_.cylinders_per_heads = geometry_.total_sectors / (geometry_.sectors_per_cylinder * geometry_.heads);
    geometry_.cylinders_per_heads = std::min(geometry_.cylinders_per_heads, 16383ul);
  }
}

void AtaDisk::ReadLba(LbaMode mode) {
  size_t block = task_file_->lba0;
  size_t count = 0;

  if (!(task_file_->device & 0x40)) {
    io_.lba_mode = kLbaModeChs;
  } else {
    io_.lba_mode = mode;
  }

  switch (io_.lba_mode)
  {
  case kLbaModeChs:
    if (task_file_->lba0 == 0) {
      io_.lba_block = 0;
      io_.lba_count = 0;
      return;
    }
    block = (((size_t)task_file_->lba2 << 8) | task_file_->lba1) * geometry_.heads * geometry_.sectors_per_cylinder +
        size_t(task_file_->device & 0xF) * geometry_.sectors_per_cylinder + (task_file_->lba0 - 1);
    count = task_file_->count0 == 0 ? 0x100 : task_file_->count0;
    break;
  case kLbaMode28:
    block |= (size_t)task_file_->lba1 << 8;
    block |= (size_t)task_file_->lba2 << 16;
    block |= (size_t)(task_file_->device & 0xF) << 24;
    count = task_file_->count0 == 0 ? 0x100 : task_file_->count0;
    break;
  case kLbaMode48:
    block |= (size_t)task_file_->lba1 << 8;
    block |= (size_t)task_file_->lba2 << 16;
    block |= (size_t)task_file_->lba3 << 24;
    block |= (size_t)task_file_->lba4 << 32;
    block |= (size_t)task_file_->lba5 << 40;
    if (task_file_->count0 == 0 && task_file_->count1 == 0) {
      count = 0x10000;
    } else {
      count = (task_file_->count1 << 8) | task_file_->count0;
    }
    break;
  default:
    MV_PANIC("unsupported lba mode %x", io_.lba_mode);
    break;
  }

  io_.lba_block = block;
  io_.lba_count = count;
}

void AtaDisk::WriteLba() {
  size_t block = io_.lba_block;
  size_t count = io_.lba_count;

  if (!(task_file_->device & 0x40)) {
    io_.lba_mode = kLbaModeChs;
  }

  switch (io_.lba_mode)
  {
  case kLbaModeChs: {
    size_t cyl = block / (geometry_.heads * geometry_.sectors_per_cylinder);
    size_t r = block % (geometry_.heads * geometry_.sectors_per_cylinder);
    task_file_->lba2 = cyl >> 8;
    task_file_->lba1 = cyl & 0xFF;
    task_file_->device = (task_file_->device & ~0xF) | ((r / geometry_.sectors_per_cylinder) & 0xF);
    task_file_->lba0 = (r % geometry_.sectors_per_cylinder) + 1;
    task_file_->count0 = count == 0x100 ? 0: count & 0xFF;
    break;
  }
  case kLbaMode28:
    task_file_->device = (task_file_->device & 0xF0) | ((block >> 24) & 0xF);
    task_file_->lba2 = (block >> 16) & 0xFF;
    task_file_->lba1 = (block >> 8) & 0xFF;
    task_file_->lba0 = (block >> 0) & 0xFF;
    task_file_->count0 = count == 0x100 ? 0: count & 0xFF;
    break;
  case kLbaMode48:
    task_file_->lba5 = (block >> 40) & 0xFF;
    task_file_->lba4 = (block >> 32) & 0xFF;
    task_file_->lba3 = (block >> 24) & 0xFF;
    task_file_->lba2 = (block >> 16) & 0xFF;
    task_file_->lba1 = (block >> 8) & 0xFF;
    task_file_->lba0 = (block >> 0) & 0xFF;
    if (count == 0x10000) {
      task_file_->count0 = task_file_->count1 = 0;
    } else {
      task_file_->count0 = count & 0xFF;
      task_file_->count1 = (count >> 8) & 0xFF;
    }
    break;
  default:
    MV_PANIC("unsupported lba mode %x", io_.lba_mode);
    break;
  }
}

void AtaDisk::Ata_DmaReadWriteSectors(bool is_write, size_t sectors) {
  if (io_.lba_count == 0) {
    return;
  }

  if (sectors > io_.lba_count) {
    sectors = io_.lba_count;
  }

  io_async_ = true;
  size_t position = io_.lba_block * geometry_.sector_size;
  size_t remain_bytes = sectors * geometry_.sector_size;
  size_t vec_index = 0;
  ImageIoRequest request = {
    .type = is_write ? kImageIoWrite : kImageIoRead,
    .position = position,
    .length = 0
  };

  while (vec_index < io_.vector.size() && remain_bytes > 0) {
    auto& iov = io_.vector[vec_index];

    size_t length = remain_bytes < iov.iov_len ? remain_bytes : iov.iov_len;
    request.vector.emplace_back(iovec {
      .iov_base = iov.iov_base,
      .iov_len = length
    });

    request.length += length;
    remain_bytes -= length;
    vec_index++;
  }

  MV_ASSERT(request.length == sectors * geometry_.sector_size);
  io_.transfer_bytes = request.length;
  io_.lba_count -= sectors;
  io_.lba_block += sectors;

  image_->QueueIoRequest(request, [this, is_write, sectors](ssize_t ret) {
    if (ret <= 0) {
      task_file_->status |= ATA_CB_STAT_ERR;
      EndCommand();
    } else {
      WriteLba();
      StartTransfer(kTransferDataToHost, [this, is_write, sectors]() {
        Ata_DmaReadWriteSectors(is_write, sectors);
      });
    }
  });
}

void AtaDisk::Ata_PioReadSectors(size_t sectors) {
  if (io_.lba_count == 0) {
    return;
  }

  if (sectors > io_.lba_count) {
    sectors = io_.lba_count;
  }

  task_file_->status |= ATA_CB_STAT_SKC;
  io_async_ = true;
  size_t position = io_.lba_block * geometry_.sector_size;
  size_t remain_bytes = sectors * geometry_.sector_size;
  size_t length = remain_bytes < io_.buffer_size ? remain_bytes : io_.buffer_size;

  ImageIoRequest request = {
    .type = kImageIoRead,
    .position = position,
    .length = length
  };
  request.vector.emplace_back(iovec {
    .iov_base = io_.buffer,
    .iov_len = length
  });

  io_.transfer_bytes = length;
  io_.lba_count -= sectors;
  io_.lba_block += sectors;

  image_->QueueIoRequest(request, [this, sectors](ssize_t ret) {
    if (ret <= 0) {
      task_file_->status |= ATA_CB_STAT_ERR;
      EndCommand();
    } else {
      WriteLba();
      StartTransfer(kTransferDataToHost, [this, sectors]() {
        Ata_PioReadSectors(sectors);
      });
    }
  });
}

void AtaDisk::Ata_PioWriteSectors(size_t sectors) {
  if (io_.lba_count == 0) {
    /* We might got here from IO request, end the command manually */
    EndCommand();
    return;
  }

  if (sectors > io_.lba_count) {
    sectors = io_.lba_count;
  }

  size_t position = io_.lba_block * geometry_.sector_size;
  size_t remain_bytes = sectors * geometry_.sector_size;
  size_t length = remain_bytes < io_.buffer_size ? remain_bytes : io_.buffer_size;
  io_.transfer_bytes = length;

  StartTransfer(kTransferDataToDevice, [this, position, length, sectors]() {
    ImageIoRequest request = {
      .type = kImageIoWrite,
      .position = position,
      .length = length
    };
    request.vector.emplace_back(iovec {
      .iov_base = io_.buffer,
      .iov_len = length
    });

    io_.lba_count -= sectors;
    io_.lba_block += sectors;
    io_async_ = true;

    image_->QueueIoRequest(request, [this, sectors](ssize_t ret) {
      if (ret <= 0) {
        task_file_->status |= ATA_CB_STAT_ERR;
        EndCommand();
      } else {
        WriteLba();
        Ata_PioWriteSectors(sectors);
      }
    });
  });
}


void AtaDisk::Ata_Trim() {
  if (io_.buffer_size == 0) {
    return;
  }

  size_t total_bytes = 0;
  struct Chunk {
    size_t position;
    size_t length;
  };

  std::vector<Chunk> chunks;
  for (size_t i = 0; i < io_.buffer_size / sizeof(uint64_t); i++) {
    uint64_t value = ((uint64_t*)io_.buffer)[i];
    size_t block = value & 0x0000FFFFFFFFFFFF;
    size_t count = value >> 48;
    if (count == 0) {
      continue;
    }
    if (block + count >= geometry_.total_sectors) {
      AbortCommand();
      return;
    }
    size_t length = count * geometry_.sector_size;
    chunks.push_back(Chunk {
      .position = block * geometry_.sector_size,
      .length = length
    });
    total_bytes += length;
  }

  std::vector<ImageIoRequest> requests;
  for (auto chunk : chunks) {
    io_async_ = true;
    requests.emplace_back(ImageIoRequest {
      .type = kImageIoDiscard,
      .position = chunk.position,
      .length = chunk.length
    });
    io_.transfer_bytes += chunk.length;
  }

  image_->QueueMultipleIoRequests(std::move(requests), [this](auto ret) {
    if (ret < 0) {
      task_file_->status |= ATA_CB_STAT_ERR;
    }
    EndCommand();
  });
}

void AtaDisk::Ata_IdentifyDevice() {
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
  p[82] = (1 << 14) | (1 << 5) | 1;
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
  size_t nb_sectors_lba28 = geometry_.total_sectors;
  if (nb_sectors_lba28 >= 1 << 28) {
      nb_sectors_lba28 = (1 << 28) - 1;
  }
  p[60] = nb_sectors_lba28;
  p[61] = nb_sectors_lba28 >> 16;
  p[100] = geometry_.total_sectors;
  p[101] = geometry_.total_sectors >> 16;
  p[102] = geometry_.total_sectors >> 32;
  p[103] = geometry_.total_sectors >> 48;

  /* Physical sector size / Logical sector size */
  p[106] = 0x6000;
  
  io_.transfer_bytes = io_.buffer_size < 512 ? io_.buffer_size : 512;
  memcpy(io_.buffer, p, io_.transfer_bytes);
  StartTransfer(kTransferDataToHost, []() {});
}

DECLARE_DEVICE(AtaDisk);
