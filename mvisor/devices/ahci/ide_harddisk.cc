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

Harddisk::Harddisk() {
  type_ = kIdeStorageTypeHarddisk;

  drive_info_.world_wide_name = rand();
  sprintf(drive_info_.serial, "TC%05ld", drive_info_.world_wide_name);
  sprintf(drive_info_.model, "TENCLASS HARDDISK");
  sprintf(drive_info_.version, "1.0");

  multiple_sectors_ = 16;
}

void Harddisk::Connect() {
  IdeStorageDevice::Connect();

  InitializeGemometry();
}

void Harddisk::InitializeGemometry() {
  gemometry_.sector_size = image_->sector_size();
  gemometry_.total_sectors = image_->sectors();
  gemometry_.heads = 16;
  gemometry_.sectors_per_cylinder = 63;
  gemometry_.cylinders_per_heads = gemometry_.total_sectors / (gemometry_.sectors_per_cylinder * gemometry_.heads);
}

void Harddisk::ReadLba() {
  auto regs = port_->registers();
  auto io = port_->io();
  size_t position = regs->lba0;
  size_t count = 0;

  switch (io->lba_mode)
  {
  case kIdeLbaMode28:
    position |= (size_t)regs->lba1 << 8;
    position |= (size_t)regs->lba2 << 16;
    position |= (size_t)(regs->device & 0xF) << 24;
    count = regs->count0 == 0 ? 0x100 : regs->count0;
    break;
  case kIdeLbaMode48:
    position |= (size_t)regs->lba1 << 8;
    position |= (size_t)regs->lba2 << 16;
    position |= (size_t)regs->lba3 << 24;
    position |= (size_t)regs->lba4 << 32;
    position |= (size_t)regs->lba5 << 40;
    if (regs->count0 == 0 && regs->count1 == 0) {
      count = 0x10000;
    } else {
      count = (regs->count1 << 8) | regs->count0;
    }
    break;
  default:
    MV_PANIC("unsupported lba mode %x", io->lba_mode);
    break;
  }
  io->lba_position = position;
  io->lba_count = count;
}

void Harddisk::WriteLba() {
  auto regs = port_->registers();
  auto io = port_->io();
  size_t position = io->lba_position;
  size_t count = io->lba_count;

  switch (io->lba_mode)
  {
  case kIdeLbaMode28:
    regs->device = (regs->device & 0xF0) | ((position >> 24) & 0xF);
    regs->lba2 = (position >> 16) & 0xFF;
    regs->lba1 = (position >> 8) & 0xFF;
    regs->lba0 = (position >> 0) & 0xFF;
    regs->count0 = count == 0x100 ? 0: count & 0xFF;
    break;
  case kIdeLbaMode48:
    regs->lba5 = (position >> 40) & 0xFF;
    regs->lba4 = (position >> 32) & 0xFF;
    regs->lba3 = (position >> 24) & 0xFF;
    regs->lba2 = (position >> 16) & 0xFF;
    regs->lba1 = (position >> 8) & 0xFF;
    regs->lba0 = (position >> 0) & 0xFF;
    if (count == 0x10000) {
      regs->count0 = regs->count1 = 0;
    } else {
      regs->count0 = count & 0xFF;
      regs->count1 = (count >> 8) & 0xFF;
    }
    break;
  default:
    MV_PANIC("unsupported lba mode %x", io->lba_mode);
    break;
  }
}

void Harddisk::StartCommand() {
  auto regs = port_->registers();
  auto io = port_->io();
  MV_LOG("HD start command=0x%x", regs->command);

  regs->status = ATA_SR_DRDY;
  switch (regs->command)
  {
  case 0x6: // DATA SET MANAGEMENT
    if (regs->feature0 == 1) { // TRIM
      io->lba_mode = kIdeLbaMode28;
      io->dma_status = 1;
      ReadLba();
      MV_LOG("trim at sector 0x%x count %d", io->lba_position, io->lba_count);
      EndCommand();
    } else {
      AbortCommand();
      MV_PANIC("unknown dsm command=0x%x", regs->feature0);
    }
    break;
  case ATA_CMD_IDENTIFY_PACKET_DEVICE:
    AbortCommand();
    break;
  case ATA_CMD_READ_SECTORS:
    io->lba_mode = kIdeLbaMode28;
    io->dma_status = 0;
    ReadLba();
    MV_LOG("read %d sectors at 0x%lx", io->lba_count, io->lba_position);
    MV_ASSERT(io->lba_count);
    Ata_ReadSectors(1);
    break;
  case ATA_CMD_READ_DMA:
    io->lba_mode = kIdeLbaMode28;
    io->dma_status = 1;
    ReadLba();
    MV_LOG("read %d sectors at 0x%lx", io->lba_count, io->lba_position);
    MV_ASSERT(io->lba_count);
    Ata_ReadSectors(io->lba_count);
    break;
  case ATA_CMD_WRITE_SECTORS:
    io->lba_mode = kIdeLbaMode28;
    ReadLba();
    MV_LOG("write %d sectors at 0x%lx", io->lba_count, io->lba_position);
    MV_ASSERT(io->lba_count);
    io->nbytes = 1 * gemometry_.sector_size;
    StartTransfer(kIdeTransferToDevice);
    break;
  case 0xB0: // SMART
    AbortCommand();
    break;
  case 0xCA: // ATA_CMD_WRITE_DMA
    io->lba_mode = kIdeLbaMode28;
    io->dma_status = 1;
    ReadLba();
    MV_LOG("write %d sectors at 0x%lx", io->lba_count, io->lba_position);
    MV_ASSERT(io->lba_count);
    io->nbytes = io->lba_count * gemometry_.sector_size;
    StartTransfer(kIdeTransferToDevice);
    break;
  case 0xE7:  // FLUSH_CACHE
  case 0xEA:  // FLUSH_CACHE_EXT
    EndCommand();
    break;
  case 0xF5:  // SECURITY_FREEZE_LOCK
    AbortCommand();
    break;
  default:
    /* Common commands */
    IdeStorageDevice::StartCommand();
    break;
  }
}

void Harddisk::EndTransfer(IdeTransferType type) {
  IdeStorageDevice::EndTransfer(type);

  auto regs = port_->registers();
  auto io = port_->io();

  if (type == kIdeTransferToDevice) {
      switch (regs->command)
      {
      case ATA_CMD_WRITE_SECTORS:
        Ata_WriteSectors(1);
        break;
      case ATA_CMD_WRITE_DMA:
        Ata_WriteSectors(io->lba_count);
        break;
      default:
        MV_PANIC("not impl end transfer of cmd %x", regs->command);
        break;
      }
  } else if (type == kIdeTransferToHost) {
    switch (regs->command)
    {
    case ATA_CMD_READ_SECTORS:
      if (io->lba_count == 0) {
        EndCommand();
      } else {
        Ata_ReadSectors(1);
      }
      break;
    default:
      EndCommand();
      break;
    }
  }
}

void Harddisk::Ata_WriteSectors(int chunk_count) {
  auto io = port_->io();

  uint64_t write_pos = io->lba_position;
  int write_count = io->lba_count;
  if (write_count > chunk_count)
    write_count = chunk_count;

  image_->Write(io->buffer, write_pos, write_count);

  // Move forward sector offset and count
  io->lba_count -= write_count;
  io->lba_position += write_count;
  if (io->lba_count == 0) {
    EndCommand();
  } else {
    io->nbytes = chunk_count * gemometry_.sector_size;
    StartTransfer(kIdeTransferToDevice);
  }
}


void Harddisk::Ata_ReadSectors(int chunk_count) {
  auto io = port_->io();

  uint64_t read_pos = io->lba_position;
  int read_count = io->lba_count;
  if (read_count > chunk_count)
    read_count = chunk_count;

  image_->Read(io->buffer, read_pos, read_count);
  io->nbytes = read_count * gemometry_.sector_size;
  StartTransfer(kIdeTransferToHost);

  // Move forward sector offset and count
  io->lba_count -= read_count;
  io->lba_position += read_count;
  WriteLba();
}

void Harddisk::Ata_IdentifyDevice() {
  auto io = port_->io();
  uint16_t* p = (uint16_t*)io->buffer;
  io->nbytes = 512;
  bzero(p, io->nbytes);

  p[0] = 0x0040;
  p[1] = gemometry_.cylinders_per_heads;
  p[3] = gemometry_.heads;
  p[4] = 512 * gemometry_.sectors_per_cylinder;
  p[5] = 512;
  p[6] = gemometry_.sectors_per_cylinder;
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
  p[54] = gemometry_.cylinders_per_heads;
  p[55] = gemometry_.heads;
  p[56] = gemometry_.sectors_per_cylinder;
  
  uint oldsize = gemometry_.cylinders_per_heads * gemometry_.heads * gemometry_.sectors_per_cylinder;
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
  if (true) {
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
  p[60] = gemometry_.total_sectors;
  p[61] = gemometry_.total_sectors >> 16;
  p[100] = gemometry_.total_sectors;
  p[101] = gemometry_.total_sectors >> 16;
  p[102] = gemometry_.total_sectors >> 32;
  p[103] = gemometry_.total_sectors >> 48;

  StartTransfer(kIdeTransferToHost);
}

DECLARE_DEVICE(Harddisk);
