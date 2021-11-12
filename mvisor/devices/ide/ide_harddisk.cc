#include "devices/ide/ide_storage.h"
#include <cstring>
#include "logger.h"
#include "images/image.h"
#include "devices/ide/ide_port.h"
#include "devices/ide/ata_interval.h"

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

IdeHarddiskStorageDevice::IdeHarddiskStorageDevice(DiskImage* image)
  : IdeStorageDevice(image) {
  type_ = kIdeStorageTypeHarddisk;
  name_ = "ide-harddisk";

  drive_info_.world_wide_name = rand();
  sprintf(drive_info_.serial, "XX%05ld", drive_info_.world_wide_name);
  sprintf(drive_info_.model, "XX HARDDISK");
  sprintf(drive_info_.version, "1.0");

  InitializeGemometry();
  multiple_sectors_ = 16;
}

void IdeHarddiskStorageDevice::InitializeGemometry() {
  gemometry_.sector_size = image_->sector_size();
  gemometry_.total_sectors = image_->sectors();
  gemometry_.heads = 16;
  gemometry_.sectors_per_cylinder = 63;
  gemometry_.cylinders_per_heads = gemometry_.total_sectors / (gemometry_.sectors_per_cylinder * gemometry_.heads);
}

void IdeHarddiskStorageDevice::StartCommand() {
  auto regs = port_->registers();
  auto io = port_->io();
  MV_LOG("HD start command=0x%x", regs->command);
  regs->status = ATA_SR_DRDY;
  switch (regs->command)
  {
  case ATA_CMD_IDENTIFY_PACKET_DEVICE:
    AbortCommand();
    break;
  case ATA_CMD_READ_SECTORS:
    Ata_ReadSectors();
    break;
  case ATA_CMD_WRITE_SECTORS:
    io->nbytes = (regs->sectors0 || 256) * gemometry_.sector_size;
    StartTransfer(kIdeTransferToDevice);
    break;
  default:
    /* Common commands */
    IdeStorageDevice::StartCommand();
    break;
  }
}

void IdeHarddiskStorageDevice::EndTransfer(IdeTransferType type) {
  IdeStorageDevice::EndTransfer(type);

  auto regs = port_->registers();

  if (type == kIdeTransferToDevice) {
      switch (regs->command)
      {
      case ATA_CMD_WRITE_SECTORS:
        Ata_WriteSectors();
        break;
      default:
        MV_PANIC("not impl paramters cmd %x", regs->command);
        break;
      }
  } else {
    /* Device to Host, such as Read() */
    EndCommand();
  }
}

void IdeHarddiskStorageDevice::Ata_WriteSectors() {
  auto regs = port_->registers();
  auto io = port_->io();

  int count = regs->sectors0 || 256;
  uint64_t sector = regs->lba0;
  // It's not recommended not to use LBA
  MV_ASSERT(regs->use_lba);

  sector |= (regs->lba1 << 8) | (regs->lba2 << 16) | ((regs->devsel & 0xF) << 24);
  image_->Write(io->buffer, sector, count);
  EndCommand();
}


void IdeHarddiskStorageDevice::Ata_ReadSectors() {
  auto regs = port_->registers();
  auto io = port_->io();

  int count = regs->sectors0 || 256;
  uint64_t sector = regs->lba0;
  // It's not recommended not to use LBA
  MV_ASSERT(regs->use_lba);

  sector |= (regs->lba1 << 8) | (regs->lba2 << 16) | ((regs->devsel & 0xF) << 24);
  image_->Read(io->buffer, sector, count);
  io->nbytes = count * gemometry_.sector_size;
  StartTransfer(kIdeTransferToHost);
}

void IdeHarddiskStorageDevice::Ata_IdentifyDevice() {
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
