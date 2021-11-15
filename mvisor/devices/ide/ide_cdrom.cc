#include "devices/ide/ide_storage.h"
#include <cstring>
#include "logger.h"
#include "devices/ide/ide_port.h"
#include "devices/ide/ata_interval.h"
#include "images/image.h"

/*
 *  SENSE KEYS
 */

#define NO_SENSE            0x00
#define RECOVERED_ERROR     0x01
#define NOT_READY           0x02
#define MEDIUM_ERROR        0x03
#define HARDWARE_ERROR      0x04
#define ILLEGAL_REQUEST     0x05
#define UNIT_ATTENTION      0x06
#define DATA_PROTECT        0x07
#define BLANK_CHECK         0x08
#define COPY_ABORTED        0x0a
#define ABORTED_COMMAND     0x0b
#define VOLUME_OVERFLOW     0x0d
#define MISCOMPARE          0x0e

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

static void padstr8(uint8_t *buf, int buf_size, const char *src)
{
  for(int i = 0; i < buf_size; i++) {
    if (*src)
      buf[i] = *src++;
    else
      buf[i] = ' ';
  }
}

IdeCdromStorageDevice::IdeCdromStorageDevice(DiskImage* image)
  : IdeStorageDevice(image)
{
  type_ = kIdeStorageTypeCdrom;
  name_ = "ide-cdrom";

  image->set_sector_size(2048);

  drive_info_.world_wide_name = rand();
  sprintf(drive_info_.serial, "XX%05ld", drive_info_.world_wide_name);
  sprintf(drive_info_.model, "XX DVD-ROM");
  sprintf(drive_info_.version, "1.0");
}

void IdeCdromStorageDevice::SetError(int sense_key, int asc) {
  auto regs = port_->registers();
  regs->error = sense_key << 4;
  regs->status = ATA_SR_DRDY | ATA_SR_ERR;
  regs->sectors0 = (regs->sectors0 & ~7) | ATA_CB_SC_P_IO | ATA_CB_SC_P_CD;
  sense_key_ = sense_key;
  asc_ = asc;
}


void IdeCdromStorageDevice::StartTransfer(IdeTransferType type) {
  auto regs = port_->registers();
  auto io = port_->io();
  if (type == kIdeTransferToHost) {
    regs->sectors0 |= ATA_CB_SC_P_IO;
  } else {
    regs->sectors0 &= ~ATA_CB_SC_P_IO;
  }
  regs->lba1 = io->nbytes;
  regs->lba2 = io->nbytes >> 8;
  IdeStorageDevice::StartTransfer(type);
}

#include <unistd.h>
void IdeCdromStorageDevice::EndTransfer(IdeTransferType type) {
  IdeStorageDevice::EndTransfer(type);

  auto regs = port_->registers();
  auto io = port_->io();

  if (type == kIdeTransferToDevice) {
    regs->sectors0 &= ~ATA_CB_SC_P_IO;
    if (regs->sectors0 & ATA_CB_SC_P_CD) {
      regs->sectors0 &= ~ATA_CB_SC_P_CD;
      switch (regs->command)
      {
      case ATA_CMD_PACKET:
        ParseCommandPacket();
        break;
      default:
        MV_PANIC("not impl paramters cmd %x", regs->command);
        break;
      }
    } else {
      MV_LOG("write data to dev end, do nothing");
    }
  } else if (type == kIdeTransferToHost) {
    /* Device to Host, such as Read() */
    if (io->lba_count == 0) {
      EndCommand();
    } else {
      Atapi_ReadOneSector();
    }
  }
}

void IdeCdromStorageDevice::EndCommand() {
  auto regs = port_->registers();
  regs->sectors0 |= ATA_CB_SC_P_CD | ATA_CB_SC_P_IO;
  IdeStorageDevice::EndCommand();
}

void IdeCdromStorageDevice::StartCommand() {
  auto regs = port_->registers();
  auto io = port_->io();
  MV_LOG("CDROM start command=0x%x", regs->command);
  regs->status = ATA_SR_DRDY;

  switch (regs->command)
  {
  case ATA_CMD_IDENTIFY_PACKET_DEVICE:
    Atapi_IdentifyData();
    break;
  case ATA_CMD_PACKET:
    /* Start a command transfer from host to device */
    io->nbytes = 12; /* PACKET CMD SIZE */
    regs->status |= ATA_SR_DRQ;
    regs->sectors0 = ATA_CB_SC_P_CD; /* CD is on, command will be handled later */
    StartTransfer(kIdeTransferToDevice);
    break;
  default:
    /* Common commands */
    IdeStorageDevice::StartCommand();
    break;
  }
}

void IdeCdromStorageDevice::ParseCommandPacket() {
  auto io = port_->io();
  uint8_t* buf = io->buffer;
  MV_LOG("#############PACKET command=0x%x", buf[0]);

  switch (buf[0])
  {
  case GPCMD_TEST_UNIT_READY:
    // Test ready
    io->nbytes = 0;
    EndCommand();
    break;
  case GPCMD_READ_10: {
    CBD_RW_DATA10* p = (CBD_RW_DATA10*)buf;
    io->lba_position = be32toh(p->lba);
    io->lba_count = be16toh(p->count);
    MV_LOG("read %x sectors at %x", io->lba_count, io->lba_position);
    if (io->lba_count == 0) {
      EndCommand();
    } else {
      Atapi_ReadOneSector();
    }
    break;
  }
  case GPCMD_INQUIRY: {
    Atapi_Inquiry();
    break;
  case GPCMD_MODE_SENSE_10:
    Atapi_ModeSense();
    break;
  case GPCMD_SEEK:
    EndCommand();
    MV_LOG("ignore seek");
    break;
  }
  case GPCMD_READ_TOC_PMA_ATIP:
    Atapi_TableOfContent();
    break;
  case GPCMD_READ_SUBCHANNEL: {
    uint8_t size = buf[8] < 8 ? buf[8] : 8;
    bzero(buf, size);
    io->nbytes = size;
    StartTransfer(kIdeTransferToHost);
    break;
  }
  default:
    DumpHex(buf, 12);
    MV_PANIC("unhandled packet 0x%x", buf[0]);
    break;
  }
}

void IdeCdromStorageDevice::Atapi_ModeSense() {
  auto io = port_->io();
  uint8_t* buf = io->buffer;
  int max_size = be16toh(*(uint16_t*)&buf[6]);
  MV_LOG("max_size = %d", max_size);
  switch (buf[2])
  {
  case 0x2A: // Capabilities
    io->nbytes = 28;
    bzero(buf, 28);
    *(uint16_t*)&buf[0] = htobe16(34);
    buf[2] = 0x70;
    buf[8] = 0x2A;
    buf[9] = 0x12;
    buf[12] = 0x70;
    buf[13] = 0x60;
    buf[14] = 41;
    *(uint16_t*)&buf[16] = htobe16(706);
    *(uint16_t*)&buf[18] = htobe16(2);
    *(uint16_t*)&buf[20] = htobe16(512);
    *(uint16_t*)&buf[22] = htobe16(706);
    StartTransfer(kIdeTransferToHost);
    break;
  default:
    MV_PANIC("not implemented cmd=0x%x", buf[2]);
    break;
  }
}

void IdeCdromStorageDevice::Atapi_TableOfContent() {
  auto io = port_->io();
  uint8_t* buf = io->buffer;

  int max_size = be16toh(*(uint16_t*)&buf[7]);
  int format = buf[9] >> 6;

  bzero(buf, max_size);
  switch (format)
  {
  case 0: // TOC Data format
    MV_PANIC("invalid TOC command %x", format);
    break;
  case 1: // Multi-session
    io->nbytes = 12;
    *(uint16_t*)&buf[0] = htobe16(0x000A);
    buf[2] = 1;
    buf[3] = 1;
    break;
  case 2: // Raw TOC Data
    MV_PANIC("invalid TOC command %x", format);
    break;
  default:
    MV_PANIC("invalid TOC command %x", format);
    break;
  }
  StartTransfer(kIdeTransferToHost);
}

void IdeCdromStorageDevice::Atapi_ReadOneSector() {
  auto io = port_->io();
  uint8_t* buf = io->buffer;

  size_t read_count = 1;

  image_->Read(buf, io->lba_position, read_count);
  io->nbytes = image_->sector_size() * read_count;
  StartTransfer(kIdeTransferToHost);

  io->lba_position += read_count;
  io->lba_count -= read_count;
}

void IdeCdromStorageDevice::Atapi_Inquiry() {
  auto io = port_->io();
  uint8_t* buf = io->buffer;

  uint8_t size = buf[4];
  MV_ASSERT(size == 36);

  buf[0] = 0x05; /* CD-ROM */
  buf[1] = 0x80; /* removable */
  buf[2] = 0x00; /* ISO */
  buf[3] = 0x21; /* ATAPI-2 (XXX: put ATAPI-4 ?) */
  buf[4] = 36 - 5;   /* size */
  buf[5] = 0;    /* reserved */
  buf[6] = 0;    /* reserved */
  buf[7] = 0;    /* reserved */
  padstr8(buf + 8, 8, "MVISOR");
  padstr8(buf + 16, 16, drive_info_.model);
  padstr8(buf + 32, 4, drive_info_.version);
  io->nbytes = size > 36 ? 36 : size;
  StartTransfer(kIdeTransferToHost);
}

void IdeCdromStorageDevice::Atapi_IdentifyData() {
  auto io = port_->io();
  uint16_t* p = (uint16_t*)io->buffer;
  io->nbytes = 512;
  bzero(p, io->nbytes);

  /* Removable CDROM, 50us response, 12 byte packets */
  p[0] = (2 << 14) | (5 << 8) | (1 << 7) | (2 << 5) | (0 << 0);

  padstr((char *)(p + 10), drive_info_.serial, 20); /* serial number */
  p[20] = 3; /* buffer type */
  p[21] = 512; /* cache size in sectors */
  p[22] = 4; /* ecc bytes */
  padstr((char *)(p + 23), drive_info_.version, 8); /* firmware version */
  padstr((char *)(p + 27), drive_info_.model, 40); /* model */
  p[48] = 1; /* dword I/O (XXX: should not be set on CDROM) */

  p[49] = 1 << 9; /* LBA supported (| 1 << 8 DMA supported) */
  p[53] = 7; /* words 64-70, 54-58, 88 valid */
  p[62] = 0; /* 0 or 7: single word dma0-2 supported */
  p[63] = 0; /* 0 or 7: mdma0-2 supported */

  p[64] = 3; /* pio3-4 supported */
  p[65] = 0xb4; /* minimum DMA multiword tx cycle time */
  p[66] = 0xb4; /* recommended DMA multiword tx cycle time */
  p[67] = 0x12c; /* minimum PIO cycle time without flow control */
  p[68] = 0xb4; /* minimum PIO cycle time with IORDY flow control */

  p[71] = 30; /* in ns */
  p[72] = 30; /* in ns */

  p[75] = 32 - 1; /* AHCI MAX_COMMANDS */
  p[76] = (1 << 8); /* NCQ supported */

  p[80] = 0x1e; /* support up to ATA/ATAPI-4 */
  if (drive_info_.world_wide_name) {
      p[84] = (1 << 8); /* supports WWN for words 108-111 */
      p[87] = (1 << 8); /* WWN enabled */
  }

  /* p[88] = 0x3f | (1 << 13); udma5 set and supported */

  if (drive_info_.world_wide_name) {
      /* LE 16-bit words 111-108 contain 64-bit World Wide Name */
      p[108] = drive_info_.world_wide_name >> 48;
      p[109] = drive_info_.world_wide_name >> 32;
      p[110] = drive_info_.world_wide_name >> 16;
      p[111] = drive_info_.world_wide_name;
  }

  StartTransfer(kIdeTransferToHost);
}
