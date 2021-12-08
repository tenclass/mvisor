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
#include "ahci_port.h"
#include "ata_interval.h"
#include "disk_image.h"

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

/* same constants as bochs */
#define ASC_NO_SEEK_COMPLETE                 0x02
#define ASC_ILLEGAL_OPCODE                   0x20
#define ASC_LOGICAL_BLOCK_OOR                0x21
#define ASC_INV_FIELD_IN_CMD_PACKET          0x24
#define ASC_MEDIUM_MAY_HAVE_CHANGED          0x28
#define ASC_INCOMPATIBLE_FORMAT              0x30
#define ASC_MEDIUM_NOT_PRESENT               0x3a
#define ASC_SAVING_PARAMETERS_NOT_SUPPORTED  0x39
#define ASC_DATA_PHASE_ERROR                 0x4b
#define ASC_MEDIA_REMOVAL_PREVENTED          0x53

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

int cdrom_read_toc_raw(int nb_sectors, uint8_t *buf, int msf, int session_num);
int cdrom_read_toc(int nb_sectors, uint8_t *buf, int msf, int start_track);

Cdrom::Cdrom()
{
  type_ = kIdeStorageTypeCdrom;

  drive_info_.world_wide_name = rand();
  sprintf(drive_info_.serial, "TC%05ld", drive_info_.world_wide_name);
  sprintf(drive_info_.model, "NUWA DVD-ROM");
  sprintf(drive_info_.version, "1.0");

  /* ATA command handlers */
  ata_handlers_[0xA0] = [=] () { // ATA_CMD_PACKET
    ParseCommandPacket();
  };
  
  ata_handlers_[0xA1] = [=] () { // ATA_CMD_IDENTIFY_PACKET_DEVICE
    Atapi_IdentifyData();
  };


  /* ATAPI command handlers */
  atapi_handlers_[0x00] = [=] () { // test unit ready
  };
  
  atapi_handlers_[0x03] = [=] () { // request sense
    Atapi_RequestSense();
  };
  
  atapi_handlers_[0x12] = [=] () { // inquiry
    Atapi_Inquiry();
  };
  
  atapi_handlers_[0x1B] = [=] () { // start stop unit such as load or eject the media
  };
  
  atapi_handlers_[0x1E] = [=] () { // prevent allow media removal
  };
  
  atapi_handlers_[0x25] = [=] () { // get media capacity
    io_.nbytes = 8;
    *(uint32_t*)&io_.buffer[0] = htobe32(total_tracks_ - 1);
    *(uint32_t*)&io_.buffer[4] = htobe32(track_size_);
  };
  
  atapi_handlers_[0x28] = [=] () { // read 10
    CBD_RW_DATA10* p = (CBD_RW_DATA10*)io_.atapi_command;
    io_.lba_block = be32toh(p->lba);
    io_.lba_count = be16toh(p->count);
    Atapi_ReadSectors();
  };
  
  atapi_handlers_[0x2B] = [=] () { // seek
  };
  
  atapi_handlers_[0x35] = [=] () { // flush cache
  };
  
  atapi_handlers_[0x42] = [=] () { // read subchannel
    uint8_t size = io_.atapi_command[8] < 8 ? io_.atapi_command[8] : 8;
    bzero(io_.buffer, size);
    io_.nbytes = size;
  };
  
  atapi_handlers_[0x43] = [=] () { // read table of content
    Atapi_TableOfContent();
  };
  
  atapi_handlers_[0x46] =          // get configuration
  atapi_handlers_[0x4A] = [=] () { // get event status notification
    SetError(ILLEGAL_REQUEST, ASC_INV_FIELD_IN_CMD_PACKET);
  };

  atapi_handlers_[0x51] = [=] () { // read disc information
    SetError(ILLEGAL_REQUEST, ASC_INV_FIELD_IN_CMD_PACKET);
  };
  
  atapi_handlers_[0x5A] = [=] () { // mode sense 10
    Atapi_ModeSense();
  };
}

void Cdrom::Connect() {
  IdeStorageDevice::Connect();

  if (image_) {
    ImageInformation info = image_->information();
    image_block_size_ = info.block_size;
    total_tracks_ = info.total_blocks * info.block_size / track_size_;
  }
}

void Cdrom::SetError(int sense_key, int asc) {
  regs_.error = sense_key << 4;
  regs_.status = ATA_SR_DRDY | ATA_SR_ERR;
  regs_.count0 |= (regs_.count0 & ~7);
  sense_key_ = sense_key;
  asc_ = asc;
}


void Cdrom::ParseCommandPacket() {
  uint8_t command = io_.atapi_command[0];
  if (image_ == nullptr && command == 0x28) {
    SetError(NOT_READY, ASC_MEDIUM_NOT_PRESENT);
    return;
  }

  auto handler = atapi_handlers_[command];
  if (handler) {
    handler();
    regs_.count0 |= ATA_CB_SC_P_IO | ATA_CB_SC_P_CD;
  } else {
    DumpHex(io_.atapi_command, sizeof(io_.atapi_command));
    MV_PANIC("unhandled ATAPI command=0x%x", command);
  }
}

void Cdrom::Atapi_ReadSectors() {
  size_t vec_index = 0;
  size_t position = io_.lba_block * track_size_;
  size_t remain_bytes = io_.lba_count * track_size_;
  while (remain_bytes > 0 && vec_index < io_.vector.size()) {
    auto iov = io_.vector[vec_index];
  
    auto length = remain_bytes < iov.iov_len ? remain_bytes : iov.iov_len;
    
    image_->Read(iov.iov_base, position, length);
    position += length;
    remain_bytes -= length;
    io_.nbytes += length;
    ++vec_index;
  }
}

void Cdrom::Atapi_RequestSense() {
  uint8_t* buf = io_.buffer;
  int max_len = io_.atapi_command[4];

  io_.nbytes = max_len > 18 ? 18 : max_len;
  bzero(buf, 18);
  buf[0] = 0x70 | (1 << 7);
  buf[2] = sense_key_;
  buf[7] = 10;
  buf[12] = asc_;

  if (sense_key_ == UNIT_ATTENTION) {
    sense_key_ = NO_SENSE;
  }
}

void Cdrom::Atapi_ModeSense() {
  uint8_t buf[100] = { 0 };

  switch (io_.atapi_command[2])
  {
  case 0x2A: // Capabilities
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
  
    io_.nbytes = io_.buffer_size < 28 ? io_.buffer_size : 28;
    memcpy(io_.buffer, buf, io_.nbytes);
    break;
  default:
    MV_PANIC("not implemented mode sense command=0x%x", io_.atapi_command[2]);
    break;
  }
}

void Cdrom::Atapi_TableOfContent() {
  uint8_t* buf = io_.buffer;

  int max_size = be16toh(*(uint16_t*)&io_.atapi_command[7]);
  int format = io_.atapi_command[9] >> 6;
  int msf = (io_.atapi_command[1] >> 1) & 1;
  int start_track = io_.atapi_command[6];

  MV_ASSERT(max_size <= io_.buffer_size);
  bzero(buf, max_size);
  switch (format)
  {
  case 0: // TOC Data format
    io_.nbytes = cdrom_read_toc(total_tracks_, buf, msf, start_track);
    if (io_.nbytes < 0) {
      SetError(ILLEGAL_REQUEST, ASC_INV_FIELD_IN_CMD_PACKET);
      return;
    }
    break;
  case 1: // Multi-session
    *(uint16_t*)&buf[0] = htobe16(0x000A);
    buf[2] = 1;
    buf[3] = 1;
    io_.nbytes = 12;
    break;
  case 2: // Raw TOC Data
    io_.nbytes = cdrom_read_toc_raw(total_tracks_, buf, msf, start_track);
    if (io_.nbytes < 0) {
      SetError(ILLEGAL_REQUEST, ASC_INV_FIELD_IN_CMD_PACKET);
      return;
    }
    break;
  default:
    MV_PANIC("invalid TOC command %x", format);
    break;
  }
}

void Cdrom::Atapi_Inquiry() {
  uint8_t* buf = io_.buffer;

  uint8_t size = io_.atapi_command[4];

  buf[0] = 0x05; /* CD-ROM */
  buf[1] = 0x80; /* removable */
  buf[2] = 0x00; /* ISO */
  buf[3] = 0x21; /* ATAPI-2 (XXX: put ATAPI-4 ?) */
  buf[4] = 36 - 5;   /* size */
  buf[5] = 0;    /* reserved */
  buf[6] = 0;    /* reserved */
  buf[7] = 0;    /* reserved */
  padstr8(buf + 8, 8, "TENCLASS");
  padstr8(buf + 16, 16, drive_info_.model);
  padstr8(buf + 32, 4, drive_info_.version);
  io_.nbytes = size > 36 ? 36 : size;
}

void Cdrom::Atapi_IdentifyData() {
  uint16_t p[256] = { 0 };

  /* Removable CDROM, 50us response, 12 byte packets */
  p[0] = (2 << 14) | (5 << 8) | (1 << 7) | (2 << 5) | (0 << 0);

  padstr((char *)(p + 10), drive_info_.serial, 20); /* serial number */
  p[20] = 3; /* buffer type */
  p[21] = 512; /* cache size in sectors */
  p[22] = 4; /* ecc bytes */
  padstr((char *)(p + 23), drive_info_.version, 8); /* firmware version */
  padstr((char *)(p + 27), drive_info_.model, 40); /* model */
  p[48] = 1; /* dword I/O (XXX: should not be set on CDROM) */

  p[49] = 1 << 9 | 1 << 8; /* LBA supported (DMA supported) */
  p[53] = 7; /* words 64-70, 54-58, 88 valid */
  p[62] = 7; /* 0 or 7: single word dma0-2 supported */
  p[63] = 7; /* 0 or 7: mdma0-2 supported */

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

  p[88] = 0x3f | (1 << 13); /* udma5 set and supported */

  if (drive_info_.world_wide_name) {
      /* LE 16-bit words 111-108 contain 64-bit World Wide Name */
      p[108] = drive_info_.world_wide_name >> 48;
      p[109] = drive_info_.world_wide_name >> 32;
      p[110] = drive_info_.world_wide_name >> 16;
      p[111] = drive_info_.world_wide_name;
  }

  io_.nbytes = io_.buffer_size < 512 ? io_.buffer_size : 512;
  memcpy(io_.buffer, p, io_.nbytes);
}

DECLARE_DEVICE(Cdrom);
