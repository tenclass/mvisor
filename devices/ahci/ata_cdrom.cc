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

#include "ata_cdrom.h"

#include <cstring>
#include <algorithm>

#include "disk_image.h"
#include "logger.h"
#include "ahci_port.h"
#include "ata_cdrom.pb.h"
#include "ata_internal.h"

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

/* Some generally useful CD-ROM information */
#define CD_MINS                       80 /* max. minutes per CD */
#define CD_SECS                       60 /* seconds per minute */
#define CD_FRAMES                     75 /* frames per second */
#define CD_FRAMESIZE                2048 /* bytes per frame, "cooked" mode */
#define CD_MAX_BYTES       (CD_MINS * CD_SECS * CD_FRAMES * CD_FRAMESIZE)


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


static void lba_to_msf(uint8_t *buf, int lba)
{
    lba += 150;
    buf[0] = (lba / 75) / 60;
    buf[1] = (lba / 75) % 60;
    buf[2] = lba % 75;
}

/* same toc as bochs. Return -1 if error or the toc length */
/* XXX: check this */
static int cdrom_read_toc(int nb_sectors, uint8_t *buf, int msf, int start_track)
{
  uint8_t *q;
  int len;

  if (start_track > 1 && start_track != 0xaa)
    return -1;
  q = buf + 2;
  *q++ = 1; /* first session */
  *q++ = 1; /* last session */
  if (start_track <= 1) {
    *q++ = 0; /* reserved */
    *q++ = 0x14; /* ADR, control */
    *q++ = 1;    /* track number */
    *q++ = 0; /* reserved */
    if (msf) {
      *q++ = 0; /* reserved */
      lba_to_msf(q, 0);
      q += 3;
    } else {
      /* sector 0 */
      *(uint32_t*)q = htobe32(0);
      q += 4;
    }
  }
  /* lead out track */
  *q++ = 0; /* reserved */
  *q++ = 0x16; /* ADR, control */
  *q++ = 0xaa; /* track number */
  *q++ = 0; /* reserved */
  if (msf) {
    *q++ = 0; /* reserved */
    lba_to_msf(q, nb_sectors);
    q += 3;
  } else {
    *(uint32_t*)q = htobe32(nb_sectors);
    q += 4;
  }
  len = q - buf;
  *(uint16_t*)buf = htobe16(len - 2);
  return len;
}

/* mostly same info as PearPc */
static int cdrom_read_toc_raw(int nb_sectors, uint8_t *buf, int msf, int session_num)
{
  MV_UNUSED(session_num);
  uint8_t *q;
  int len;

  q = buf + 2;
  *q++ = 1; /* first session */
  *q++ = 1; /* last session */

  *q++ = 1; /* session number */
  *q++ = 0x14; /* data track */
  *q++ = 0; /* track number */
  *q++ = 0xa0; /* lead-in */
  *q++ = 0; /* min */
  *q++ = 0; /* sec */
  *q++ = 0; /* frame */
  *q++ = 0;
  *q++ = 1; /* first track */
  *q++ = 0x00; /* disk type */
  *q++ = 0x00;

  *q++ = 1; /* session number */
  *q++ = 0x14; /* data track */
  *q++ = 0; /* track number */
  *q++ = 0xa1;
  *q++ = 0; /* min */
  *q++ = 0; /* sec */
  *q++ = 0; /* frame */
  *q++ = 0;
  *q++ = 1; /* last track */
  *q++ = 0x00;
  *q++ = 0x00;

  *q++ = 1; /* session number */
  *q++ = 0x14; /* data track */
  *q++ = 0; /* track number */
  *q++ = 0xa2; /* lead-out */
  *q++ = 0; /* min */
  *q++ = 0; /* sec */
  *q++ = 0; /* frame */
  if (msf) {
    *q++ = 0; /* reserved */
    lba_to_msf(q, nb_sectors);
    q += 3;
  } else {
    *(uint32_t*)q = htobe32(nb_sectors);
    q += 4;
  }

  *q++ = 1; /* session number */
  *q++ = 0x14; /* ADR, control */
  *q++ = 0;    /* track number */
  *q++ = 1;    /* point */
  *q++ = 0; /* min */
  *q++ = 0; /* sec */
  *q++ = 0; /* frame */
  if (msf) {
    *q++ = 0;
    lba_to_msf(q, 0);
    q += 3;
  } else {
    *q++ = 0;
    *q++ = 0;
    *q++ = 0;
    *q++ = 0;
  }

  len = q - buf;
  *(uint16_t*)buf = htobe16(len - 2);
  return len;
}


AtaCdrom::AtaCdrom()
{
  type_ = kAtaStorageTypeCdrom;

  drive_info_.world_wide_name = 0;
  sprintf(drive_info_.serial, "TC%05ld", drive_info_.world_wide_name);
  sprintf(drive_info_.model, "DVD-ROM");
  sprintf(drive_info_.version, "1.1");

  /* ATA command handlers */
  ata_handlers_[0xA0] = [=] () { // ATA_CMD_PACKET
    if (io_.atapi_set) {
      ParseCommandPacket();
    } else {
      io_.transfer_bytes = 12;
      task_file_->count0 = ATA_CB_SC_P_CD;
      StartTransfer(kTransferAtapiCommand, [this]() {
        memcpy(io_.atapi_command, io_.buffer, 12);
        task_file_->count0 = 0;
        ParseCommandPacket();
      });
    }
  };
  
  ata_handlers_[0xA1] = [=] () { // ATA_CMD_IDENTIFY_PACKET_DEVICE
    Atapi_IdentifyData();
  };
  
  ata_handlers_[0xEC] = [=] () { // ATA_CMD_IDENTIFY_DEVICE
    SetSignature(task_file_);
    AbortCommand();
    task_file_->status |= ATA_CB_STAT_BSY;
  };


  /* ATAPI command handlers */
  atapi_handlers_[0x00] = [=] () { // test unit ready
    if (image_ == nullptr) {
      SetError(NOT_READY, ASC_MEDIUM_NOT_PRESENT);
    } else {
      task_file_->status |= ATA_CB_STAT_SKC;
    }
  };
  
  atapi_handlers_[0x03] = [=] () { // request sense
    if (task_file_->feature0 & 1) {
      WaitForDma([this]() { Atapi_RequestSense(); });
    } else {
      Atapi_RequestSense();
    }
  };
  
  atapi_handlers_[0x12] = [=] () { // inquiry
    if (task_file_->feature0 & 1) {
      WaitForDma([this]() { Atapi_Inquiry(); });
    } else {
      Atapi_Inquiry();
    }
  };
  
  atapi_handlers_[0x1B] = [=] () { // start stop unit such as load or eject the media
    task_file_->status |= ATA_CB_STAT_SKC;
  };
  
  atapi_handlers_[0x1E] = [=] () { // prevent allow media removal
    task_file_->status |= ATA_CB_STAT_SKC;
  };
  
  atapi_handlers_[0x25] = [=] () { // get media capacity
    if (image_ == nullptr) {
      SetError(NOT_READY, ASC_MEDIUM_NOT_PRESENT);
      return;
    }

    if (task_file_->feature0 & 1) {
      WaitForDma([this]() { Atapi_GetMediaCapacity(); });
    } else {
      Atapi_GetMediaCapacity();
    }
  };
  
  atapi_handlers_[0x28] = [=] () { // read 10
    if (image_ == nullptr) {
      SetError(NOT_READY, ASC_MEDIUM_NOT_PRESENT);
      return;
    }
    CBD_RW_DATA10* p = (CBD_RW_DATA10*)io_.atapi_command;
    io_.lba_block = be32toh(p->lba);
    io_.lba_count = be16toh(p->count);
    if (debug_) {
      MV_LOG("ATAPI read block=0x%lx count=0x%lx", io_.lba_block, io_.lba_count);
    }
    if (io_.lba_block + io_.lba_count > geometry_.total_sectors) {
      SetError(ILLEGAL_REQUEST, ASC_LOGICAL_BLOCK_OOR);
      return;
    }
  
    if (task_file_->feature0 & 1) {
      WaitForDma([this]() { Atapi_ReadSectors(io_.lba_count); });
    } else {
      Atapi_ReadSectors(io_.lba_count);
    }
  };
  
  atapi_handlers_[0x2B] = [=] () { // seek
    Atapi_Seek();
  };
  
  atapi_handlers_[0x35] = [=] () { // flush cache
  };
  
  atapi_handlers_[0x42] = [=] () { // read subchannel
    if (task_file_->feature0 & 1) {
      WaitForDma([this]() { Atapi_ReadSubchannel(); });
    } else {
      Atapi_ReadSubchannel();
    }
  };
  
  atapi_handlers_[0x43] = [=] () { // read table of content
    if (image_ == nullptr) {
      SetError(NOT_READY, ASC_MEDIUM_NOT_PRESENT);
      return;
    }

    if (task_file_->feature0 & 1) {
      WaitForDma([this]() { Atapi_ReadTableOfContent(); });
    } else {
      Atapi_ReadTableOfContent();
    }
  };
  
  atapi_handlers_[0x46] = [=] () { // get configuration
    /* only feature 0 is supported */
    if (io_.atapi_command[2] != 0 || io_.atapi_command[3] != 0) {
      SetError(ILLEGAL_REQUEST, ASC_INV_FIELD_IN_CMD_PACKET);
      return;
    }

    if (task_file_->feature0 & 1) {
      WaitForDma([this]() { Atapi_GetConfiguration(); });
    } else {
      Atapi_GetConfiguration();
    }
  };
  atapi_handlers_[0x4A] = [=] () { // get event status notification
    if (!(io_.atapi_command[1] & 1)) {
      /* Only polling is supported, asynchronous mode is not. */
      SetError(ILLEGAL_REQUEST, ASC_INV_FIELD_IN_CMD_PACKET);
      return;
    }

    if (task_file_->feature0 & 1) {
      WaitForDma([this]() { Atapi_GetEventStatusNotification(); });
    } else {
      Atapi_GetEventStatusNotification();
    }
  };

  atapi_handlers_[0x51] = [=] () { // read disc information
    if (task_file_->feature0 & 1) {
      WaitForDma([this]() { Atapi_ReadDiscInformation(); });
    } else {
      Atapi_ReadDiscInformation();
    }
  };
  
  atapi_handlers_[0x5A] = [=] () { // mode sense 10
    if (io_.atapi_command[2] != 0x2A) {
      /* Only MODE_PAGE_CAPABILITIES is supported */
      if (debug_) {
        MV_WARN("unhandled mode sense %x", io_.atapi_command[2]);
      }
      SetError(ILLEGAL_REQUEST, ASC_INV_FIELD_IN_CMD_PACKET);
      return;
    }

    if (task_file_->feature0 & 1) {
      WaitForDma([this]() { Atapi_ModeSense(); });
    } else {
      Atapi_ModeSense();
    }
  };
}

void AtaCdrom::Connect() {
  AtaStorageDevice::Connect();

  if (image_) {
    ImageInformation info = image_->information();
    image_block_size_ = info.block_size;
    geometry_.sector_size = 2048;
    geometry_.total_sectors = info.total_blocks * info.block_size / geometry_.sector_size;
  }
}

void AtaCdrom::SetError(uint sense_key, uint asc) {
  task_file_->error = sense_key << 4;
  task_file_->status |= ATA_CB_STAT_ERR;
  task_file_->count0 = (task_file_->count0 & ~7) | ATA_CB_SC_P_CD | ATA_CB_SC_P_IO;
  sense_key_ = sense_key;
  asc_ = asc;

  /* FIXME: the device didn't call StartTransfer */
  if (io_async_) {
    io_async_ = false;
    if (debug_) {
      MV_WARN("failed to process async ATAPI command 0x%x", io_.atapi_command[0]);
    }
    EndCommand();
  }
}


void AtaCdrom::ParseCommandPacket() {
  uint8_t command = io_.atapi_command[0];
  task_file_->error = 0;
  if (debug_) {
    MV_HEXDUMP("atapi", io_.atapi_command, sizeof(io_.atapi_command));
  }

  auto handler = atapi_handlers_[command];
  if (!handler) {
    SetError(ILLEGAL_REQUEST, ASC_ILLEGAL_OPCODE);
    if (debug_) {
      MV_HEXDUMP("unhandled ATAPI command", io_.atapi_command, sizeof(io_.atapi_command));
    }
    return;
  }

  if (handler) {
    handler();
  }
}


void AtaCdrom::StartTransfer(TransferType type, VoidCallback end_cb) {
  if (type == kTransferDataToHost) {
    task_file_->count0 |= ATA_CB_SC_P_IO;
  } else {
    task_file_->count0 &= ~ATA_CB_SC_P_IO;
  }
  task_file_->lba1 = io_.transfer_bytes & 0xFF;
  task_file_->lba2 = io_.transfer_bytes >> 8;
  AtaStorageDevice::StartTransfer(type, std::move(end_cb));
}

void AtaCdrom::EndCommand() {
  task_file_->count0 = (task_file_->count0 & ~7) | ATA_CB_SC_P_CD | ATA_CB_SC_P_IO;
  AtaStorageDevice::EndCommand();
}

void AtaCdrom::Atapi_ReadSectors(size_t sectors) {
  if (io_.lba_count == 0) {
    return;
  }

  io_async_ = true;
  size_t position = io_.lba_block * geometry_.sector_size;
  size_t remain_bytes = sectors * geometry_.sector_size;
  size_t vec_index = 0;
  ImageIoRequest request = {
    .type = kImageIoRead,
    .position = position,
    .length = 0
  };

  while (vec_index < io_.vector.size() && remain_bytes > 0) {
    auto& iov = io_.vector[vec_index];

    size_t length = std::min(remain_bytes, iov.iov_len);
    request.vector.emplace_back(iovec {
      .iov_base = iov.iov_base,
      .iov_len = length
    });

    request.length += length;
    remain_bytes -= length;
    vec_index++;
  }

  MV_ASSERT(request.length == geometry_.sector_size * sectors);
  io_.transfer_bytes = request.length;
  io_.lba_count -= sectors;
  io_.lba_block += sectors;

  image_->QueueIoRequest(request, [this, sectors, request](ssize_t ret) {
    if (ret <= 0) {
      MV_ERROR("io error ret=%ld, position=%lu length=%lu", ret, request.position, request.length);
      SetError(ILLEGAL_REQUEST, ASC_LOGICAL_BLOCK_OOR);
    } else {
      StartTransfer(kTransferDataToHost, [this, sectors]() {
        Atapi_ReadSectors(sectors);
      });
    }
  });
}

void AtaCdrom::Atapi_Seek() {
  uint lba = be32toh(*(uint32_t*)&io_.atapi_command[2]);
  if (lba >= geometry_.total_sectors) {
    SetError(ILLEGAL_REQUEST, ASC_LOGICAL_BLOCK_OOR);
  } else {
    task_file_->status |= ATA_CB_STAT_SKC;
  }
}

void AtaCdrom::Atapi_RequestSense() {
  uint8_t buf[18];
  bzero(buf, 18);
  buf[0] = 0x70 | (1 << 7);
  buf[2] = sense_key_;
  buf[7] = 10;
  buf[12] = asc_;

  if (sense_key_ == UNIT_ATTENTION) {
    sense_key_ = NO_SENSE;
  }

  int max_size = io_.atapi_command[4];
  io_.transfer_bytes = std::min(max_size, 18);
  memcpy(io_.buffer, buf, io_.transfer_bytes);
  StartTransfer(kTransferDataToHost, []() {});
}

void AtaCdrom::Atapi_GetMediaCapacity() {
  io_.transfer_bytes = 8;
  *(uint32_t*)&io_.buffer[0] = htobe32(geometry_.total_sectors - 1);
  *(uint32_t*)&io_.buffer[4] = htobe32(geometry_.sector_size);
  StartTransfer(kTransferDataToHost, []() {});
}

void AtaCdrom::Atapi_ModeSense() {
  uint8_t buf[100] = { 0 };
  *(uint16_t*)&buf[0] = htobe16(30 - 2);
  buf[2] = 0x70;

  buf[8] = 0x2A;
  buf[9] = 30 - 10;
  buf[10] = 0x3B; /* read CDR/CDRW/DVDROM/DVDR/DVDRAM */

  buf[12] = 0x70;
  buf[13] = 3 << 5;
  buf[14] = (1 << 0) | (1 << 3) | (1 << 5);
  *(uint16_t*)&buf[16] = htobe16(706);
  *(uint16_t*)&buf[18] = htobe16(2);
  *(uint16_t*)&buf[20] = htobe16(512);
  *(uint16_t*)&buf[22] = htobe16(706);

  io_.transfer_bytes = std::min((int)io_.buffer_size, 30);
  memcpy(io_.buffer, buf, io_.transfer_bytes);
  StartTransfer(kTransferDataToHost, []() {});
}

void AtaCdrom::Atapi_ReadTableOfContent() {
  uint8_t* buf = io_.buffer;

  size_t max_size = be16toh(*(uint16_t*)&io_.atapi_command[7]);
  int format = io_.atapi_command[9] >> 6;
  int msf = (io_.atapi_command[1] >> 1) & 1;
  int start_track = io_.atapi_command[6];

  MV_ASSERT(max_size <= io_.buffer_size);
  bzero(buf, max_size);
  switch (format)
  {
  case 0: // TOC Data format
    io_.transfer_bytes = (size_t)cdrom_read_toc(geometry_.total_sectors, buf, msf, start_track);
    break;
  case 1: // Multi-session
    *(uint16_t*)&buf[0] = htobe16(0x000A);
    buf[2] = 1;
    buf[3] = 1;
    io_.transfer_bytes = 12;
    break;
  case 2: // Raw TOC Data
    io_.transfer_bytes = (size_t)cdrom_read_toc_raw(geometry_.total_sectors, buf, msf, start_track);
    break;
  default:
    MV_PANIC("invalid TOC command %x", format);
    break;
  }

  // do not copy more than max_size
  if (io_.transfer_bytes > max_size)
    io_.transfer_bytes = max_size;
  StartTransfer(kTransferDataToHost, []() {});
}

void AtaCdrom::Atapi_ReadSubchannel() {
  size_t length = std::min((int)io_.atapi_command[8], 8);
  bzero(io_.buffer, length);

  io_.transfer_bytes = length;
  StartTransfer(kTransferDataToHost, []() {});
}

void AtaCdrom::Atapi_ReadDiscInformation() {
  uint8_t type = io_.atapi_command[1] & 0x0F;

  /* Types 1/2 are only defined for Blu-Ray.  */
  if (type != 0) {
    SetError(ILLEGAL_REQUEST, ASC_INV_FIELD_IN_CMD_PACKET);
    return;
  }

  uint16_t max_size = be16toh(*(uint16_t*)&io_.atapi_command[7]);
  uint8_t disk_info[34] = {
    0, 32, 0xE, 1, 1, 1, 1, 0x20, 0x00
  };
  io_.transfer_bytes = std::min((size_t)max_size, sizeof(disk_info));
  memcpy(io_.buffer, disk_info, io_.transfer_bytes);
  StartTransfer(kTransferDataToHost, []() {});
}

void AtaCdrom::Atapi_GetConfiguration() {
  uint8_t buf[20];
  bzero(buf, sizeof(buf));

  bool is_dvd = geometry_.total_sectors * geometry_.sector_size > CD_MAX_BYTES;
  if (is_dvd) {
    *(uint16_t*)&buf[6] = htobe16(0x0010);  /* DVD-ROM */
  } else {
    *(uint16_t*)&buf[6] = htobe16(0x0008);  /* CD-ROM */
  }

  buf[10] = 0x02 | 0x01; /* persistent and current */
  buf[11] = 8; /* length of the profile list */
  
  uint8_t* profile = &buf[12];
  *(uint16_t*)profile = htobe16(0x0010); /* DVD-ROM */
  profile[2] = is_dvd ? 0x01 : 0x00;
  profile[3] = 0;

  profile += 4;
  *(uint16_t*)profile = htobe16(0x0008); /* CD-ROM */
  profile[2] = is_dvd ? 0x00 : 0x01;
  profile[3] = 0;

  *(uint32_t*)&buf[0] = htobe32(20 - 4);
  
  size_t max_size = be16toh(*(uint16_t*)&io_.atapi_command[7]);
  io_.transfer_bytes = std::min((int)max_size, 20);
  memcpy(io_.buffer, buf, io_.transfer_bytes);
  StartTransfer(kTransferDataToHost, []() {});
}

void AtaCdrom::Atapi_GetEventStatusNotification() {
  uint8_t* buf = io_.buffer;

  /* current supported event: media */
  buf[3] = 1 << 4;

  if (io_.atapi_command[4] & (1 << 4)) {
    buf[2] = 1 << 4; // media
    buf[4] = 0; // 0: no change 2: new media 1: eject requested 
    buf[5] = 2; // 1: tray open 2: media present
    buf[6] = 0; // reserved
    buf[7] = 0; // reserved
    buf[0] = 0;
    buf[1] = 4;
    io_.transfer_bytes = 8;
    StartTransfer(kTransferDataToHost, []() {});
  } else {
    buf[0] = 0;
    buf[1] = 0;
    buf[2] = 0x80; // no event
    io_.transfer_bytes = 4;
    StartTransfer(kTransferDataToHost, []() {});
  }
}

void AtaCdrom::Atapi_Inquiry() {
  uint8_t* buf = io_.buffer;
  int max_size = io_.atapi_command[4];

  /* If the EVPD (Enable Vital Product Data) bit is set in byte 1,
    * we are being asked for a specific page of info indicated by byte 2. */
  if (io_.atapi_command[1] & 1) {
    uint8_t page = io_.atapi_command[2];
    int index = 0;

    buf[index++] = 0x05; /* CD-ROM */
    buf[index++] = page; /* page code */
    buf[index++] = 0;    /* reserved */
    buf[index++] = 0;    /* length */

    switch (page)
    {
    case 0x00:
      /* Supported Pages: List of supported VPD responses. */
      buf[index++] = 0x00; /* 0x00: Supported Pages, and: */
      buf[index++] = 0x83; /* 0x83: Device Identification. */
      break;
    case 0x83:
      /* Device Indentification */
      // Entry 1: Serial
      if (index + 24 > max_size) {
        SetError(ILLEGAL_REQUEST, ASC_DATA_PHASE_ERROR);
        return;
      }
      buf[index++] = 0x02; /* Ascii */
      buf[index++] = 0x00; /* Vendor specific */
      buf[index++] = 0x00;
      buf[index++] = 20; /* Remain length */
      padstr8(buf + index, 20, drive_info_.serial);
      index += 20;

      // Entry 2: Drive Model and Serial
      if (index + 72 > max_size) {
        break;
      }
      buf[index++] = 0x02; /* Ascii */
      buf[index++] = 0x01; /* T10 Vendor ID */
      buf[index++] = 0x00;
      buf[index++] = 68; /* Remain length */
      padstr8(buf + index, 8, "ATA"); /* T10 Vendor ID */
      index += 8;
      padstr8(buf + index, 40, drive_info_.model);
      index += 40;
      padstr8(buf + index, 20, drive_info_.serial);
      index += 20;

      // Entry 3: WWN
      if (drive_info_.world_wide_name && (index + 12 <= max_size)) {
        buf[index++] = 0x01; /* Binary */
        buf[index++] = 0x03; /* NAA */
        buf[index++] = 0x00;
        buf[index++] = 8; /* Remain length */
        buf[index++] = drive_info_.world_wide_name >> 40;
        buf[index++] = drive_info_.world_wide_name >> 32;
        buf[index++] = drive_info_.world_wide_name >> 24;
        buf[index++] = drive_info_.world_wide_name >> 16;
        buf[index++] = drive_info_.world_wide_name >> 8;
        buf[index++] = drive_info_.world_wide_name;
      }
      break;
    default:
      SetError(ILLEGAL_REQUEST, ASC_INV_FIELD_IN_CMD_PACKET);
      return;
    }

    buf[3] = index - 4;
    io_.transfer_bytes = std::min(max_size, index);
    StartTransfer(kTransferDataToHost, []() {});
  } else {
    buf[0] = 0x05; /* CD-ROM */
    buf[1] = 0x80; /* removable */
    buf[2] = 0x00; /* ISO */
    buf[3] = 0x21; /* ATAPI-2 (XXX: put ATAPI-4 ?) */
    buf[4] = 36 - 5;   /* length */
    buf[5] = 0;    /* reserved */
    buf[6] = 0;    /* reserved */
    buf[7] = 0;    /* reserved */
    padstr8(buf + 8, 8, "Tenclass");
    padstr8(buf + 16, 16, drive_info_.model);
    padstr8(buf + 32, 4, drive_info_.version);

    io_.transfer_bytes = std::min(max_size, 36);
    StartTransfer(kTransferDataToHost, []() {});
  }
}

void AtaCdrom::Atapi_IdentifyData() {
  uint16_t p[256] = { 0 };
  bool dma_supported = true;

  /* Removable CDROM, 50us response, 12 byte packets */
  p[0] = (2 << 14) | (5 << 8) | (1 << 7) | (2 << 5) | (0 << 0);

  padstr((char *)(p + 10), drive_info_.serial, 20); /* serial number */
  p[20] = 3; /* buffer type */
  p[21] = 512; /* cache size in sectors */
  p[22] = 4; /* ecc bytes */
  padstr((char *)(p + 23), drive_info_.version, 8); /* firmware version */
  padstr((char *)(p + 27), drive_info_.model, 40); /* model */
  p[48] = 1; /* dword I/O (XXX: should not be set on CDROM) */

  if (dma_supported) {
    p[49] = 1 << 9 | 1 << 8; /* LBA supported (DMA supported) */
    p[53] = 7; /* words 64-70, 54-58, 88 valid */
    p[62] = 7; /* 0 or 7: single word dma0-2 supported */
    p[63] = 7; /* 0 or 7: mdma0-2 supported */
  } else {
    p[49] = 1 << 9; /* LBA supported (no DMA) */
    p[53] = 3; /* words 64-70, 54-58 valid */
    p[63] = 0x103; /* ??? */
  }

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
  
  if (dma_supported) {
    p[88] = 0x3f | (1 << 13); /* udma5 set and supported */
  }

  if (drive_info_.world_wide_name) {
      /* LE 16-bit words 111-108 contain 64-bit World Wide Name */
      p[108] = drive_info_.world_wide_name >> 48;
      p[109] = drive_info_.world_wide_name >> 32;
      p[110] = drive_info_.world_wide_name >> 16;
      p[111] = drive_info_.world_wide_name;
  }

  io_.transfer_bytes = std::min((int)io_.buffer_size, 512);
  memcpy(io_.buffer, p, io_.transfer_bytes);
  StartTransfer(kTransferDataToHost, []() {});
}


bool AtaCdrom::SaveState(MigrationWriter* writer) {
  AtaCdromState state;
  state.set_sense_key(sense_key_);
  state.set_asc(asc_);
  writer->WriteProtobuf("CDROM", state);
  return AtaStorageDevice::SaveState(writer);
}

bool AtaCdrom::LoadState(MigrationReader* reader) {
  if (!AtaStorageDevice::LoadState(reader)) {
    return false;
  }

  AtaCdromState state;
  if (!reader->ReadProtobuf("CDROM", state)) {
    return false;
  }
  sense_key_ = state.sense_key();
  asc_ = state.asc();
  return true;
}

DECLARE_DEVICE(AtaCdrom);
