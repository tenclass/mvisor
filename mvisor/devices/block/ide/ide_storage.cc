#include "devices/ide.h"
#include <cstring>
#include "logger.h"
#include "device_manager.h"
#include "devices/ata_interval.h"
#include "images/image.h"

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


IdeStorageDevice::IdeStorageDevice(DiskImage* image) : StorageDevice(image) {
  name_ = "ide-storage";
}

void IdeStorageDevice::ProcessPacket(uint8_t* packet, ssize_t* nbytes) {
  MV_PANIC("not implemented");
}


void IdeCdromStorageDevice::ProcessPacket(uint8_t* buf, ssize_t* nbytes) {
  MV_LOG("packet=0x%x", buf[0]);
  switch (buf[0])
  {
  case GPCMD_TEST_UNIT_READY:
    // Test ready
    *nbytes = 0;
    break;
  case GPCMD_READ_10: {
    CBD_RW_DATA10* p = (CBD_RW_DATA10*)buf;
    uint32_t lba = be32toh(p->lba);
    uint16_t count = be16toh(p->count);
    image_->Read(buf, lba, count);
    *nbytes = image_->sector_size() * count;
    break;
  }
  case GPCMD_INQUIRY: {
    buf[0] = 0x05; /* CD-ROM */
    buf[1] = 0x80; /* removable */
    buf[2] = 0x00; /* ISO */
    buf[3] = 0x21; /* ATAPI-2 (XXX: put ATAPI-4 ?) */
    buf[4] = 36 - 5;   /* size */
    buf[5] = 0;    /* reserved */
    buf[6] = 0;    /* reserved */
    buf[7] = 0;    /* reserved */
    padstr8(buf + 8, 8, "QEMU");
    padstr8(buf + 16, 16, "QEMU DVD-ROM");
    padstr8(buf + 32, 4, version_string_);
    *nbytes = 36;
    break;
  }
  default:
    MV_PANIC("unhandled packet 0x%x", buf[0]);
    break;
  }
}

IdeCdromStorageDevice::IdeCdromStorageDevice(DiskImage* image)
  : IdeStorageDevice(image)
{
  type_ = kIdeStorageTypeCdrom;
  name_ = "ide-cdrom";

  image->set_sector_size(2048);
  drive_serial_ = rand();
  sprintf(drive_serial_string_, "MV%05d", drive_serial_);
  sprintf(drive_model_string_, "DVD-ROM");
  sprintf(version_string_, "1.0");
  world_wide_name_ = rand();
}

void IdeCdromStorageDevice::GetIdentityData(uint8_t* buffer, size_t buffer_size, ssize_t* nbytes) {
  uint16_t* p = (uint16_t*)identify_data_;
  if (p[0])
    goto fill_buffer;

  /* Removable CDROM, 50us response, 12 byte packets */
  p[0] = (2 << 14) | (5 << 8) | (1 << 7) | (2 << 5) | (0 << 0);

  padstr((char *)(p + 10), drive_serial_string_, 20); /* serial number */
  p[20] = 3; /* buffer type */
  p[21] = 512; /* cache size in sectors */
  p[22] = 4; /* ecc bytes */
  padstr((char *)(p + 23), version_string_, 8); /* firmware version */
  padstr((char *)(p + 27), drive_model_string_, 40); /* model */
  p[48] = 1; /* dword I/O (XXX: should not be set on CDROM) */

  p[49] = 1 << 9 | 1 << 8; /* DMA and LBA supported */
  p[53] = 7; /* words 64-70, 54-58, 88 valid */
  p[62] = 7;  /* single word dma0-2 supported */
  p[63] = 7;  /* mdma0-2 supported */

  p[64] = 3; /* 3: pio3-4 supported */
  p[65] = 0xb4; /* minimum DMA multiword tx cycle time */
  p[66] = 0xb4; /* recommended DMA multiword tx cycle time */
  p[67] = 0x12c; /* minimum PIO cycle time without flow control */
  p[68] = 0xb4; /* minimum PIO cycle time with IORDY flow control */

  p[71] = 30; /* in ns */
  p[72] = 30; /* in ns */

  /* AHCI MAX_COMMANDS */
  p[75] = 32 - 1;
  /* NCQ supported */
  p[76] = (1 << 8);

  p[80] = 0x1e; /* support up to ATA/ATAPI-4 */
  if (world_wide_name_) {
      p[84] = (1 << 8); /* supports WWN for words 108-111 */
      p[87] = (1 << 8); /* WWN enabled */
  }

  p[88] = 0x3f | (1 << 13); /* udma5 set and supported */

  if (world_wide_name_) {
      /* LE 16-bit words 111-108 contain 64-bit World Wide Name */
      p[108] = world_wide_name_ >> 48;
      p[109] = world_wide_name_ >> 32;
      p[110] = world_wide_name_ >> 16;
      p[111] = world_wide_name_;
  }

fill_buffer:
  *nbytes = buffer_size < sizeof(identify_data_) ? buffer_size : sizeof(identify_data_);
  memcpy(buffer, p, *nbytes);
}


void IdeHarddiskStorageDevice::GetIdentityData(uint8_t* buffer, size_t buffer_size, ssize_t* nbytes) {
  *nbytes = 0;
}
