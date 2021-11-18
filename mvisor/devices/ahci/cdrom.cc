#include <cstdint>
#include <cstdlib>

static void lba_to_msf(uint8_t *buf, int lba)
{
    lba += 150;
    buf[0] = (lba / 75) / 60;
    buf[1] = (lba / 75) % 60;
    buf[2] = lba % 75;
}

/* same toc as bochs. Return -1 if error or the toc length */
/* XXX: check this */
int cdrom_read_toc(int nb_sectors, uint8_t *buf, int msf, int start_track)
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
int cdrom_read_toc_raw(int nb_sectors, uint8_t *buf, int msf, int session_num)
{
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
