#ifndef _MVISOR_DEVICES_PCI_VGA_H
#define _MVISOR_DEVICES_PCI_VGA_H

#include "devices/pci_device.h"

/*
 * bochs vesa bios extension interface
 */
#define VBE_DISPI_MAX_XRES              16000
#define VBE_DISPI_MAX_YRES              12000
#define VBE_DISPI_MAX_BPP               32

#define VBE_DISPI_INDEX_ID              0x0
#define VBE_DISPI_INDEX_XRES            0x1
#define VBE_DISPI_INDEX_YRES            0x2
#define VBE_DISPI_INDEX_BPP             0x3
#define VBE_DISPI_INDEX_ENABLE          0x4
#define VBE_DISPI_INDEX_BANK            0x5
#define VBE_DISPI_INDEX_VIRT_WIDTH      0x6
#define VBE_DISPI_INDEX_VIRT_HEIGHT     0x7
#define VBE_DISPI_INDEX_X_OFFSET        0x8
#define VBE_DISPI_INDEX_Y_OFFSET        0x9
#define VBE_DISPI_INDEX_NB              0xa /* size of vbe_regs[] */
#define VBE_DISPI_INDEX_VIDEO_MEMORY_64K 0xa /* read-only, not in vbe_regs */

/* VBE_DISPI_INDEX_ID */
#define VBE_DISPI_ID0                   0xB0C0
#define VBE_DISPI_ID1                   0xB0C1
#define VBE_DISPI_ID2                   0xB0C2
#define VBE_DISPI_ID3                   0xB0C3
#define VBE_DISPI_ID4                   0xB0C4
#define VBE_DISPI_ID5                   0xB0C5

/* VBE_DISPI_INDEX_ENABLE */
#define VBE_DISPI_DISABLED              0x00
#define VBE_DISPI_ENABLED               0x01
#define VBE_DISPI_GETCAPS               0x02
#define VBE_DISPI_8BIT_DAC              0x20
#define VBE_DISPI_LFB_ENABLED           0x40
#define VBE_DISPI_NOCLEARMEM            0x80

class VgaDevice : public PciDevice {
 public:
  VgaDevice();
  ~VgaDevice();

  void Connect();
  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length);
  bool IsTextMode();
  uint64_t GetVRamAddress();
  void GetCursorLocation(uint8_t* x, uint8_t* y, uint8_t* sel_start, uint8_t* sel_end);

  const uint8_t* pallete() const { return pallete_; }
 private:
  void ResetRegisters();
  void VbeIoReadData(uint16_t* data);
  void VbeIoWriteData(uint16_t value);


  uint8_t misc_ouput_reg_ = 0;
  uint8_t sequence_index_ = 0;
  uint8_t sequence_registers_[256] = { 0 };
  uint8_t gfx_index_ = 0;
  uint8_t gfx_registers_[256] = { 0 };
  uint8_t attribute_index_ = 0;
  uint8_t attribute_registers_[0x15] = { 0 };
  uint16_t pallete_read_index_ = 0;
  uint16_t pallete_write_index_ = 0;
  uint8_t pallete_[256 * 3];
  uint8_t crtc_index_ = 0;
  uint8_t crtc_registers_[0x19] = { 0 };

  uint32_t vbe_size_ = 0;
  uint16_t vbe_index_ = 0;
  uint16_t vbe_registers_[16] = { 0 };
};

#endif // _MVISOR_DEVICES_PCI_VGA_H
