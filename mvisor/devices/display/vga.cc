/*
 * Since we want to implement QXL, so don't waste time on VGA
 * terrence.huang@tenclass.com
 */

#include "devices/vga.h"
#include <cstring>
#include "logger.h"
#include "device_manager.h"

#define VGA_ROM_PATH "../assets/vgabios-qxl.bin"
#define VGA_PIO_BASE    0x3C0
#define VGA_PIO_SIZE    0x20
#define VBE_PIO_BASE    0x1CE
#define VBE_PIO_SIZE    2

VgaDevice::VgaDevice() {
  name_ = "vga";
  devfn_ = PCI_MAKE_DEVFN(2, 0);
  
  /* PCI config */
  pci_header_.vendor_id = 0x1b36;
  pci_header_.device_id = 0x0100;
  pci_header_.class_code = 0x030000;
  pci_header_.revision_id = 5;
  pci_header_.header_type = PCI_HEADER_TYPE_NORMAL;
  pci_header_.subsys_vendor_id = 0x1af4;
  pci_header_.subsys_id = 0x1100;
  pci_header_.command = PCI_COMMAND_IO | PCI_COMMAND_MEMORY;
  pci_header_.irq_pin = 1;

  /* Initialize rom data and rom bar size */
  LoadRomFile(VGA_ROM_PATH);

  /* 384MB vgamem */
  bar_size_[0] = 0x18000000;
  /* 64MB vram */
  vbe_size_ = 0x04000000;
  /* 8MB vram */
  bar_size_[1] = 0x00800000;
  bar_size_[2] = 0x00002000;
  /* MMIO */
  bar_size_[3] = 0x20;
  pci_header_.bar[3] = 1;

  AddIoResource(kIoResourceTypePio, VGA_PIO_BASE, VGA_PIO_SIZE, "vga-io");
  AddIoResource(kIoResourceTypePio, VBE_PIO_BASE, VBE_PIO_SIZE, "vbe-io");

  ResetRegisters();
}

VgaDevice::~VgaDevice() {
}

void VgaDevice::Connect() {
  PciDevice::Connect();
}

void VgaDevice::ResetRegisters() {
  bzero(vbe_registers_, sizeof(vbe_registers_));
}

void VgaDevice::VbeIoReadData(uint16_t* data) {
  uint32_t val;
  if (vbe_index_ < VBE_DISPI_INDEX_NB) {
    if (vbe_registers_[VBE_DISPI_INDEX_ENABLE] & VBE_DISPI_GETCAPS) {
      /* VBE initialization will enable and get capabilities and then disable */
      switch(vbe_index_) {
      case VBE_DISPI_INDEX_XRES:
        val = VBE_DISPI_MAX_XRES;
        break;
      case VBE_DISPI_INDEX_YRES:
        val = VBE_DISPI_MAX_YRES;
        break;
      case VBE_DISPI_INDEX_BPP:
        val = VBE_DISPI_MAX_BPP;
        break;
      default:
        val = vbe_registers_[vbe_index_];
        break;
      }
    } else {
      val = vbe_registers_[vbe_index_];
    }
  } else if (vbe_index_ == VBE_DISPI_INDEX_VIDEO_MEMORY_64K) {
    val = vbe_size_ / (64 * 1024);
  } else {
    MV_LOG("read index = %d data=%x", vbe_index_, vbe_registers_[vbe_index_]);
    val = 0;
  }
  *data = val;
}

void VgaDevice::VbeIoWriteData(uint16_t val) {
  if (vbe_index_ <= VBE_DISPI_INDEX_NB) {
    vbe_registers_[vbe_index_] = val;
  }
}

void VgaDevice::Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  uint64_t port = ir.base + offset;
  if (ir.base == VGA_PIO_BASE) {
    switch (port)
    {
    case 0x3C0:
      MV_ASSERT(size == 1);
      *data = attribute_index_;
      break;
    case 0x3C1:
      MV_ASSERT(size == 1);
      *data = attribute_registers_[attribute_index_];
      break;
    case 0x3C5:
      MV_ASSERT(size == 1);
      *data = sequence_registers_[sequence_index_];
      break;
    case 0x3C9:
      for (uint32_t i = 0; i < size; i++) {
        *data++ = pallete_[pallete_read_index_++];
      }
      break;
    case 0x3CC:
      MV_ASSERT(size == 1);
      *data = misc_ouput_reg_;
      break;
    case 0x3CF:
      MV_ASSERT(size == 1);
      *data = gfx_registers_[gfx_index_];
      break;
    case 0x3D5:
      MV_ASSERT(size == 1);
      *data = crtc_registers_[crtc_index_];
      break;
    case 0x3DA:
      MV_ASSERT(size == 1);
      static int b;
      b = !b;
      *data = (b << 3);
      break;
    default:
      MV_PANIC("not implemented %s port=0x%lx size=%d data=0x%lx",
        name_.c_str(), port, size, *(uint64_t*)data);
      break;
    }
  } else if (ir.base == VBE_PIO_BASE) {
    VbeIoReadData((uint16_t*)data);
  }
}

void VgaDevice::Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  uint64_t port = ir.base + offset;
  if (port != 0x3ce && port != 0x3cf && port != 0x3d4 && *data != 0x0e && *data != 0x0f && port != 0x3c8 && port != 0x3c9 && port != 0x1ce) {
    // MV_LOG("port=0x%lx size=%d data=0x%lx", port, size, *(uint64_t*)data);
  }
  if (ir.base == VGA_PIO_BASE) {
    uint8_t value = *data;
    switch (port)
    {
    case 0x3C0:
      MV_ASSERT(size == 1);
      attribute_index_ = value;
      break;
    case 0x3C2:
      MV_ASSERT(size == 1);
      misc_ouput_reg_ = value & ~0x10;
      break;
    case 0x3C4:
      sequence_index_ = value;
      if (size == 2) {
        sequence_registers_[sequence_index_] = data[1];
      }
      break;
    case 0x3C5:
      MV_ASSERT(size == 1);
      sequence_registers_[sequence_index_] = value;
      break;
    case 0x3C6:
      MV_ASSERT(value == 0xFF);
      break;
    case 0x3C7:
      MV_ASSERT(size == 1);
      pallete_read_index_ = value * 3;
      break;
    case 0x3C8:
      MV_ASSERT(size == 1);
      pallete_write_index_ = value * 3;
      break;
    case 0x3C9:
      MV_ASSERT(size == 1);
      pallete_[pallete_write_index_++] = value;
      break;
    case 0x3CE:
      gfx_index_ = value;
      if (size == 2) {
        gfx_registers_[gfx_index_] = data[1];
      }
      break;
    case 0x3CF:
      MV_ASSERT(size == 1);
      gfx_registers_[gfx_index_] = value;
      break;
    case 0x3D4:
      crtc_index_ = value;
      if (size == 2) {
        crtc_registers_[crtc_index_] = data[1];
      }
      break;
    case 0x3D5:
      MV_ASSERT(size == 1);
      crtc_registers_[crtc_index_] = value;
      break;
    default:
      MV_PANIC("not implemented %s port=0x%lx size=%d data=0x%lx",
        name_.c_str(), port, size, value);
      break;
    }
  } else if (ir.base == VBE_PIO_BASE) {
    uint16_t value = *(uint16_t*)data;
    if (offset == 0) {
      // 0x1CE Index
      vbe_index_ = value;
    } else if (offset == 1) {
      // 0x1CF Data
      VbeIoWriteData(value);
    }
  }
}

void VgaDevice::WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
  PciDevice::WritePciConfigSpace(offset, data, length);

}

bool VgaDevice::IsTextMode() {
  return (gfx_registers_[6] & 0x1) == 0;
}

uint64_t VgaDevice::GetVRamAddress() {
  uint64_t base = 0;
  uint64_t __attribute__((unused)) size = 0;
  if (IsTextMode()) {
    /* Memory map select */
    switch ((gfx_registers_[6] >> 2) & 0b11)
    {
    case 0:
      base = 0xA0000;
      size = 0x20000;
      break;
    case 1:
      base = 0xA0000;
      size = 0x10000;
      break;
    case 2:
      base = 0xB0000;
      size = 0x08000;
      break;
    case 3:
      base = 0xB8000;
      size = 0x08000;
      break;
    }
  }
  return base;
}

void VgaDevice::GetCursorLocation(uint8_t* x, uint8_t* y, uint8_t* sel_start, uint8_t* sel_end) {
  uint16_t location = (crtc_registers_[0xE] << 8) | (crtc_registers_[0xF]);
  *sel_start = crtc_registers_[0xA] & 0x1F;
  *sel_end = crtc_registers_[0xB] & 0x1F;
  *y = location / 80;
  *x = location % 80;
}
