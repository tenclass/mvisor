#include "devices/vga.h"
#include <cstring>
#include "logger.h"
#include "device_manager.h"

#define VGA_ROM_PATH "../assets/vgabios-qxl.bin"
#define VGA_PIO_BASE    0x3C0
#define VGA_PIO_SIZE    0x20
#define VBE_PIO_BASE    0x1CE
#define VBE_PIO_SIZE    2

VgaDevice::VgaDevice(DeviceManager* manager)
  : PciDevice(manager) {
  name_ = "vga";
  
  /* PCI config */
  header_.vendor_id = 0x1b36;
  header_.device_id = 0x0100;
  header_.class_code = 0x030000;
  header_.revision_id = 5;
  header_.header_type = PCI_HEADER_TYPE_NORMAL;
  header_.subsys_vendor_id = 0x1af4;
  header_.subsys_id = 0x1100;
  header_.command = PCI_COMMAND_IO | PCI_COMMAND_MEMORY;
  devfn_ = PCI_MAKE_DEVFN(2, 0);

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
  header_.bar[3] = 1;

  AddIoResource(kIoResourceTypePio, VGA_PIO_BASE, VGA_PIO_SIZE, "vga-io");
  AddIoResource(kIoResourceTypePio, VBE_PIO_BASE, VBE_PIO_SIZE, "vbe-io");

  ResetRegisters();
}

VgaDevice::~VgaDevice() {
}

void VgaDevice::ResetRegisters() {
  bzero(vbe_registers_, sizeof(vbe_registers_));
}

void VgaDevice::VbeIoReadData(uint16_t* data) {
  uint32_t val;
  if (vbe_index_ < VBE_DISPI_INDEX_NB) {
      if (vbe_registers_[VBE_DISPI_INDEX_ENABLE] & VBE_DISPI_GETCAPS) {
          switch(vbe_index_) {
              /* XXX: do not hardcode ? */
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
      *data = attribute_index_;
      break;
    case 0x3CC:
      *data = misc_ouput_reg_;
      break;
    case 0x3DA:
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
    MV_LOG("index = %d data=%x", vbe_index_, *(uint16_t*)data);
  }
}

void VgaDevice::Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  uint64_t port = ir.base + offset;
  // MV_LOG("port=0x%lx size=%d data=0x%lx", port, size, *(uint64_t*)data);
  if (ir.base == VGA_PIO_BASE) {
    uint8_t value = *data;
    switch (port)
    {
    case 0x3C0:
      attribute_index_ = value;
      break;
    case 0x3C2:
      misc_ouput_reg_ = value & ~0x10;
      break;
    case 0x3C4:
      sequence_index_ = value;
      break;
    case 0x3C6:
      MV_ASSERT(value == 0xFF);
      break;
    case 0x3C8:
      pallete_write_address_ = value * 3;
      break;
    case 0x3C9:
      pallete_[pallete_write_address_++] = value;
      break;
    case 0x3CE:
      graphics_control_address_ = value;
      break;
    case 0x3D4:
      crtc_index_ = value;
      break;
    default:
      MV_PANIC("not implemented %s port=0x%lx size=%d data=0x%lx",
        name_.c_str(), port, size, value);
      break;
    }
  } else if (ir.base == VBE_PIO_BASE) {
    uint16_t value = *(uint16_t*)data;
    if (offset == 0) {
      // Index
      vbe_index_ = value;
    } else if (offset == 1) {
      // Data
      VbeIoWriteData(*(uint16_t*)data);
    }
  }
}

void VgaDevice::WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
  PciDevice::WritePciConfigSpace(offset, data, length);

}
