/*
 * Since we want to implement QXL, so don't waste time on VGA
 * terrence.huang@tenclass.com
 */

#include <cstring>
#include <sys/mman.h>
#include "logger.h"
#include "device_manager.h"
#include "machine.h"
#include "vbe.h"
#include "device_interface.h"

#define VGA_ROM_PATH "../assets/vgabios-qxl.bin"
#define VGA_PIO_BASE    0x3C0
#define VGA_PIO_SIZE    0x20
#define VBE_PIO_BASE    0x1CE
#define VBE_PIO_SIZE    2
#define VBE_LINEAR_FRAMEBUFFER_BASE 0xE0000000

// When LFB mode disabled, the tradition VGA video memory address is used
#define VGA_MMIO_BASE   0x000A0000
#define VGA_MMIO_SIZE   0x00020000


class Qxl : public PciDevice, public DisplayInterface {
 private:
  uint8_t misc_output_reg_ = 0;
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
  uint8_t status_registers_[2] = { 0 };

  uint16_t vbe_version_;
  uint16_t vbe_index_ = 0;
  uint16_t vbe_registers_[16] = { 0 };

  uint32_t vram_size_ = 0;
  uint8_t* vram_base_ = nullptr;

  uint8_t* vram_map_select_ = nullptr;
  uint32_t vram_map_select_size_;
  uint8_t* vram_read_select_ = nullptr;

  uint16_t width_;
  uint16_t height_;
  uint16_t bpp_;

  std::vector<DisplayChangeListener> display_change_listerners_;


  void ResetRegisters() {
    bzero(vbe_registers_, sizeof(vbe_registers_));
  }

  void RegisterDisplayChangeListener(DisplayChangeListener callback) {
    display_change_listerners_.push_back(callback);
  }

  void VbeReadPort(uint64_t port, uint16_t* data) {
    if (port == 0x1CE) {
      *data = vbe_index_;
      return;
    }
    if (vbe_index_ < VBE_DISPI_INDEX_NB) {
      if (vbe_registers_[VBE_DISPI_INDEX_ENABLE] & VBE_DISPI_GETCAPS) {
        /* VBE initialization will enable and get capabilities and then disable */
        const uint16_t max_values[] = {
          0, VBE_DISPI_MAX_XRES, VBE_DISPI_INDEX_YRES, VBE_DISPI_MAX_BPP
        };
        MV_ASSERT(vbe_index_ < sizeof(max_values) / sizeof(uint16_t));
        *data = max_values[vbe_index_];
      } else {
        *data = vbe_registers_[vbe_index_];
      }
    } else if (vbe_index_ == VBE_DISPI_INDEX_VIDEO_MEMORY_64K) {
      *data = vram_size_ >> 16;
    } else {
      *data = 0;
      MV_PANIC("invalid read index = %d", vbe_index_);
    }
  }

  void VbeWritePort(uint64_t port, uint16_t value) {
    if (port == 0x1CE) { // index
      if (value > VBE_DISPI_INDEX_NB) {
        MV_PANIC("invalid vbe index 0x%x", value);
      }
      vbe_index_ = value;
    } else if (port == 0x1CF) { // data
      switch (vbe_index_)
      {
      case VBE_DISPI_INDEX_ID:
        vbe_version_ = value;
        break;
      case VBE_DISPI_INDEX_XRES:
        width_ = value;
        break;
      case VBE_DISPI_INDEX_YRES:
        height_ = value;
        break;
      case VBE_DISPI_INDEX_BPP:
        bpp_ = value;
        break;
      case VBE_DISPI_INDEX_ENABLE:
        MV_LOG("set vbe enable %x to %x %dx%d bpp=%d", vbe_registers_[4], value,
          vbe_registers_[1], vbe_registers_[2], vbe_registers_[3]);
        if (value & 1) {
          UpdateRenderer();
        }
        break;
      case VBE_DISPI_INDEX_BANK:
        vram_read_select_ = vram_base_ + (value << 16);
        break;
      }
      vbe_registers_[vbe_index_] = value;
    }
  }

  void VgaReadPort(uint64_t port, uint8_t* data, uint32_t size) {
    switch (port)
    {
    case 0x3C0:
      MV_ASSERT(size == 1);
      *data = attribute_index_;
      break;
    case 0x3C1:
      MV_ASSERT(size == 1);
      *data = attribute_registers_[attribute_index_ & 0x7F];
      break;
    case 0x3C4:
      MV_ASSERT(size == 1);
      *data = sequence_index_;
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
      *data = misc_output_reg_;
      break;
    case 0x3CE:
      MV_ASSERT(size == 1);
      *data = gfx_index_;
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
      attribute_index_ &= ~0x80; // Clears attribute flip-flop
      status_registers_[1] ^= 9;
      *data = status_registers_[1];
      break;
    default:
      MV_PANIC("not implemented %s port=0x%lx size=%d data=0x%lx",
        name_, port, size, *(uint64_t*)data);
      break;
    }
  }

  void VgaWritePort(uint64_t port, uint8_t* data, uint32_t size) {
    uint8_t value = *data;
    switch (port)
    {
    case 0x3C0:
      MV_ASSERT(size == 1);
      if (attribute_index_ & 0x80) { // set data
        attribute_index_ &= ~0x80;
        attribute_registers_[attribute_index_] = value;
      } else { // set index
        attribute_index_ = 0x80 | value;
        if (attribute_index_ & 0x20) {
          // renderer changed event
          UpdateRenderer();
        }
      }
      break;
    case 0x3C2:
      MV_ASSERT(size == 1);
      misc_output_reg_ = value & ~0x10;
      break;
    case 0x3C4:
      sequence_index_ = value;
      if (size == 2) {
        sequence_registers_[sequence_index_] = data[1];
        if (sequence_index_ == 4) {
          sequence_registers_[sequence_index_] &= 0b1110;
        }
      }
      break;
    case 0x3C5:
      MV_ASSERT(size == 1);
      sequence_registers_[sequence_index_] = value;
      break;
    case 0x3C6:
      MV_ASSERT(value == 0xFF); // pallete mask
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
        if (gfx_index_ == 4 || gfx_index_ == 6) {
          UpdateVRamMemoryMap();
        }
      }
      break;
    case 0x3CF:
      MV_ASSERT(size == 1);
      gfx_registers_[gfx_index_] = value;
      if (gfx_index_ == 4 || gfx_index_ == 6) {
        UpdateVRamMemoryMap();
      }
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
        name_, port, size, value);
      break;
    }
  }

  void UpdateVRamMemoryMap() {
    const int map_types[][2] = {
      { 0xA0000, 0x20000 }, { 0xA0000, 0x10000 },
      { 0xB0000, 0x08000 }, { 0xB8000, 0x08000 }
    };
    /* Memory map select */
    int index = (gfx_registers_[6] >> 2) & 0b11;
    int read_index = gfx_registers_[4] & 0b11;
    vram_map_select_size_ = map_types[index][1];
    vram_map_select_ = vram_base_ + map_types[index][0] - VGA_MMIO_BASE;
    vram_read_select_ = vram_base_ + vram_map_select_size_ * read_index;
    MV_LOG("map index=%d read_index=%d", index, read_index);
  }

  void UpdateRenderer() {
    for (auto listener : display_change_listerners_) {
      listener();
    }
  }

 public:
  Qxl() {
    devfn_ = PCI_MAKE_DEVFN(1, 0);
    
    /* PCI config */
    pci_header_.vendor_id = 0x1b36;
    pci_header_.device_id = 0x0100;
    // pci_header_.vendor_id = 0x1234;
    // pci_header_.device_id = 0x1111;
    pci_header_.class_code = 0x030000;
    pci_header_.revision_id = 5;
    pci_header_.header_type = PCI_HEADER_TYPE_NORMAL;
    pci_header_.subsys_vendor_id = 0x1af4;
    pci_header_.subsys_id = 0x1100;
    pci_header_.command = PCI_COMMAND_IO | PCI_COMMAND_MEMORY;
    pci_header_.irq_pin = 1;

    /* Initialize rom data and rom bar size */
    LoadRomFile(VGA_ROM_PATH);

    
    AddPciBar(0, 384 << 20, kIoResourceTypeRam);   /* 384MB vgamem */
    // AddPciBar(1, 8 << 20, kIoResourceTypeMmio);     /* 8MB vram */
    AddPciBar(2, 8192, kIoResourceTypeMmio);        /* MMIO */
    AddPciBar(3, 32, kIoResourceTypePio);           /* PIO */

    /* 64MB vram */
    vram_size_ = 0x04000000;
    vram_base_ = (uint8_t*)mmap(nullptr, vram_size_, PROT_READ | PROT_WRITE,
      MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
    pci_bars_[0].host_memory = vram_base_;
    

    AddIoResource(kIoResourceTypePio, VGA_PIO_BASE, VGA_PIO_SIZE, "VGA IO");
    AddIoResource(kIoResourceTypePio, VBE_PIO_BASE, VBE_PIO_SIZE, "VBE IO");
    AddIoResource(kIoResourceTypeMmio, VGA_MMIO_BASE, VGA_MMIO_SIZE, "VGA MMIO");

    ResetRegisters();
  }

  ~Qxl() {
    munmap((void*)vram_base_, vram_size_);
  }

  uint8_t* GetVRamHostAddress() {
    return vram_map_select_;
  }

  void GetCursorLocation(uint8_t* x, uint8_t* y, uint8_t* sel_start, uint8_t* sel_end) {
    uint16_t location = (crtc_registers_[0xE] << 8) | (crtc_registers_[0xF]);
    *sel_start = crtc_registers_[0xA] & 0x1F;
    *sel_end = crtc_registers_[0xB] & 0x1F;
    *y = location / 80;
    *x = location % 80;
  }

  void GetDisplayMode(DisplayMode *mode, uint16_t* w, uint16_t* h, uint16_t* b) {
    if ((gfx_registers_[6] & 0x1) == 0) {
      *mode = kDisplayTextMode;
    } else if (vbe_registers_[VBE_DISPI_INDEX_ENABLE] & VBE_DISPI_ENABLED) {
      *mode = kDisplayVbeMode;
    } else {
      *mode = kDisplayVgaMode;
    }
    if (*mode == kDisplayTextMode) {
      *w = 640;
      *h = 400;
      *b = 8;
    } else {
      *w = width_;
      *h = height_;
      *b = bpp_;
    }
  }

  const uint8_t* GetPallete() const {
    return pallete_;
  }


  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
    uint64_t port = ir.base + offset;

    if (ir.base == VGA_MMIO_BASE) {
      memcpy(data, vram_read_select_ + offset, size);
    } else if (ir.base == VGA_PIO_BASE) {
      VgaReadPort(port, data, size);
    } else if (ir.base == VBE_PIO_BASE) {
      MV_ASSERT(size == 2);
      VbeReadPort(port, (uint16_t*)data);
    }
  }

  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
    uint64_t port = ir.base + offset;
    if (ir.base == VGA_MMIO_BASE) {
      memcpy(vram_read_select_ + offset, data, size);
    } else if (ir.base == VGA_PIO_BASE) {
      VgaWritePort(port, data, size);
    } else if (ir.base == VBE_PIO_BASE) {
      VbeWritePort(port, *(uint16_t*)data);
    } else {
      memcpy(vram_read_select_ + offset, data, size);
    }
  }

};

DECLARE_DEVICE(Qxl);
