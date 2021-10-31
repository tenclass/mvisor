#include "devices/firmware_config.h"
#include <cstring>
#include "logger.h"
#include "device_manager.h"
#include "machine.h"

#define FW_CFG_ACPI_DEVICE_ID	"QEMU0002"

/* selector key values for "well-known" fw_cfg entries */
#define FW_CFG_SIGNATURE	0x00
#define FW_CFG_ID		0x01
#define FW_CFG_UUID		0x02
#define FW_CFG_RAM_SIZE		0x03
#define FW_CFG_NOGRAPHIC	0x04
#define FW_CFG_NB_CPUS		0x05
#define FW_CFG_MACHINE_ID	0x06
#define FW_CFG_KERNEL_ADDR	0x07
#define FW_CFG_KERNEL_SIZE	0x08
#define FW_CFG_KERNEL_CMDLINE	0x09
#define FW_CFG_INITRD_ADDR	0x0a
#define FW_CFG_INITRD_SIZE	0x0b
#define FW_CFG_BOOT_DEVICE	0x0c
#define FW_CFG_NUMA		0x0d
#define FW_CFG_BOOT_MENU	0x0e
#define FW_CFG_MAX_CPUS		0x0f
#define FW_CFG_KERNEL_ENTRY	0x10
#define FW_CFG_KERNEL_DATA	0x11
#define FW_CFG_INITRD_DATA	0x12
#define FW_CFG_CMDLINE_ADDR	0x13
#define FW_CFG_CMDLINE_SIZE	0x14
#define FW_CFG_CMDLINE_DATA	0x15
#define FW_CFG_SETUP_ADDR	0x16
#define FW_CFG_SETUP_SIZE	0x17
#define FW_CFG_SETUP_DATA	0x18
#define FW_CFG_FILE_DIR		0x19

#define FW_CFG_FILE_FIRST	0x20
#define FW_CFG_FILE_SLOTS_MIN	0x10

#define FW_CFG_WRITE_CHANNEL	0x4000
#define FW_CFG_ARCH_LOCAL	0x8000
#define FW_CFG_ENTRY_MASK	(~(FW_CFG_WRITE_CHANNEL | FW_CFG_ARCH_LOCAL))

#define FW_CFG_ACPI_TABLES      (FW_CFG_ARCH_LOCAL + 0)
#define FW_CFG_SMBIOS_ENTRIES   (FW_CFG_ARCH_LOCAL + 1)
#define FW_CFG_IRQ0_OVERRIDE    (FW_CFG_ARCH_LOCAL + 2)
#define FW_CFG_E820_TABLE       (FW_CFG_ARCH_LOCAL + 3)
#define FW_CFG_HPET             (FW_CFG_ARCH_LOCAL + 4)

#define FW_CFG_INVALID		0xffff

/* width in bytes of fw_cfg control register */
#define FW_CFG_CTL_SIZE		0x02

/* fw_cfg "file name" is up to 56 characters (including terminating nul) */
#define FW_CFG_MAX_FILE_PATH	56

/* size in bytes of fw_cfg signature */
#define FW_CFG_SIG_SIZE 4

/* FW_CFG_ID bits */
#define FW_CFG_VERSION		0x01
#define FW_CFG_VERSION_DMA	0x02

/* fw_cfg file directory entry type */
struct fw_cfg_file {
	uint32_t size;
	uint16_t select;
	uint16_t reserved;
	char name[FW_CFG_MAX_FILE_PATH];
};

/* FW_CFG_DMA_CONTROL bits */
#define FW_CFG_DMA_CTL_ERROR	0x01
#define FW_CFG_DMA_CTL_READ	0x02
#define FW_CFG_DMA_CTL_SKIP	0x04
#define FW_CFG_DMA_CTL_SELECT	0x08
#define FW_CFG_DMA_CTL_WRITE	0x10

#define FW_CFG_DMA_SIGNATURE    0x51454d5520434647ULL /* "QEMU CFG" */

/* Control as first field allows for different structures selected by this
 * field, which might be useful in the future
 */
struct fw_cfg_dma_access {
	uint32_t control;
	uint32_t length;
	uint64_t address;
};

#define FW_CFG_VMCOREINFO_FILENAME "etc/vmcoreinfo"

#define FW_CFG_VMCOREINFO_FORMAT_NONE 0x0
#define FW_CFG_VMCOREINFO_FORMAT_ELF 0x1

struct fw_cfg_vmcoreinfo {
	uint16_t host_format;
	uint16_t guest_format;
	uint32_t size;
	uint64_t paddr;
};

#define FW_CFG_IO_BASE     0x510
#define FW_CFG_DMA_IO_BASE    0x514

FirmwareConfigDevice::FirmwareConfigDevice(DeviceManager* manager)
  : Device(manager) {
  name_ = "fw_cfg_io";

  InitializeConfig();

  AddIoResource(kIoResourceTypePio, FW_CFG_IO_BASE, 2);
  AddIoResource(kIoResourceTypePio, FW_CFG_DMA_IO_BASE, 8);
}

void FirmwareConfigDevice::OnWrite(uint64_t base, uint8_t* data, uint32_t size) {
  if (base == FW_CFG_IO_BASE && size == 2) {
    current_index_ = *(uint16_t*)data;
    current_offset_ = 0;
    MV_LOG("select entry %d", current_index_);
  } else if (base >= FW_CFG_DMA_IO_BASE) {
    if (size == 4) {
      if (base == FW_CFG_DMA_IO_BASE) { // High 32bit address
        dma_address_ = be32toh(*(uint32_t*)data);
        dma_address_ <<= 32;
      } else if (base == FW_CFG_DMA_IO_BASE + 4) { // Low 32bit address
        dma_address_ |= be32toh(*(uint32_t*)data);
        DmaTransfer();
      }
    } else if (size == 8) {
      dma_address_ = be64toh(*(uint64_t*)data);
      DmaTransfer();
    }
  } else {
    MV_PANIC("not implemented OnWrite for %s base=0x%lx size=%d", name_.c_str(), base, size);
  }
}

void FirmwareConfigDevice::OnRead(uint64_t base, uint8_t* data, uint32_t size) {
  if (base == FW_CFG_IO_BASE + 1) {
    auto it = config_.find(current_index_);
    if (it == config_.end()) {
      MV_PANIC("config entry %d not found", current_index_);
    }
    while (size--) {
      if (current_offset_ < it->second.size()) {
        *data++ = it->second[current_offset_++];
      } else {
        *data++ = 0;
      }
    }
  } else {
    MV_PANIC("not implemented OnRead for %s base=0x%lx size=%d", name_.c_str(), base, size);
  }
}

void FirmwareConfigDevice::DmaTransfer() {
  fw_cfg_dma_access* dma = (fw_cfg_dma_access*)manager_->TranslateGuestMemory(dma_address_);
  dma_address_ = 0;

  dma->control = be32toh(dma->control);
  dma->address = be64toh(dma->address);
  dma->length = be32toh(dma->length);
  
  if (dma->control & FW_CFG_DMA_CTL_SELECT) {
    current_index_ = dma->control >> 16;
    current_offset_ = 0;
  }

  MV_LOG("control=%08x address=%016lx len=%x", dma->control, dma->address, dma->length);
  auto it = config_.find(current_index_);
  if (it == config_.end()) {
    dma->control = be32toh(FW_CFG_DMA_CTL_ERROR);
    MV_LOG("config entry not found 0x%x", current_index_);
    return;
  }

  uint8_t* data = (uint8_t*)manager_->TranslateGuestMemory(dma->address);
  if (dma->control & FW_CFG_DMA_CTL_READ) {
    uint32_t size = it->second.size() - current_offset_;
    if (size > dma->length)
      size = dma->length;
    memcpy(data, it->second.data() + current_offset_, size);
    current_offset_ += size;
  } else if (dma->control & FW_CFG_DMA_CTL_WRITE) {
    MV_PANIC("not supported");
  }
  dma->control = 0;
}

void FirmwareConfigDevice::InitializeConfig() {
  SetConfigBytes(FW_CFG_SIGNATURE, "QEMU");
  uint32_t version = FW_CFG_VERSION | FW_CFG_VERSION_DMA;
  SetConfigUInt32(FW_CFG_ID, version);
  SetConfigUInt32(FW_CFG_FILE_DIR, 0);

  int num_vcpus = manager_->machine()->num_vcpus();
  SetConfigUInt16(FW_CFG_NB_CPUS, num_vcpus);
  SetConfigUInt16(FW_CFG_MAX_CPUS, num_vcpus);
  uint64_t numa_cfg[num_vcpus + 1] = { 0 };
  SetConfigBytes(FW_CFG_NUMA, std::string((const char*)numa_cfg, sizeof(numa_cfg)));
  SetConfigUInt16(FW_CFG_NOGRAPHIC, 0);
  SetConfigUInt32(FW_CFG_IRQ0_OVERRIDE, 1);
}

void FirmwareConfigDevice::SetConfigBytes(uint16_t index, std::string bytes) {
  config_[index] = bytes;
}

void FirmwareConfigDevice::SetConfigUInt32(uint16_t index, uint32_t value) {
  config_[index] = std::string((const char*)&value, sizeof(value));
}

void FirmwareConfigDevice::SetConfigUInt16(uint16_t index, uint16_t value) {
  config_[index] = std::string((const char*)&value, sizeof(value));
}
