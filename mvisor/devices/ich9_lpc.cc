#include "devices/ich9_lpc.h"
#include "logger.h"
#include "device_manager.h"

#define ICH9_LPC_PMBASE                         0x40
#define ICH9_LPC_PMBASE_BASE_ADDRESS_MASK       Q35_MASK(32, 15, 7)
#define ICH9_LPC_PMBASE_RTE                     0x1
#define ICH9_LPC_PMBASE_DEFAULT                 0x1
#define ICH9_PMIO_SMI_EN_APMC_EN       (1 << 5)

#define ICH9_LPC_ACPI_CTRL                      0x44
#define ICH9_LPC_ACPI_CTRL_ACPI_EN              0x80
#define ICH9_LPC_ACPI_CTRL_SCI_IRQ_SEL_MASK     Q35_MASK(8, 2, 0)
#define ICH9_LPC_ACPI_CTRL_9                    0x0
#define ICH9_LPC_ACPI_CTRL_10                   0x1
#define ICH9_LPC_ACPI_CTRL_11                   0x2
#define ICH9_LPC_ACPI_CTRL_20                   0x4
#define ICH9_LPC_ACPI_CTRL_21                   0x5
#define ICH9_LPC_ACPI_CTRL_DEFAULT              0x0

#define ICH9_LPC_PIRQA_ROUT                     0x60
#define ICH9_LPC_PIRQB_ROUT                     0x61
#define ICH9_LPC_PIRQC_ROUT                     0x62
#define ICH9_LPC_PIRQD_ROUT                     0x63

#define ICH9_LPC_PIRQE_ROUT                     0x68
#define ICH9_LPC_PIRQF_ROUT                     0x69
#define ICH9_LPC_PIRQG_ROUT                     0x6a
#define ICH9_LPC_PIRQH_ROUT                     0x6b

#define ICH9_LPC_GEN_PMCON_1                    0xa0
#define ICH9_LPC_GEN_PMCON_1_SMI_LOCK           (1 << 4)
#define ICH9_LPC_GEN_PMCON_2                    0xa2
#define ICH9_LPC_GEN_PMCON_3                    0xa4
#define ICH9_LPC_GEN_PMCON_LOCK                 0xa6

#define ICH9_LPC_RCBA                           0xf0
#define ICH9_LPC_RCBA_BA_MASK                   Q35_MASK(32, 31, 14)
#define ICH9_LPC_RCBA_EN                        0x1
#define ICH9_LPC_RCBA_DEFAULT                   0x0
/* 16KB. Chipset configuration registers */
#define ICH9_CC_SIZE                            (16 * 1024)

#define ICH9_PMIO_SIZE                          128

Ich9LpcDevice::Ich9LpcDevice(DeviceManager* manager)
  : PciDevice(manager) {
  name_ = "ich9-lpc";
  
  header_.vendor_id = 0x8086;
  header_.device_id = 0x2918;
  header_.class_code = 0x060100;
  header_.revision_id = 2;
  header_.header_type = PCI_HEADER_TYPE_NORMAL;
  header_.subsys_vendor_id = 0x1af4;
  header_.subsys_id = 0x1100;

  devfn_ = PCI_MAKE_DEVFN(0x1f, 0);
}

Ich9LpcDevice::~Ich9LpcDevice() {
}

void Ich9LpcDevice::WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
  PciDevice::WritePciConfigSpace(offset, data, length);

  if (ranges_overlap(offset, length, ICH9_LPC_PMBASE, 4) ||
    ranges_overlap(offset, length, ICH9_LPC_ACPI_CTRL, 1)) {
      /* pm io base || lsacpi enable, SCI: IRQ9 000b = irq9*/
    UpdatePmBaseSci();
  }
  if (ranges_overlap(offset, length, ICH9_LPC_RCBA, 4)) {
    /* set root complex register block BAR */
    UpdateRootComplexRegisterBLock();
  }
  if (ranges_overlap(offset, length, ICH9_LPC_PIRQA_ROUT, 4)) {
    /* activate irq remapping in LPC A-D */
  }
  if (ranges_overlap(offset, length, ICH9_LPC_PIRQE_ROUT, 4)) {
    /* activate irq remapping in LPC E-H */
  }
  if (ranges_overlap(offset, length, ICH9_LPC_GEN_PMCON_1, 8)) {
    MV_PANIC("ich9_lpc_pmcon_update(lpc);");
  }
}

void Ich9LpcDevice::UpdatePmBaseSci() {
  uint32_t pm_io_base = *(uint32_t*)(header_.data + ICH9_LPC_PMBASE);
  uint8_t acpi_control = *(uint8_t*)(header_.data + ICH9_LPC_ACPI_CTRL);
  if (acpi_control & ICH9_LPC_ACPI_CTRL_ACPI_EN) {
    pm_io_base &= ICH9_LPC_PMBASE_BASE_ADDRESS_MASK;
    AddIoResource(kIoResourceTypePio, pm_io_base, ICH9_PMIO_SIZE, "pm-io");
  } else {
    RemoveIoResource(kIoResourceTypePio, "pm-io");
  }
}

void Ich9LpcDevice::UpdateRootComplexRegisterBLock() {
  uint32_t rcrb = *(uint32_t*)(header_.data + ICH9_LPC_RCBA);
  if (rcrb & ICH9_LPC_RCBA_EN) {
    AddIoResource(kIoResourceTypeMmio, rcrb & ICH9_LPC_RCBA_BA_MASK, ICH9_CC_SIZE, "rcba");
  } else {
    RemoveIoResource(kIoResourceTypeMmio, "rcba");
  }
}

void Ich9LpcDevice::Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  if (offset >= 0x30 && offset < 0x38) { // SMI
    MV_ASSERT(size == 4);
    if (offset == 0x30) {
      // Tell SeaBIOS not to initialize SMM
      *(uint32_t*)data = ICH9_PMIO_SMI_EN_APMC_EN;
    } else {
      MV_PANIC("not supported");
    }
  }
}

void Ich9LpcDevice::Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  MV_PANIC("not implemented %s base=0x%lx offset=0x%lx size=%d data=0x%lx",
    name_.c_str(), ir.base, offset, size, *(uint64_t*)data);
}
