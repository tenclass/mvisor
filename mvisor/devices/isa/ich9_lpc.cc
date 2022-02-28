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

#include <asm/kvm.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <cstring>
#include "logger.h"
#include "pci_device.h"
#include "device_manager.h"
#include "machine.h"

#define ICH9_LPC_PMBASE                         0x40
#define ICH9_LPC_PMBASE_BASE_ADDRESS_MASK       Q35_MASK(32, 15, 7)
#define ICH9_LPC_PMBASE_RTE                     0x1
#define ICH9_LPC_PMBASE_DEFAULT                 0x1
#define ICH9_PMIO_SMI_EN_APMC_EN                (1 << 5)

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

#define ACPI_BITMASK_SLEEP_ENABLE               0x2000

#define PM_TIMER_FREQUENCY 3579545

/* get host real time in nanosecond */
static inline uint64_t get_clock_realtime(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec * 1000000000LL + (tv.tv_usec * 1000);
}

static uint64_t acpi_get_clock(uint64_t now)
{
  return (__int128_t)now * PM_TIMER_FREQUENCY / 1000000000;
}

class Ich9Lpc : public PciDevice {
 private:
  uint8_t acpi_gpe_regs_[16];
  uint16_t acpi_control_;
  uint8_t apm_control_;
  uint8_t apm_state_;
  uint16_t acpi_pm_event_status_;
  uint16_t acpi_pm_event_enable_;


  void UpdatePmBaseSci() {
    uint32_t pm_io_base = *(uint32_t*)(pci_header_.data + ICH9_LPC_PMBASE);
    uint8_t acpi_control = *(uint8_t*)(pci_header_.data + ICH9_LPC_ACPI_CTRL);
    if (acpi_control & ICH9_LPC_ACPI_CTRL_ACPI_EN) {
      pm_io_base &= ICH9_LPC_PMBASE_BASE_ADDRESS_MASK;
      AddIoResource(kIoResourceTypePio, pm_io_base, ICH9_PMIO_SIZE, "LPC PM");
    } else {
      RemoveIoResource(kIoResourceTypePio, "LPC PM");
    }
  }

  void UpdateRootComplexRegisterBLock() {
    uint32_t rcrb = *(uint32_t*)(pci_header_.data + ICH9_LPC_RCBA);
    if (rcrb & ICH9_LPC_RCBA_EN) {
      AddIoResource(kIoResourceTypeMmio, rcrb & ICH9_LPC_RCBA_BA_MASK, ICH9_CC_SIZE, "LPC RCRB");
    } else {
      RemoveIoResource(kIoResourceTypeMmio, "LPC RCRB");
    }
  }


 public:
  Ich9Lpc() {
    devfn_ = PCI_MAKE_DEVFN(0x1F, 0);
    
    pci_header_.vendor_id = 0x8086;
    pci_header_.device_id = 0x2918;
    pci_header_.class_code = 0x060100;
    pci_header_.revision_id = 2;
    pci_header_.header_type = PCI_MULTI_FUNCTION | PCI_HEADER_TYPE_NORMAL;
    pci_header_.subsys_vendor_id = 0x1af4;
    pci_header_.subsys_id = 0x1100;

    AddIoResource(kIoResourceTypePio, 0xB2, 2, "LPC APM");
  }

  void Reset() {
    PciDevice::Reset();

    bzero(&acpi_gpe_regs_, sizeof(acpi_gpe_regs_));
    acpi_control_ = 0;
    apm_control_ = apm_state_ = 0;
    acpi_pm_event_status_ = 1;
    acpi_pm_event_enable_ = 0;
  }

  void WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
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
      // MV_PANIC("activate irq remapping in LPC A-D");
    }
    if (ranges_overlap(offset, length, ICH9_LPC_PIRQE_ROUT, 4)) {
      /* activate irq remapping in LPC E-H */
      // MV_PANIC("activate irq remapping in LPC E-H");
    }
    if (ranges_overlap(offset, length, ICH9_LPC_GEN_PMCON_1, 8)) {
      MV_PANIC("ich9_lpc_pmcon_update(lpc);");
    }
  }

  void Read(const IoResource* ir, uint64_t offset, uint8_t* data, uint32_t size) {
    if (ir->base == 0xB2) { // APM IO
      if (offset == 0) {
        *data = apm_control_;
      } else {
        *data = apm_state_;
      }
      return;
    }
    if (offset >= 0x8 && offset < 0xC) {          // ACPI TMR
      MV_ASSERT(size == 4);
      *(uint32_t*)data = acpi_get_clock(get_clock_realtime()) & 0xFFFFFF;
    } else if (offset >= 0x0 && offset < 0x4) {   // ACPI Event
      MV_ASSERT(size == 2);
      if (offset == 0) {
        *(uint16_t*)data = acpi_pm_event_status_;
      } else {
        *(uint16_t*)data = acpi_pm_event_enable_;
      }
    } else if (offset >= 0x4 && offset < 0x8) {   // ACPI CNT
      MV_ASSERT(size == 2);
      *(uint16_t*)data = acpi_control_;
    } else if (offset >= 0x20 && offset < 0x30) { // ACPI GPE
      MV_ASSERT(size == 1);
      offset -= 0x20;
      data[0] = acpi_gpe_regs_[offset];
    } else if (offset >= 0x30 && offset < 0x38) { // ACPI SMI
      MV_ASSERT(size == 4);
      if (offset == 0x30) {
        // Tell SeaBIOS not to initialize SMM
        *(uint32_t*)data = ICH9_PMIO_SMI_EN_APMC_EN;
      } else {
        MV_PANIC("not supported");
      }
    } else {
      MV_PANIC("not supported read at 0x%x", ir->base + offset);
    }
  }

  void Write(const IoResource* ir, uint64_t offset, uint8_t* data, uint32_t size) {
    if (ir->base == 0xB2) { // APM IO
      if (offset == 0) {
        apm_control_ = *data;
        if (apm_control_ == 2) { // Enable ACPI
          acpi_control_ |= 1;
        } else if (apm_control_ == 3) { // Disable ACPI
          acpi_control_ &= ~1;
        } else {
          MV_PANIC("unknown apm control=0x%x", apm_control_);
        }
      } else {
        apm_state_ = *data;
      }
      return;
    }

    if (offset >= 0x0 && offset < 0x4) { // ACPI Event
      MV_ASSERT(size == 2);
      uint16_t value = *(uint16_t*)data;
      if (offset == 0) {
        acpi_pm_event_status_ &= ~value;
      } else {
        acpi_pm_event_enable_ = value;
      }
    } else if (offset >= 0x4 && offset < 0x8) { // ACPI CNT
      MV_ASSERT(size == 2);
      uint16_t value = *(uint16_t*)data;
      acpi_control_ = value & ~ACPI_BITMASK_SLEEP_ENABLE;
      if (value & ACPI_BITMASK_SLEEP_ENABLE) {
        AcpiSuspend((value >> 10) & 7);
      }
    } else if (offset >= 0x20 && offset < 0x30) {  // ACPI GPE
      MV_ASSERT(size == 1);
      offset -= 0x20;
      if (offset < 8) {
        acpi_gpe_regs_[offset] &= ~data[0]; // GPE_STS
      } else {
        acpi_gpe_regs_[offset] = data[0];   // GPE_EN
      }
    } else {
      MV_PANIC("not implemented %s base=0x%lx offset=0x%lx size=%d data=0x%lx",
        name_, ir->base, offset, size, *(uint64_t*)data);
    }
  }

  void AcpiSuspend(uint8_t type) {
    switch (type)
    {
    case 0: // soft power off
      manager_->machine()->Quit();
      break;
    case 1: // suspend request
      manager_->machine()->Quit();
      break;
    default:
      MV_LOG("unknown acpi suspend type=%d", type);
      break;
    }
  }
};

DECLARE_DEVICE(Ich9Lpc);
