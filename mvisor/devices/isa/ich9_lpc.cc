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
#include "device_interface.h"
#include "machine.h"
#include "pb/ich9_lpc.pb.h"

#define ICH9_LPC_PMBASE                         0x40
#define ICH9_LPC_PMBASE_BASE_ADDRESS_MASK       Q35_MASK(32, 15, 7)
#define ICH9_LPC_PMBASE_RTE                     0x1
#define ICH9_LPC_PMBASE_DEFAULT                 0x1

#define ICH9_LPC_PMIO_PM1_CTRL_SMI_EN           (1)
#define ICH9_LPC_PMIO_ACPI_SMI_EN_APMC_EN       (1 << 5)

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

class Ich9Lpc : public PciDevice, public PowerDownInterface {
 private:
  Ich9LpcState  state_;
  bool          initialized_pmio_ = false;
  bool          initialized_rcrb_ = false;
  int           system_control_irq_ = -1;

  void UpdateSystemControlIrq() {
    if (system_control_irq_ == -1)
      return;
    auto acpi = state_.mutable_acpi();
    int level = (acpi->pm1_enable() & acpi->pm1_status() & 0x521) || (acpi->gpe0_enable() & acpi->gpe0_status());
    manager_->SetGsiLevel(system_control_irq_, level);
  }

  void UpdatePmBase() {
    /* PM IO base should be 0xB000 */
    uint32_t pm_io_base = *(uint32_t*)(pci_header_.data + ICH9_LPC_PMBASE);
    uint8_t acpi_control = *(uint8_t*)(pci_header_.data + ICH9_LPC_ACPI_CTRL);
    if (acpi_control & ICH9_LPC_ACPI_CTRL_ACPI_EN) {
      if (!initialized_pmio_) {
        pm_io_base &= ICH9_LPC_PMBASE_BASE_ADDRESS_MASK;
        AddIoResource(kIoResourceTypePio, pm_io_base, ICH9_PMIO_SIZE, "LPC PM");
        initialized_pmio_ = true;
        /* update system control irq */
        auto irq_select = acpi_control & ICH9_LPC_ACPI_CTRL_SCI_IRQ_SEL_MASK;
        MV_ASSERT(irq_select == ICH9_LPC_ACPI_CTRL_9);
        system_control_irq_ = 9;
      }
    } else {
      if (initialized_pmio_) {
        RemoveIoResource(kIoResourceTypePio, "LPC PM");
        initialized_pmio_ = false;
        system_control_irq_ = -1;
      }
    }
  }

  void UpdateRootComplexRegisterBLock() {
    uint32_t rcrb = *(uint32_t*)(pci_header_.data + ICH9_LPC_RCBA);
    if (rcrb & ICH9_LPC_RCBA_EN) {
      if (!initialized_rcrb_) {
        AddIoResource(kIoResourceTypeMmio, rcrb & ICH9_LPC_RCBA_BA_MASK, ICH9_CC_SIZE, "LPC RCRB");
        initialized_rcrb_ = true;
      }
    } else {
      if (initialized_rcrb_) {
        RemoveIoResource(kIoResourceTypeMmio, "LPC RCRB");
        initialized_rcrb_ = false;
      }
    }
  }


 public:
  Ich9Lpc() {
    slot_ = 31;
    function_ = 0;
    
    pci_header_.vendor_id = 0x8086;
    pci_header_.device_id = 0x2918;
    pci_header_.class_code = 0x060100;
    pci_header_.revision_id = 2;
    pci_header_.header_type = PCI_MULTI_FUNCTION | PCI_HEADER_TYPE_NORMAL;
    pci_header_.subsys_vendor_id = 0x1AF4;
    pci_header_.subsys_id = 0x1100;

    AddIoResource(kIoResourceTypePio, 0xB2, 2, "LPC APM");

    /* https://qemu.readthedocs.io/en/v6.2.0/specs/acpi_pci_hotplug.html */
    AddIoResource(kIoResourceTypePio, 0xAE00, 20, "ACPI PCI HOTPLUG");
  }

  bool SaveState(MigrationWriter* writer) {
    writer->WriteProtobuf("LPC", state_);
    return PciDevice::SaveState(writer);
  }

  bool LoadState(MigrationReader* reader) {
    if (!PciDevice::LoadState(reader)) {
      return false;
    }
    if (!reader->ReadProtobuf("LPC", state_)) {
      return false;
    }
    UpdatePmBase();
    UpdateRootComplexRegisterBLock();
    return true;
  }

  void Reset() {
    PciDevice::Reset();
    state_.Clear();
    state_.mutable_acpi()->set_smi_enable(ICH9_LPC_PMIO_ACPI_SMI_EN_APMC_EN);
    UpdatePmBase();
  }

  void WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
    PciDevice::WritePciConfigSpace(offset, data, length);

    if (ranges_overlap(offset, length, ICH9_LPC_PMBASE, 4) ||
      ranges_overlap(offset, length, ICH9_LPC_ACPI_CTRL, 1)) {
        /* pm io base || lsacpi enable, SCI: IRQ9 000b = irq9*/
      UpdatePmBase();
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
    }
    if (ranges_overlap(offset, length, ICH9_LPC_GEN_PMCON_1, 8)) {
      MV_PANIC("ich9_lpc_pmcon_update(lpc);");
    }
  }

  void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    if (resource->base == 0xB2) { // APM IO
      auto &apm = state_.apm();
      if (offset == 0) {
        *data = apm.control();
      } else {
        *data = apm.status();
      }
      return;
    } else if (resource->base == 0xAE00) { // ACPI PCI HOTPLUG
      if (offset == 0x0C) { // PCI removability status
        /* disable all PCIs hotplug */
        bzero(data, size);
      }
      return;
    }

    /* PM IO */
    MV_ASSERT(resource->length == ICH9_PMIO_SIZE);
    auto &acpi = state_.acpi();
    uint64_t value;

    switch (offset)
    {
    case 0x00:
      value = acpi.pm1_status();
      break;
    case 0x02:
      value = acpi.pm1_enable();
      break;
    case 0x04:
      value = acpi.pm1_control();
      break;
    case 0x08:
      value = acpi_get_clock(get_clock_realtime()) & 0xFFFFFF;
      break;
    case 0x20 ... 0x27:
      value = acpi.gpe0_status() >> ((offset - 0x20) << 3);
      break;
    case 0x28 ... 0x2F:
      value = acpi.gpe0_enable() >> ((offset - 0x28) << 3);
      break;
    case 0x30:
      value = acpi.smi_enable();
      break;
    case 0x34:
      value = acpi.smi_status();
    default:
      MV_PANIC("not supported reading at ACPI offset=0x%x", offset);
      break;
    }

    memcpy(data, &value, size);
  }

  void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    auto acpi = state_.mutable_acpi();
  
    if (resource->base == 0xB2) { // APM IO
      auto apm = state_.mutable_apm();
      if (offset == 0) {
        apm->set_control(*data);
        if (apm->control() == 2) { // Enable ACPI
          acpi->set_pm1_control(acpi->pm1_control() | ICH9_LPC_PMIO_PM1_CTRL_SMI_EN);
        } else if (apm->control() == 3) { // Disable ACPI
          acpi->set_pm1_control(acpi->pm1_control() & ~ICH9_LPC_PMIO_PM1_CTRL_SMI_EN);
        } else {
          MV_PANIC("unknown apm control=0x%x", *data);
        }
      } else {
        apm->set_status(*data);
      }
      return;
    }

    /* PM IO */
    MV_ASSERT(resource->length == ICH9_PMIO_SIZE);
    uint64_t value = 0;
    memcpy(&value, data, size);
  
    switch (offset)
    {
    case 0x00:
      acpi->set_pm1_status(acpi->pm1_status() & ~value);
      UpdateSystemControlIrq();
      break;
    case 0x02:
      acpi->set_pm1_enable(value);
      UpdateSystemControlIrq();
      break;
    case 0x04:
      acpi->set_pm1_control(value & ~ACPI_BITMASK_SLEEP_ENABLE);
      if (value & ACPI_BITMASK_SLEEP_ENABLE) {
        AcpiSuspend((value >> 10) & 7);
      }
      break;
    case 0x20 ... 0x27:
      value = acpi->gpe0_status();
      memcpy((uint8_t*)&value + offset - 0x20, data, size);
      acpi->set_gpe0_status(value);
      break;
    case 0x28 ... 0x2F:
      value = acpi->gpe0_enable();
      memcpy((uint8_t*)&value + offset - 0x28, data, size);
      acpi->set_gpe0_enable(value);
      break;
    default:
      MV_PANIC("not supported writing at ACPI offset=0x%lx value=0x%lx", offset, value);
      break;
    }
  }

  void AcpiSuspend(uint8_t type) {
    switch (type)
    {
    case 0: // soft power off
      std::thread([this]() {
        manager_->machine()->Pause();
        MV_LOG("machine is power off");
      }).detach();
      break;
    case 1: // suspend request
      MV_PANIC("suspend is not supported");
      break;
    default:
      MV_LOG("unknown acpi suspend type=%d", type);
      break;
    }
  }

  /* Power down interface */
  virtual void PowerDown() {
    manager_->io()->Schedule([this]() {
      auto acpi = state_.mutable_acpi();
      if (acpi->pm1_enable() & 0x100) {
        acpi->set_pm1_status(acpi->pm1_status() | 0x100);
        UpdateSystemControlIrq();
      }
    });
  }
};

DECLARE_DEVICE(Ich9Lpc);
