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

#include "pmio.h"
#include "ich9_lpc.pb.h"
#include "device_manager.h"
#include "machine.h"
#include "logger.h"

#define ICH9_LPC_PMIO_BASE                      0x40
#define ICH9_LPC_PMIO_BASE_RTE                  0x1
#define ICH9_LPC_PMIO_BASE_DEFAULT              0x1

#define ICH9_LPC_ACPI_CTRL                      0x44

#define ICH9_LPC_PIRQA_ROUTE                    0x60
#define ICH9_LPC_PIRQB_ROUTE                    0x61
#define ICH9_LPC_PIRQC_ROUTE                    0x62
#define ICH9_LPC_PIRQD_ROUTE                    0x63

#define ICH9_LPC_PIRQE_ROUTE                    0x68
#define ICH9_LPC_PIRQF_ROUTE                    0x69
#define ICH9_LPC_PIRQG_ROUTE                    0x6a
#define ICH9_LPC_PIRQH_ROUTE                    0x6b

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


class Ich9Lpc : public Pmio {
 private:
  struct {
    uint8_t     control;
    uint8_t     status;
  } apm_;

  bool          initialized_pmio_ = false;
  bool          initialized_rcrb_ = false;

  void UpdatePmio() {
    /* PM IO base should be 0xB000 */
    pmio_base_ = *(uint32_t*)&pci_header_.data[ICH9_LPC_PMIO_BASE];
    pmio_base_ &= 0xFFC0;
    uint8_t acpi_ctrl = pci_header_.data[ICH9_LPC_ACPI_CTRL];

    if (acpi_ctrl & 0x80) {
      if (!initialized_pmio_) {
        AddIoResource(kIoResourceTypePio, pmio_base_, ICH9_PMIO_SIZE, "PMIO");
        initialized_pmio_ = true;

        /* update system control irq, 0 means IRQ 9 */
        auto irq_select = acpi_ctrl & 7;
        MV_ASSERT(irq_select == 0);
      }
    } else {
      if (initialized_pmio_) {
        RemoveIoResource(kIoResourceTypePio, "PMIO");
        pmio_base_ = 0;
        initialized_pmio_ = false;
      }
    }
  }

  void UpdateRootComplexRegisterBlock() {
    uint32_t rcrb = *(uint32_t*)(pci_header_.data + ICH9_LPC_RCBA);
    if (rcrb & ICH9_LPC_RCBA_EN) {
      if (!initialized_rcrb_) {
        AddIoResource(kIoResourceTypeMmio, rcrb & ICH9_LPC_RCBA_BA_MASK, ICH9_CC_SIZE, "RCRB");
        initialized_rcrb_ = true;
        manager_->set_pci_irq_translator([this](uint slot, uint function, uint pin) -> uint {
          MV_UNUSED(function);
          return TranslatePciIrq(slot, pin);
        });
      }
    } else {
      if (initialized_rcrb_) {
        RemoveIoResource(kIoResourceTypeMmio, "RCRB");
        initialized_rcrb_ = false;
        manager_->set_pci_irq_translator(nullptr);
      }
    }
  }

  /* According to the ACPI configuration (check acpi_dsdt used by FirmwareConfig),
   * DEV 0-24       INTx[A-D] -> PIRQ[E-F]
   * DEV 25-29, 31  INTx[A-D] -> PIRQ[A-D]
   * DEV 30         INTx[A-D] -> PIRQ[E-F]
   */
  uint TranslatePciIrq(uint slot, uint pin) {
    uint8_t intx = pin - 1;
    uint8_t pirq = (slot + intx) % 4 + 4;

    /* For ICH9 special devices, we should read the configuration from LPC RCBA */
    if (slot == 30) {
      pirq = (intx % 4) + 4;
    } else if (slot >= 25) {
      pirq = intx % 4;
    }

    /* PIRQ is starting from index 16 */
    return 16 + pirq;
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

  void Connect() {
    Pmio::Connect();

    /* Device present COM1 COM2 LPT Floppy */
    pci_header_.data[0x82] = (0 << 0) | (0 << 1) | (0 << 2) | (0 << 3);
  }

  void Reset() {
    Pmio::Reset();
    
    for (int i = 0; i < 4; i++) {
      pci_header_.data[ICH9_LPC_PIRQA_ROUTE + i] = 0x80;
      pci_header_.data[ICH9_LPC_PIRQE_ROUTE + i] = 0x80;
    }
    
    bzero(&apm_, sizeof(apm_));
    *(uint32_t*)(pci_header_.data + ICH9_LPC_PMIO_BASE) = ICH9_LPC_PMIO_BASE_DEFAULT;
    *(uint8_t*)(pci_header_.data + ICH9_LPC_ACPI_CTRL) = 0;
    UpdatePmio();

    *(uint32_t*)(pci_header_.data + ICH9_LPC_RCBA) = 0;
    UpdateRootComplexRegisterBlock();
  }

  bool SaveState(MigrationWriter* writer) {
    Ich9LpcState state;
    auto apm = state.mutable_apm();
    apm->set_control(apm_.control);
    apm->set_status(apm_.status);

    writer->WriteProtobuf("LPC", state);
    return Pmio::SaveState(writer);
  }

  bool LoadState(MigrationReader* reader) {
    if (!Pmio::LoadState(reader)) {
      return false;
    }

    Ich9LpcState state;
    if (!reader->ReadProtobuf("LPC", state)) {
      return false;
    }
    auto& apm = state.apm();
    apm_.control = apm.control();
    apm_.status = apm.status();

    UpdatePmio();
    UpdateRootComplexRegisterBlock();
    return true;
  }

  void WritePciConfigSpace(uint64_t offset, uint8_t* data, uint32_t length) {
    Pmio::WritePciConfigSpace(offset, data, length);

    if (ranges_overlap(offset, length, ICH9_LPC_PMIO_BASE, 4) ||
        ranges_overlap(offset, length, ICH9_LPC_ACPI_CTRL, 1)) {
      /* pm io base || lsacpi enable, SCI: IRQ9 000b = irq9*/
      UpdatePmio();
    }
    if (ranges_overlap(offset, length, ICH9_LPC_RCBA, 4)) {
      /* set root complex register block BAR */
      UpdateRootComplexRegisterBlock();
    }
    if (ranges_overlap(offset, length, ICH9_LPC_PIRQA_ROUTE, 4)) {
      /* activate irq remapping in LPC A-D */
      if (debug_) {
        MV_LOG("activate irq remapping in LPC A-D");
        MV_HEXDUMP("remapping", pci_header_.data + ICH9_LPC_PIRQA_ROUTE, 4);
      }
    }
    if (ranges_overlap(offset, length, ICH9_LPC_PIRQE_ROUTE, 4)) {
      /* activate irq remapping in LPC E-H */
      if (debug_) {
        MV_LOG("activate irq remapping in LPC E-H");
        MV_HEXDUMP("remapping", pci_header_.data + ICH9_LPC_PIRQA_ROUTE, 4);
      }
    }
    if (ranges_overlap(offset, length, ICH9_LPC_GEN_PMCON_1, 8)) {
      MV_PANIC("ich9_lpc_pmcon_update(lpc);");
    }
  }

  void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    if (resource->base == 0xB2) { // APM IO
      if (offset == 0) {
        data[0] = apm_.control;
      } else {
        data[0] = apm_.status;
      }
    } else if (resource->base == 0xAE00) { // ACPI PCI HOTPLUG
      if (offset == 0x0C) { // PCI removability status
        /* disable all PCIs hotplug */
        bzero(data, size);
      }
    } else {
      Pmio::Read(resource, offset, data, size);
    }
  }

  void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    if (resource->base == 0xB2) { // APM IO
      if (offset == 0) {
        apm_.control = data[0];
        if (apm_.control == 2) { // Enable ACPI
          pm1_.control |= PMIO_PM1_CTRL_SMI_EN;
        } else if (apm_.control == 3) { // Disable ACPI
          pm1_.control &= ~PMIO_PM1_CTRL_SMI_EN;
        } else {
          MV_PANIC("unknown apm control=0x%x", *data);
        }
      } else {
        apm_.status = data[0];
      }
    } else {
      Pmio::Write(resource, offset, data, size);
    }
  }
};

DECLARE_DEVICE(Ich9Lpc);
