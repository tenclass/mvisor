/* 
 * MVisor
 * Copyright (C) 2022 Terrence <terrence@tenclass.com>
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

#include "logger.h"
#include "pci_device.h"
#include "device_manager.h"


#define PIIX_PIRQA_ROUTE   0x60
#define PIIX_PIRQB_ROUTE   0x61
#define PIIX_PIRQC_ROUTE   0x62
#define PIIX_PIRQD_ROUTE   0x63


class Piix3 : public PciDevice {
 private:

  uint TranslatePciIrq(uint slot, uint function, uint pin) {
    /* According to ACPI configuration, slot 1 uses IRQ 9? */
    if (slot == 1 && function == 0) {
      return 9;
    }

    /* INTx[A-D] -> PIRQ[D-A] */
    uint8_t pirq = (slot + pin - 1 + 3) % 4;
    uint8_t gsi = pci_header_.data[PIIX_PIRQA_ROUTE + pirq] & ~0x80;
    return gsi;
  }

 public:
  Piix3() {
    slot_ = 1;
    function_ = 0;

    pci_header_.vendor_id = 0x8086;
    pci_header_.device_id = 0x7000;
    pci_header_.class_code = 0x060100;
    pci_header_.header_type = PCI_MULTI_FUNCTION | PCI_HEADER_TYPE_NORMAL;
    pci_header_.subsys_vendor_id = 0x1AF4;
    pci_header_.subsys_id = 0x1100;
  }

  void Connect() {
    PciDevice::Connect();

    manager_->set_pci_irq_translator([this](uint slot, uint function, uint pin) -> uint {
      return TranslatePciIrq(slot, function, pin);
    });
  }

  void Disconnect() {
    PciDevice::Disconnect();
    manager_->set_pci_irq_translator(nullptr);
  }

  void Reset() {
    PciDevice::Reset();
    
    for (int i = 0; i < 4; i++) {
      pci_header_.data[PIIX_PIRQA_ROUTE + i] = 0x80;
    }
  }
};

DECLARE_DEVICE(Piix3);
