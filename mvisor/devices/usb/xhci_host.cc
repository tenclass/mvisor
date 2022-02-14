/* 
 * MVisor USB 3.0 XHCI
 * SPEC: <https://www.intel.com/content/dam/www/public/us/en/documents/ \
 *    technical-specifications/extensible-host-controler-interface-usb-xhci.pdf>
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

#include <cstring>
#include <vector>
#include "pci_device.h"
#include "device_manager.h"
#include "xhci_internal.h"

class XhciHost : public PciDevice {
 private:
    /* 8 ports in total, USB 2.0 ports 1-4, USB 3.0 ports 5-8 */
    int max_ports_ = 8;
    int max_interrupts_ = 16;
    int max_slots_ = 64;
    IoTimePoint microframe_index_start_;
    XhciCapabilityRegisters capability_regs_;
    XhciOperationalRegisters operational_regs_;
    XhciRuntimeRegisters runtime_regs_;
    std::vector<XhciPortRegisters> port_regs_;
    std::vector<XhciInterruptRegisters> interrupt_regs_;

 public:
  XhciHost() {
    devfn_ = PCI_MAKE_DEVFN(3, 0);
    
    pci_header_.vendor_id = 0x1B36;
    pci_header_.device_id = 0x000D;
    pci_header_.class_code = 0x0C0330;
    pci_header_.revision_id = 1;
    pci_header_.header_type = PCI_HEADER_TYPE_NORMAL;
    pci_header_.subsys_vendor_id = 0x1AF4;
    pci_header_.subsys_id = 0x1100;
    pci_header_.irq_pin = 0;
    pci_header_.cacheline_size = 0x10;
    pci_header_.command = PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;

    /* Specification Release 3.0 */
    pci_header_.data[0x60] = 0x30;
    next_capability_offset_ = 0x90;

    AddPciBar(0, 0x4000, kIoResourceTypeMmio);
    AddMsiXCapability(0, 16, 0x3000, 0x1000);
  }

  void Connect() {
    PciDevice::Connect();
    
    for (int i = 0; i < max_ports_; i++) {
      port_regs_.push_back(XhciPortRegisters {
        .status_control = 0
      });
    }
    for (int i = 0; i < max_interrupts_; i++) {
      interrupt_regs_.push_back(XhciInterruptRegisters {
        .management = 0
      });
    }
  }

  void Disconnect() {
    port_regs_.clear();
    interrupt_regs_.clear();

    PciDevice::Disconnect();
  }

  void Reset() {
    PciDevice::Reset();

    MV_ASSERT(sizeof(capability_regs_) == 0x40);
    bzero(&capability_regs_, sizeof(capability_regs_));
    bzero(&operational_regs_, sizeof(operational_regs_));
    bzero(&runtime_regs_, sizeof(runtime_regs_));

    capability_regs_.capability_length = sizeof(capability_regs_);
    capability_regs_.interface_version = 0x0100;
    capability_regs_.max_slots = max_slots_;
    capability_regs_.max_interrupts = max_interrupts_;
    capability_regs_.max_ports = max_ports_;
    capability_regs_.hcs_params2 = 0x0000000F;
    capability_regs_.hcs_params3 = 0x00000000;
    capability_regs_.capability_params1 = 0x00080001 | (7 << 12); // 64bit / 256 primary streams
    capability_regs_.doorbell_offset = 0x2000;
    capability_regs_.runtime_registers_offset = 0x1000;

    /* Extended capabilites */
    uint32_t* ext = capability_regs_.extended_capabilities;
    /* Supported protocol USB 2.0 (port 1-4) */
    *ext++ = 0x02000402;
    *ext++ = 0x20425355;
    *ext++ = (4 << 8) | 1;
    *ext++ = 0;
    /* Supported protocol USB 3.0 (port 5-8) */
    *ext++ = 0x03000002;
    *ext++ = 0x20425355;
    *ext++ = (4 << 8) | 5;
    *ext++ = 0;

    /* Operational registers */
    operational_regs_.usb_status = USBSTS_HCH; // halted
    operational_regs_.page_size = 1; // 4KB

    for (int i = 1; i <= max_slots_; i++) {
      DisableSlot(i);
    }
    
    for (int i = 0; i < max_ports_; i++) {
      AttachPort(i);
    }

    for (auto &interrupt: interrupt_regs_) {
      bzero(&interrupt, sizeof(interrupt));
    }
  }

  bool IsRunning() {
    return !(operational_regs_.usb_status & USBSTS_HCH);
  }

  void ResetPort(int index, bool warm_reset) {
    auto &port = port_regs_[index];
    if (true) { // port has no device
      return;
    }
    /* FIXME: Reset device */
    MV_ASSERT(!warm_reset);
    set_field(&port.status_control, PLS_U0, PORTSC_PLS);
    port.status_control |= PORTSC_PED;
    port.status_control &= ~PORTSC_PR;
    NotifyPort(index, PORTSC_PRC);
    MV_PANIC("reset port %d", index);
  }

  void AttachPort(int index) {
    auto &port = port_regs_[index];
    uint32_t port_link_state = PLS_RX_DETECT;
    port.status_control = PORTSC_PP;

    if (false) { // port has device
      /* FIXME: Set port speed */
      MV_PANIC("set port %d speed", index);
    }
    port.status_control |= port_link_state << PORTSC_PLS_SHIFT;
    NotifyPort(index, PORTSC_CSC);
  }

  void NotifyPort(int index, uint32_t bits) {
    auto &port = port_regs_[index];
    if ((port.status_control & bits) == bits) {
      return;
    }
    if (!IsRunning()) {
      return;
    }
    port.status_control |= bits;

    MV_PANIC("notify port %d", index);
  }

  void DisableSlot(int slot_id) {
    // MV_LOG("disable slot %d", slot_id);
  }

  void RaiseInterrupt(int vector) {
    if (debug_) {
      MV_LOG("Raise interrupt %d", vector);
    }
  }

  void CheckInterrupt(int vector) {
    if (debug_) {
      MV_LOG("Check interrupt %d", vector);
    }
  }

  void ReadOperationalRegs(uint64_t offset, uint8_t* data, uint32_t size) {
    memcpy(data, (uint8_t*)&operational_regs_ + offset, size);
  }

  void ReadPortRegs(uint64_t offset, uint8_t* data, uint32_t size) {
    uint64_t index = offset / sizeof(XhciPortRegisters);
    offset = offset % sizeof(XhciPortRegisters);
    auto& port = port_regs_[index];
    switch (offset)
    {
    case offsetof(XhciPortRegisters, status_control):
    case offsetof(XhciPortRegisters, link_info):
      memcpy(data, (uint8_t*)&port + offset, size);
      break;
    default:
      MV_PANIC("ReadPortRegs offset=0x%lx size=%u", offset, size);
      break;
    }
  }

  void ReadRuntimeRegs(uint64_t offset, uint8_t* data, uint32_t size) {
    if (offset < 0x20) {
      if (offset == offsetof(XhciRuntimeRegisters, microframe_index)) {
        auto delta_ns = (std::chrono::steady_clock::now() - microframe_index_start_).count();
        int64_t mfindex = delta_ns / 125000;
        memcpy(data, &mfindex, size);
      } else {
        MV_PANIC("ReadRuntimeRegs offset=0x%lx size=%u", offset, size);
      }
    } else {
      auto index = (offset - 0x20) / sizeof(XhciInterruptRegisters);
      offset = offset % sizeof(XhciInterruptRegisters);
      auto &interrupt = interrupt_regs_[index];
      memcpy(data, (uint8_t*)&interrupt + offset, size);
    }
  }

  void WriteOperationalUsbCommand(uint32_t command) {
    if ((command & USBCMD_RS) && !(operational_regs_.usb_command & USBCMD_RS)) {
      // RUN
      operational_regs_.usb_status &= ~USBSTS_HCH;
      microframe_index_start_ = std::chrono::steady_clock::now();
    } else if (!(command & USBCMD_RS) && (operational_regs_.usb_command & USBCMD_RS)) {
      // STOP
      operational_regs_.usb_status |= USBSTS_HCH;
      operational_regs_.command_ring_control &= ~CRCR_CRR;
    }
    if (command & USBCMD_CSS) { // Save state
      operational_regs_.usb_status &= ~USBSTS_SRE;
    }
    if (command & USBCMD_CRS) { // Restore state
      operational_regs_.usb_status |= USBSTS_SRE;
    }
    operational_regs_.usb_command = command & 0xC0F;
    // mfwrap_timer is not supported yet
    MV_ASSERT((command & (USBCMD_RS | USBCMD_EWE)) != (USBCMD_RS | USBCMD_EWE));
    if (command & USBCMD_HCRST) {
      Reset();
    }
    CheckInterrupt(0);
  }

  void WriteOperationalRegs(uint64_t offset, uint8_t* data, uint32_t size) {
    switch (offset)
    {
    case offsetof(XhciOperationalRegisters, usb_command):
      WriteOperationalUsbCommand(*(uint32_t*)data);
      break;
    case offsetof(XhciOperationalRegisters, configure):
      operational_regs_.configure = data[0];
      break;
    case offsetof(XhciOperationalRegisters, device_notification_control):
    case offsetof(XhciOperationalRegisters, context_base_array_pointer):
    case offsetof(XhciOperationalRegisters, context_base_array_pointer) + 4:
    case offsetof(XhciOperationalRegisters, command_ring_control):
    case offsetof(XhciOperationalRegisters, command_ring_control) + 4:
      memcpy((uint8_t*)&operational_regs_ + offset, data, size);
      break;
    default:
      MV_PANIC("WriteOperationalRegs offset=0x%lx size=%u data=0x%lx",
        offset, size, *(uint64_t*)data);
      break;
    }
  }

  void WritePortStatusControl(uint64_t index, uint32_t value) {
    auto& port = port_regs_[index];
    /* write-1-to-start bits */
    if (value & PORTSC_WPR) {
      ResetPort(index, true);
      return;
    }
    if (value & PORTSC_PR) {
      ResetPort(index, false);
      return;
    }
    uint32_t sc = port.status_control;
    uint32_t notify = 0;
    /* write-1-to-clear bits */
    sc &= ~(value & (PORTSC_CSC|PORTSC_PEC|PORTSC_WRC|PORTSC_OCC|PORTSC_PRC|PORTSC_PLC|PORTSC_CEC));
    
    if (value & PORTSC_LWS) { // overwrite PLS
      uint32_t old_pls = get_field(port.status_control, PORTSC_PLS);
      uint32_t new_pls = get_field(value, PORTSC_PLS);
      switch (new_pls)
      {
      case PLS_U0:
        if (old_pls != PLS_U0) {
          set_field(&sc, new_pls, PORTSC_PLS);
          notify = PORTSC_PLC;
        }
        break;
      case PLS_U3:
        if (old_pls < PLS_U3) {
          set_field(&sc, new_pls, PORTSC_PLS);
        }
        break;
      default:
        MV_PANIC("ignore PLS write old=0x%x new=0x%x", old_pls, new_pls);
        break;
      }
    }
    /* read/write bits */
    sc &= ~(PORTSC_PP|PORTSC_WCE|PORTSC_WDE|PORTSC_WOE);
    sc |= (value & (PORTSC_PP|PORTSC_WCE|PORTSC_WDE|PORTSC_WOE));
    port.status_control = sc;
    if (notify) {
      NotifyPort(index, notify);
    }
  }

  void WritePortRegs(uint64_t offset, uint8_t* data, uint32_t size) {
    uint64_t index = offset / sizeof(XhciPortRegisters);
    offset = offset % sizeof(XhciPortRegisters);
    switch (offset)
    {
    case offsetof(XhciPortRegisters, status_control):
      MV_ASSERT(size == 4);
      WritePortStatusControl(index, *(uint32_t*)data);
      break;
    default:
      MV_PANIC("WritePortRegs offset=0x%lx size=%u data=0x%lx", offset, size, *(uint64_t*)data);
      break;
    }
  }

  void ResetEventRing(int index) {
    auto &interrupt = interrupt_regs_[index];

    if (interrupt.event_ring_table_size == 0 || interrupt.event_ring_table_base == 0) {
      /* disabled */
      bzero(&interrupt.event_ring_segment, sizeof(interrupt.event_ring_segment));
      return;
    }
    MV_ASSERT(interrupt.event_ring_table_size == 1);

    interrupt.event_ring_segment = *(XhciEventRingSegment*)manager_->TranslateGuestMemory(
      interrupt.event_ring_table_base);
    interrupt.event_ring_enqueue_index = 0;
    auto &segment = interrupt.event_ring_segment;

    if (debug_) {
      MV_LOG("event ring[%d] start=0x%lX size=%d", index, segment.start, segment.size); 
    }
    MV_ASSERT(segment.size >= 16 && segment.size < 4096);
  }

  void WriteRuntimeRegs(uint64_t offset, uint8_t* data, uint32_t size) {
    MV_ASSERT(offset >= 0x20);
    uint64_t value = 0;
    memcpy(&value, data, size);
    int index = (offset - 0x20) / sizeof(XhciInterruptRegisters);
    auto &interrupt = interrupt_regs_[index];
    switch (offset & 0x1F)
    {
    case offsetof(XhciInterruptRegisters, management):
      if (value & IMAN_IP) {
        interrupt.management &= ~IMAN_IP;
      }
      interrupt.management &= ~IMAN_IE;
      interrupt.management |= value & IMAN_IE;
      CheckInterrupt(index);
      break;
    case offsetof(XhciInterruptRegisters, moderation):
      interrupt.moderation = value & 0xFFFFFFFF;
      break;
    case offsetof(XhciInterruptRegisters, event_ring_table_size):
      interrupt.event_ring_table_size = value & 0xFFFF;
      break;
    case offsetof(XhciInterruptRegisters, event_ring_table_base):
      interrupt.event_ring_table_base = value & 0xFFFFFFFFFFFFFFC0ULL;
      ResetEventRing(index);
      break;
    case offsetof(XhciInterruptRegisters, event_ring_dequeue_pointer):
      if (value & ERDP_EHB) {
        interrupt.event_ring_dequeue_pointer &= ~ERDP_EHB;
      }
      interrupt.event_ring_dequeue_pointer = (value & ~ERDP_EHB) |
        (interrupt.event_ring_dequeue_pointer & ERDP_EHB);
      if ((value & ERDP_EHB) && interrupt.event_ring_segment.size) {
        uint64_t dequeue_index = (interrupt.event_ring_dequeue_pointer -
          interrupt.event_ring_segment.start) / TRB_SIZE;
        if (dequeue_index != interrupt.event_ring_enqueue_index) {
          RaiseInterrupt(index);
        }
      }
      break;
    default:
      memcpy((uint8_t*)&interrupt + offset, data, size);
      break;
    }
  }

  void WriteDoorbellRegs(uint64_t offset, uint8_t* data, uint32_t size) {
    MV_PANIC("WriteDoorbellRegs offset=0x%lx size=%u data=0x%lx", offset, size, *(uint64_t*)data);
  }

  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
    uint64_t end = offset + size;
    if (ir.base == pci_bars_[0].address && end < 0x3000) {
      if (end <= capability_regs_.capability_length) {
        memcpy(data, (uint8_t*)&capability_regs_ + offset, size);
      } else if (end <= capability_regs_.capability_length + 0x400ULL) {
        ReadOperationalRegs(offset - capability_regs_.capability_length, data, size);
      } else if (end <= 0x1000) {
        ReadPortRegs(offset - capability_regs_.capability_length - 0x400, data, size);
      } else if (end <= 0x2000) {
        ReadRuntimeRegs(offset - 0x1000, data, size);
      } else if (end <= 0x3000) {
        bzero(data, size);
      }
      if (debug_) {
        MV_LOG("Read offset=0x%lx size=%u data=0x%lx", offset, size, *(uint64_t*)data);
      }
    } else {
      PciDevice::Read(ir, offset, data, size);
    }
  }

  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
    uint64_t end = offset + size;
    if (ir.base == pci_bars_[0].address && end <= 0x3000) {
      if (debug_) {
        MV_LOG("Write offset=0x%lx size=%u data=0x%lx", offset, size, *(uint64_t*)data);
      }
      if (end <= capability_regs_.capability_length) {
      } else if (end <= capability_regs_.capability_length + 0x400ULL) {
        WriteOperationalRegs(offset - capability_regs_.capability_length, data, size);
      } else if (end <= 0x1000) {
        WritePortRegs(offset - capability_regs_.capability_length - 0x400, data, size);
      } else if (end <= 0x2000) {
        WriteRuntimeRegs(offset - 0x1000, data, size);
      } else if (end <= 0x3000) {
        WriteDoorbellRegs(offset - 0x3000, data, size);
      }
    } else {
      PciDevice::Write(ir, offset, data, size);
    }
  }
};

DECLARE_DEVICE(XhciHost);
