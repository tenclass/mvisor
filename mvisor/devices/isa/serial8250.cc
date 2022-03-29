/* 
 * MVisor Serial Console
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

#include "device.h"

#include <string>
#include <linux/serial_reg.h>

#include "device_manager.h"
#include "logger.h"

/*
 * This fakes a U6_16550A. The fifo len needs to be 64 as the kernel
 * expects that for autodetection.
 */
#define FIFO_LEN            64
#define FIFO_MASK	          (FIFO_LEN - 1)
#define UART_IIR_TYPE_BITS	0xC0

struct Console {
  uint          index;
  std::string   name;
  uint64_t      ioport_base;

  uint          irq_line;
  uint8_t       irq_status;
  int           write_count;
  int           read_count;
  int           read_done;

  char          read_buffer[FIFO_LEN];
  char          write_buffer[FIFO_LEN];

  uint8_t       interrupt_id;
  uint8_t       interrupt_enable;
  uint8_t       fifo_control;
  uint8_t       line_control;
  uint8_t       line_status;
  uint8_t       modem_status;
  uint8_t       modem_control;
  uint8_t       scratch;
  uint16_t      divsor_latch;
};

class Serial8250 : public Device {
 private:
  std::array<Console, 4>  consoles_;

  void AddConsole(uint index, const char* name, uint64_t ioport_base, uint irq_line) {
    MV_ASSERT(index < consoles_.size());
    auto& console = consoles_[index];
    console.index = index;
    console.name = name;
    console.ioport_base = ioport_base;
    console.irq_line = irq_line;
    console.interrupt_id = UART_IIR_NO_INT;
    console.line_status = UART_LSR_TEMT | UART_LSR_THRE;
    console.modem_status = UART_MSR_DCD | UART_MSR_DSR | UART_MSR_CTS;
    console.modem_control = UART_MCR_OUT2;
    AddIoResource(kIoResourceTypePio, ioport_base, 8, name);
  }

  Console* GetConsoleByIoPortBase(uint64_t ioport_base) {
    for (auto& console : consoles_) {
      if (console.ioport_base == ioport_base)
        return &console;
    }
    return nullptr;
  }

  void UpdateIrqLevel(Console* console) {
    uint8_t status = 0;

    /* Handle clear rx */
    if (console->line_control & UART_FCR_CLEAR_RCVR) {
      console->line_control &= ~UART_FCR_CLEAR_RCVR;
      console->read_count = console->read_done = 0;
      console->line_status &= ~UART_LSR_DR;
    }

    /* Handle clear tx */
    if (console->line_control & UART_FCR_CLEAR_XMIT) {
      console->line_control &= ~UART_FCR_CLEAR_XMIT;
      console->write_count = 0;
      console->line_status |= UART_LSR_TEMT | UART_LSR_THRE;
    }

    /* Data ready and rcv interrupt enabled ? */
    if ((console->interrupt_enable & UART_IER_RDI) && (console->line_status & UART_LSR_DR))
      status |= UART_IIR_RDI;

    /* Transmitter empty and interrupt enabled ? */
    if ((console->interrupt_enable & UART_IER_THRI) && (console->line_status & UART_LSR_TEMT))
      status |= UART_IIR_THRI;

    /* Now update the irq line, if necessary */
    if (!status) {
      console->interrupt_id = UART_IIR_NO_INT;
      if (console->irq_status)
        manager_->SetGsiLevel(console->irq_line, 0);
    } else {
      console->interrupt_id = status;
      if (!console->irq_status)
        manager_->SetGsiLevel(console->irq_line, 1);
    }
    console->irq_status = status;

    /*
    * If the kernel disabled the tx interrupt, we know that there
    * is nothing more to transmit, so we can reset our tx logic
    * here.
    */
    if (!(console->interrupt_enable & UART_IER_THRI))
      FlushConsole(console);
  }

  void FlushConsole(Console* console) {
    console->line_status |= UART_LSR_TEMT | UART_LSR_THRE;
    if (console->write_count) {
      for (int i = 0; i < console->write_count; i++) {
        putchar(console->write_buffer[i]);
      }
      console->write_count = 0;
    }
  }

  uint8_t ReadConsole(Console* console) {
    if (console->read_done == console->read_count)
      return 0;
    
    if (console->line_status & UART_LSR_BI) {
      console->line_status &= ~UART_LSR_BI;
      return 0;
    }

    auto data = console->read_buffer[console->read_done++];
    if (console->read_count == console->read_done) {
      console->line_status &= ~UART_LSR_DR;
      console->read_count = console->read_done = 0;
    }
    return data;
  }

 public:
  Serial8250() {
    set_parent_name("ich9-lpc");

    bzero(&consoles_, sizeof(consoles_));
    AddConsole(0, "COM 1", 0x3F8, 4);
    AddConsole(1, "COM 2", 0x2F8, 3);
    AddConsole(2, "COM 3", 0x3E8, 4);
    AddConsole(3, "COM 4", 0x2E8, 3);
  }

  virtual void Reset() {
    Device::Reset();
  }

  void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    auto console = GetConsoleByIoPortBase(resource->base);
    MV_ASSERT(console);
    switch (offset)
    {
    case UART_RX:
      if (console->line_control & UART_LCR_DLAB)
        data[0] = console->divsor_latch & 0xFF;
      else
        data[0] = ReadConsole(console);
      break;
    case UART_IER:
      if (console->line_control & UART_LCR_DLAB)
        data[0] = console->divsor_latch >> 8;
      else
        data[0] = console->interrupt_enable;
      break;
    case UART_IIR:
      data[0] = console->interrupt_id | UART_IIR_TYPE_BITS;
      break;
    case UART_LCR:
      data[0] = console->line_control;
      break;
    case UART_MCR:
      data[0] = console->modem_control;
      break;
    case UART_LSR:
      data[0] = console->line_status;
      break;
    case UART_MSR:
      data[0] = console->modem_status;
      break;
    case UART_SCR:
      data[0] = console->scratch;
    default:
      Device::Read(resource, offset, data, size);
    }
    UpdateIrqLevel(console);
  }

  void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    auto console = GetConsoleByIoPortBase(resource->base);
    MV_ASSERT(console);

    switch (offset)
    {
    case UART_TX:
      if (console->line_control & UART_LCR_DLAB) {
        console->divsor_latch = (console->divsor_latch & 0xFF00) | data[0];
        break;
      }

      /* Loopback mode */
      if (console->modem_control & UART_MCR_LOOP) {
        if (console->read_count < FIFO_LEN) {
          console->read_buffer[console->read_count++] = data[0];
          console->line_status |= UART_LSR_DR;
        }
        break;
      }

      if (console->write_count < FIFO_LEN) {
        console->write_buffer[console->write_count++] = data[0];
        console->line_status &= ~UART_LSR_TEMT;
        if (console->write_count == FIFO_LEN / 2)
          console->line_status &= ~UART_LSR_THRE;
        /* Flush every write currently */
        FlushConsole(console);
      } else {
        console->line_status &= ~(UART_LSR_TEMT | UART_LSR_THRE);
        MV_PANIC("should never got here");
      }
      break;
    case UART_IER:
      if (console->line_control & UART_LCR_DLAB)
        console->divsor_latch = (console->divsor_latch & 0xFF) | (data[0] << 8);
      else
        console->interrupt_enable = data[0] & 0xF;
      break;
    case UART_FCR:
      console->fifo_control = data[0];
      break;
    case UART_LCR:
      console->line_control = data[0];
      break;
    case UART_MCR:
      console->modem_control = data[0];
      break;
    case UART_SCR:
      console->scratch = data[0];
      break;
    case UART_LSR:
    case UART_MSR:
    default:
      Device::Write(resource, offset, data, size);
    }
    UpdateIrqLevel(console);
  }
};

DECLARE_DEVICE(Serial8250);
