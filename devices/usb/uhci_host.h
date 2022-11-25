/* 
 * MVisor USB 1.0 UHCI
 * https://wiki.osdev.org/Universal_Host_Controller_Interface
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

#ifndef _MVISOR_DEVICES_USB_UHCI_HOST_H
#define _MVISOR_DEVICES_USB_UHCI_HOST_H

#include <set>

#include "pci_device.h"
#include "usb_device.h"
#include "uhci_internal.h"

struct UhciTransfer {
  bool                    isochronous;
  bool                    completed;
  bool                    stop;
  bool                    interrupt_on_completion;
  uint                    link;
  UsbPacket*              packet;
  UsbDevice*              device;
  uint                    endpoint_address;
  std::vector<UhciTransferDescriptor*> tds;
};

struct UhciQueue {
  UhciQueueHead*          qh;
  UsbDevice*              device;
  uint                    endpoint_address;
  UhciTransfer*           transfer;
  int                     valid;
  bool                    nak;
};

struct UhciPortSate {
  uint                    id;
  uint                    speed_mask;
  UsbDevice*              device;
};

class UhciHost : public UsbHost {
 public:
  UhciHost();

  virtual void Reset();
  virtual void Connect();
  virtual void Disconnect();
  virtual bool SaveState(MigrationWriter* writer);
  virtual bool LoadState(MigrationReader* reader);
  virtual void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);

 private:
  void Run();
  void Halt();
  bool AttachUsbDevice(UsbDevice* device);
  void UpdateIrqLevel();
  void ResetPort(uint index);

  void SetError(UhciTransferDescriptor* td, int status);
  void NotifyEndpoint(UsbDevice* device, uint endpoint_address);

  UsbDevice* FindDevice(UhciTransferDescriptor* td);
  UhciTransfer* CreateTransfer(UhciTransferDescriptor* td, uint32_t link);
  void CompleteTransfer(UhciTransfer* transfer, uint32_t* next_link);
  void FreeTransfer(UhciTransfer* transfer);

  void OnFrameTimer();
  void WriteUsbCommand(uint16_t value);
  void WritePortStatusControl(uint index, uint16_t value);

  UhciRegisters                       uhci_;
  IoTimer*                            frame_timer_ = nullptr;
  uint32_t*                           frame_list_ = nullptr;

  uint                                max_ports_ = 8;
  std::array<UhciPortSate, 128>       port_states_;
  std::map<uint32_t, UhciQueue*>      queues_;
};

#endif // _MVISOR_DEVICES_USB_UHCI_HOST_H
