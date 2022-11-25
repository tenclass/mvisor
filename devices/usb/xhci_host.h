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

#ifndef _MVISOR_DEVICES_USB_XHCI_HOST_H
#define _MVISOR_DEVICES_USB_XHCI_HOST_H

#include <array>
#include <vector>
#include <set>

#include "pci_device.h"
#include "usb_device.h"
#include "xhci_internal.h"

struct XhciPortState {
  uint          id;
  uint          speed_mask;
  UsbDevice*    device;
};

struct XhciRing {
  bool          consumer_cycle_bit;
  uint64_t      dequeue;
};

struct XhciTransfer;
struct XhciEndpoint {
  uint                    id;
  uint                    slot_id;
  EPType                  type;
  uint                    max_packet_size;
  uint                    max_pstreams;
  bool                    linear_stream_array;
  uint                    interval;
  XhciRing                ring;
  int64_t                 mfindex_last;
  uint                    state;
  uint64_t                context_address;
  std::set<XhciTransfer*> transfers;
  uint                    endpoint_address;
};

struct XhciSlot {
  bool          enabled;
  bool          addressed;
  uint64_t      context_address;
  XhciEndpoint* endpoints[31];
  UsbDevice*    device;
  int           interrupt_vector;
};

struct XhciEvent {
  TRBType       type;
  TRBCCode      completion_code = CC_INVALID;
  uint64_t      poniter = 0;
  uint32_t      length = 0;
  uint32_t      flags = 0;
  uint          slot_id = 0;
  uint          endpoint_id = 0;
};

struct XhciTransfer {
  XhciEndpoint* endpoint;
  TRBCCode      status;
  uint          stream_id;
  bool          completed;
  bool          in_direction;
  bool          interrupt_on_completion;
  std::vector<XhciTransferRequestBlock> trbs;
  UsbPacket*    packet;
  UsbDevice*    device;
};

class XhciHost : public UsbHost {
 private:
    /* 8 ports in total, USB 2.0 ports 1-4, USB 3.0 ports 5-8 */
    uint max_ports_ = 8;
    uint max_interrupts_ = 16;
    uint max_slots_ = 64;
    uint max_pstreams_mask_ = 7;
  
    std::array<XhciPortState, 128>          port_states_;
    std::array<XhciPortRegisters, 128>      port_regs_;
    std::array<XhciInterruptRegisters, 128> interrupt_regs_;
    std::array<XhciSlot, 128>               slots_;
    XhciRing                                command_ring_;
    IoTimePoint                             microframe_index_start_;
    XhciCapabilityRegisters                 capability_regs_;
    XhciOperationalRegisters                operational_regs_;
    XhciRuntimeRegisters                    runtime_regs_;

 private:
  void Run();
  void Halt();
  bool IsRunning();
  bool AttachUsbDevice(UsbDevice* device);
  void PushEvent(uint vector, XhciEvent &event);
  void WriteEvent(uint vector, XhciEvent &event);
  void SetupRing(XhciRing &ring, uint64_t base);
  bool PopRing(XhciRing &ring, XhciTransferRequestBlock &trb);
  void ResetEventRing(int index);
  void RaiseInterrupt(uint vector);
  void CheckInterrupt(uint vector);

  void WriteRuntimeRegs(uint64_t offset, uint8_t* data, uint32_t size);
  void ReadOperationalRegs(uint64_t offset, uint8_t* data, uint32_t size);
  void ReadPortRegs(uint64_t offset, uint8_t* data, uint32_t size);
  int64_t GetMicroFrameIndex();
  void ReadRuntimeRegs(uint64_t offset, uint8_t* data, uint32_t size);
  void WriteOperationalUsbCommand(uint32_t command);
  void WriteOperationalRegs(uint64_t offset, uint8_t* data, uint32_t size);

  /* ======================== Slot Functions ======================= */
  TRBCCode EnableSlot(uint slot_id);
  TRBCCode DisableSlot(uint slot_id);
  TRBCCode ResetSlot(uint slot_id);
  bool GetSlot(XhciEvent &event, XhciTransferRequestBlock &trb, uint &slot_id);
  TRBCCode EvaluateSlot(uint slot_id, uint64_t input_addr);
  TRBCCode ConfigureSlot(uint slot_id, uint64_t input_addr, bool deconfigure);
  void PostLoadSlot(uint slot_id);
  TRBCCode AddressSlot(uint slot_id, uint64_t input_addr, bool block_set_request);

  /* ======================== Port Functions ======================= */
  void SetupPort(int index);
  void ResetPort(int index, bool warm_reset);
  void NotifyPort(uint index, uint32_t bits);
  void WritePortStatusControl(uint64_t index, uint32_t value);
  void WritePortRegs(uint64_t offset, uint8_t* data, uint32_t size);

  UsbDevice* LookupDevice(uint port_id, uint32_t route_string);

  /* ======================== Endpoint Functions ======================= */
  XhciEndpoint* CreateEndpoint(uint slot_id, uint endpoint_id, uint32_t* context);
  void EnableEndpoint(uint slot_id, uint endpoint_id, uint32_t* context, bool autorun);
  TRBCCode DisableEndpoint(uint slot_id, uint endpoint_id);
  TRBCCode StopEndpoint(uint slot_id, uint endpoint_id);
  TRBCCode ResetEndpoint(uint slot_id, uint endpoint_id);
  TRBCCode SetEndpointDequee(uint slot_id, uint endpoint_id, uint stream_id, uint64_t dequeue);
  void StallEndpoint(XhciTransfer* transfer);
  void SetEndpointState(XhciEndpoint* endpoint, uint32_t state);
  void NotifyEndpoint(UsbDevice* device, uint endpoint_address);

  /* ======================== Transfer Functions ======================= */
  XhciTransfer* CreateTransfer(XhciEndpoint* endpoint, uint stream_id, int length);
  void TerminateTransfer(XhciTransfer* transfer, TRBCCode report);
  void TerminateAllTransfers(XhciEndpoint* endpoint, TRBCCode report);
  void FreeTransfer(XhciTransfer* transfer);
  void ReportTransfer(XhciTransfer* transfer);
  void CompleteTransfer(XhciTransfer* transfer);
  void SetupTransfer(XhciTransfer* transfer);
  void HandleTransfer(XhciTransfer* transfer);
  int GetRingChainLength(XhciRing &ring);
  void KickEndpoint(uint slot_id, uint endpoint_id, uint stream_id);
  void ProcessCommands();
  void WriteDoorbellRegs(uint64_t address, uint64_t offset, uint8_t* data, uint32_t size);

 public:
  XhciHost();

  virtual void Reset();
  virtual void Connect();
  virtual void Disconnect();
  virtual bool SaveState(MigrationWriter* writer);
  virtual bool LoadState(MigrationReader* reader);
  virtual void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size);
};

#endif // _MVISOR_DEVICES_USB_XHCI_HOST_H
