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

#ifndef _MVISOR_DEVICES_VIRTIO_PCI_H
#define _MVISOR_DEVICES_VIRTIO_PCI_H

#include "pci_device.h"

#include <linux/virtio_pci.h>
#include <linux/uio.h>
#include <vector>

/* We support indirect buffer descriptors */
#define VIRTIO_RING_F_INDIRECT_DESC	28

/* The Guest publishes the used index for which it expects an interrupt
 * at the end of the avail ring. Host should ignore the avail->flags field. */
/* The Host publishes the avail index for which it expects a kick
 * at the end of the used ring. Guest should ignore the used->flags field. */
#define VIRTIO_RING_F_EVENT_IDX		  29


/* Virtio ring descriptors: 16 bytes.  These can chain together via "next". */
struct VRingDescriptor {
	/* Address (guest-physical). */
	uint64_t address;
	/* Length. */
	uint32_t length;

/* This marks a buffer as continuing via the next field. */
#define VRING_DESC_F_NEXT	1
/* This marks a buffer as write-only (otherwise read-only). */
#define VRING_DESC_F_WRITE	2
/* This means the buffer contains a list of buffer descriptors. */
#define VRING_DESC_F_INDIRECT	4
	/* The flags as indicated above. */
	uint16_t flags;
	/* We chain unused descriptors via this, too */
	uint16_t next;
} __attribute__((packed));

struct VRingAvailable {
/* The Guest uses this in avail->flags to advise the Host: don't interrupt me
 * when you consume a buffer.  It's unreliable, so it's simply an
 * optimization.  */
#define VRING_AVAIL_F_NO_INTERRUPT	1
	uint16_t flags;
	uint16_t index;
	uint16_t items[];
} __attribute__((packed));

/* u32 is used here for ids for padding reasons. */
struct VRingUsedElement {
	/* Index of start of used descriptor chain. */
	uint32_t id;
	/* Total length of the descriptor chain which was used (written to) */
	uint32_t length;
} __attribute__((packed));

struct VRingUsed {
/* The Host uses this in used->flags to advise the Guest: don't kick me when
 * you add a buffer.  It's unreliable, so it's simply an optimization.  Guest
 * will still kick if it's out of buffers. */
#define VRING_USED_F_NO_NOTIFY	1
	uint16_t flags;
	uint16_t index;
	struct VRingUsedElement items[];
} __attribute__((packed));


typedef std::function<void (void)> VoidCallback;
struct VirtQueue {
  bool              enabled;
  int               msix_vector;
  VoidCallback      notification_callback;
  
  int               index;
  int               size;
  VRingDescriptor*  descriptor_table;
  VRingAvailable*   available_ring;
  VRingUsed*        used_ring;
  int               last_available_index;
};

struct VirtElement {
  int                       id;
  uint32_t                  length;
  std::vector<struct iovec> vector;

 private:
  /* disallow const copy */
  const VirtElement& operator=(const VirtElement&);
};

class VirtioPci : public PciDevice {
 public:
  VirtioPci();
  virtual ~VirtioPci();
  virtual void Reset();

 protected: 
  void PrintQueue(VirtQueue& vq);
  bool PopQueue(VirtQueue& vq, VirtElement& element);
  void PushQueue(VirtQueue& vq, const VirtElement& element);
  void NotifyQueue(VirtQueue& vq);
  void AddQueue(uint16_t queue_size, VoidCallback callback);
  void EnableQueue(uint16_t queue_index, uint64_t desc_gpa, uint64_t avail_gpa, uint64_t used_gpa);
  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  virtual void ReadDeviceConfig(uint64_t offset, uint8_t* data, uint32_t size);
  void ReadCommonConfig(uint64_t offset, uint8_t* data, uint32_t size);
  void WriteCommonConfig(uint64_t offset, uint8_t* data, uint32_t size);
  void WriteNotification(uint64_t offset, uint8_t* data, uint32_t size);

  virtio_pci_common_cfg       common_config_;
  uint64_t                    device_features_;
  uint32_t                    driver_features_[2];
  std::array<VirtQueue, 64>   queues_;
  uint8_t                     isr_status_;
};

#endif // _MVISOR_DEVICES_VIRTIO_PCI_H
