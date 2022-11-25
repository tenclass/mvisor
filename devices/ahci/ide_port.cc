/* 
 * MVisor - Ide Port
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

#include "ide_port.h"

#include <sys/ioctl.h>
#include <cstring>

#include "logger.h"
#include "ide_host.h"
#include "ata_storage.h"
#include "ata_internal.h"
#include "device_manager.h"


IdePort::IdePort(DeviceManager* manager, IdeHost* host, int index)
  : manager_(manager), host_(host), port_index_(index)
{
  drives_[0] = nullptr;
  drives_[1] = nullptr;
}

IdePort::~IdePort() {
}

void IdePort::Reset() {
  for (uint i = 0; i < 2; i++) {
    bzero(&task_files_[i], sizeof(task_files_[i]));
  
    if (drives_[i]) {
      drives_[i]->Reset();
      drives_[i]->SetSignature(&task_files_[i]);
    }
  }

  active_tf_index_ = 0;
  bus_master_.command = 0;
  bus_master_.prdt_address = 0;
  bus_master_.status = 0;
}

void IdePort::AttachDevice(AtaStorageDevice* device, bool slave) {
  if (host_->debug()) {
    MV_LOG("attach %s to [%d,%d]", device->name(), port_index_, slave);
  }
  device->set_port(this);
  if (slave) {
    drives_[1] = device;
  } else {
    drives_[0] = device;
  }
}

void IdePort::SetIrq(uint level) {
  if (control_ & ATA_CB_DC_NIEN) {
    return;
  }

  if (level) {
    bus_master_.status |= BM_STATUS_INT;
  }
  host_->SetPortIrq(port_index_, level);
}

void IdePort::Write(uint64_t offset, uint8_t* data, uint32_t size) {
  auto tf = &task_files_[active_tf_index_];

  switch (offset)
  {
  case 0:
    MV_ASSERT(tf->status & ATA_CB_STAT_DRQ);
    WriteIoBuffer(data, size);
    break;
  case 1:
    task_files_[0].feature1 = task_files_[0].feature0;
    task_files_[1].feature1 = task_files_[1].feature0;
    task_files_[0].feature0 = data[0];
    task_files_[1].feature0 = data[0];
    break;
  case 2:
    task_files_[0].count1 = task_files_[0].count0;
    task_files_[1].count1 = task_files_[1].count0;
    task_files_[0].count0 = data[0];
    task_files_[1].count0 = data[0];
    break;
  case 3:
    task_files_[0].lba3 = task_files_[0].lba0;
    task_files_[1].lba3 = task_files_[1].lba0;
    task_files_[0].lba0 = data[0];
    task_files_[1].lba0 = data[0];
    break;
  case 4:
    task_files_[0].lba4 = task_files_[0].lba1;
    task_files_[1].lba4 = task_files_[1].lba1;
    task_files_[0].lba1 = data[0];
    task_files_[1].lba1 = data[0];
    break;
  case 5:
    task_files_[0].lba5 = task_files_[0].lba2;
    task_files_[1].lba5 = task_files_[1].lba2;
    task_files_[0].lba2 = data[0];
    task_files_[1].lba2 = data[0];
    break;
  case 6: // device select
    active_tf_index_ = (data[0] >> 4) & 1;
    task_files_[0].device = data[0];
    task_files_[1].device = data[0];
    break;
  case 7: // command
    tf->command = data[0];
    if (drives_[active_tf_index_]) {
      ExecuteCommand();
    }
    break;
  default:
    MV_PANIC("offset=%lx data=%x size=%x", offset, *data, size);
    break;
  }
}

void IdePort::Read(uint64_t offset, uint8_t* data, uint32_t size) {
  auto tf = &task_files_[active_tf_index_];

  /* If no drives, disable this port */
  if (!drives_[0] && !drives_[1]) {
    data[0] = 0;
    return;
  }

  switch (offset)
  {
  case 0:
    MV_ASSERT(tf->status & ATA_CB_STAT_DRQ);
    ReadIoBuffer(data, size);
    break;
  case 1:
    data[0] = tf->error;
    break;
  case 2:
    data[0] = tf->count0;
    break;
  case 3:
    data[0] = tf->lba0;
    break;
  case 4:
    data[0] = tf->lba1;
    break;
  case 5:
    data[0] = tf->lba2;
    break;
  case 6: // device select
    data[0] = tf->device;
    break;
  case 7: // status
    SetIrq(0);
    data[0] = tf->status;
    break;
  default:
    MV_PANIC("offset=%lx data=%x size=%x", offset, *data, size);
    break;
  }
}

void IdePort::ReadControl(uint64_t offset, uint8_t* data, uint32_t size) {
  MV_ASSERT(size == 1);
  auto tf = &task_files_[active_tf_index_];

  /* If no drives, disable this port */
  if (!drives_[0] && !drives_[1]) {
    data[0] = 0;
    return;
  }

  if (offset == ATA_CB_ASTAT) {
    data[0] = tf->status;
  }
}

void IdePort::WriteControl(uint64_t offset, uint8_t* data, uint32_t size) {
  MV_ASSERT(size == 1);

  if (offset == ATA_CB_DC) {
    control_ = data[0];
    if (control_ & ATA_CB_DC_SRST) {
      Reset();
    }
  }
}

void IdePort::ReadIoBuffer(uint8_t* data, uint32_t size) {
  auto io = drives_[active_tf_index_]->io();
  auto ptr = buffer_.data() + buffer_position_;
  switch (size)
  {
  case 1:
    *data = *ptr;
    break;
  case 2:
    *(uint16_t*)data = *(uint16_t*)ptr;
    break;
  case 4:
    *(uint32_t*)data = *(uint32_t*)ptr;
    break;
  }

  buffer_position_ += size;
  if (buffer_position_ >= io->transfer_bytes) {
    MV_ASSERT(buffer_position_ <= io->transfer_bytes);
    buffer_position_ = 0;
    drives_[active_tf_index_]->StopTransfer();
  }
}

void IdePort::WriteIoBuffer(uint8_t* data, uint32_t size) {
  auto io = drives_[active_tf_index_]->io();
  auto ptr = buffer_.data() + buffer_position_;
  switch (size)
  {
  case 1:
    *ptr = *data;
    break;
  case 2:
    *(uint16_t*)ptr = *(uint16_t*)data;
    break;
  case 4:
    *(uint32_t*)ptr = *(uint32_t*)data;
    break;
  }

  buffer_position_ += size;
  if (buffer_position_ >= io->transfer_bytes) {
    MV_ASSERT(buffer_position_ <= io->transfer_bytes);
    buffer_position_ = 0;
    drives_[active_tf_index_]->StopTransfer();
  }
}

void IdePort::ExecuteCommand() {
  auto io = drives_[active_tf_index_]->io();
  if (buffer_.size() != io->pio_buffer_size) {
    buffer_.resize(io->pio_buffer_size);
  }
  buffer_position_ = 0;

  io->buffer = (uint8_t*)buffer_.data();
  io->buffer_size = buffer_.size();
  io->vector.clear();
  io->vector.emplace_back(iovec {
    .iov_base = buffer_.data(),
    .iov_len = buffer_.size()
  });

  io->atapi_set = false;
  drives_[active_tf_index_]->StartCommand(&task_files_[active_tf_index_]);
}

void IdePort::OnDmaPrepare() {
  auto io = drives_[active_tf_index_]->io();
  if (bus_master_.status & BM_STATUS_RUNNING) {
    if (io->dma_enabled && io->dma_callback) {
      MV_ASSERT(!current_dma_vector_.empty());
      auto &front = current_dma_vector_.front();
      io->buffer = (uint8_t*)front.iov_base;
      io->buffer_size = front.iov_len;
      io->vector = current_dma_vector_;

      auto cb = std::move(io->dma_callback);
      cb();
    }
  }
}

void IdePort::OnDmaTransfer() {
  drives_[active_tf_index_]->StopTransfer();
}

void IdePort::OnPioTransfer() {
  /* FIXME: What if user changed active device while data is transferring??? */
  auto io = drives_[active_tf_index_]->io();

  /* ATAPI DMA transfer doesn't need this interrupt while transferring commands with PIO */
  if (io->type == kTransferAtapiCommand) {
    return;
  }

  SetIrq(1);
}

void IdePort::OnCommandDone() {
  auto io = drives_[active_tf_index_]->io();
  if (io->dma_enabled) {
    StopBusMasterDma();
  }

  SetIrq(1);
}

void IdePort::ReadBusMaster(uint64_t offset, uint8_t* data, uint32_t size) {
  switch (offset)
  {
  case 0: // command
    data[0] = bus_master_.command;
    break;
  case 2: // status
    data[0] = bus_master_.status;
    break;
  case 4: // prdt address
    *(uint32_t*)data = bus_master_.prdt_address;
    break;
  default:
    memset(data, 0xFF, size);
  }
}

void IdePort::WriteBusMaster(uint64_t offset, uint8_t* data, uint32_t size) {
  switch (offset)
  {
  case 0: // command
    if ((bus_master_.command ^ data[0]) & BM_CMD_START) {
      if (data[0] & BM_CMD_START) {
        StartBusMasterDma();
      } else {
        StopBusMasterDma();
      }
    }
    bus_master_.command = data[0] & 0x09;
    break;
  case 2: // status
    bus_master_.status = (data[0] & 0x60) | (bus_master_.status & 1) | (bus_master_.status & ~data[0] & 0x06);
    break;
  case 4: // prdt address
    bus_master_.prdt_address = *(uint32_t*)data;
    break;
  default:
    MV_PANIC("write offset=%lx data=%x size=%d", offset, *data, size);
    break;
  }
}

void IdePort::StartBusMasterDma() {
  current_dma_vector_.clear();

  auto manager = host_->manager();
  auto entry = (PhysicalRegionDescriptor*)manager->TranslateGuestMemory(bus_master_.prdt_address);
  while (true) {
    void* host = manager->TranslateGuestMemory(entry->physical_address);
    size_t length = entry->byte_count ? entry->byte_count : 65536;
    current_dma_vector_.emplace_back(iovec {
      .iov_base = host,
      .iov_len = length
    });
    if (entry->end_of_table) {
      break;
    }
    ++entry;
  }

  if (!(bus_master_.status & BM_STATUS_RUNNING)) {
    bus_master_.status |= BM_STATUS_RUNNING;
    OnDmaPrepare();
  }
}

void IdePort::StopBusMasterDma() {
  bus_master_.status &= ~BM_STATUS_RUNNING;
}

/* Called by IdeHost */
void IdePort::SaveState(IdeHostState_PortState* port_state) {
  auto bm = port_state->mutable_bus_master();
  bm->set_command(bus_master_.command);
  bm->set_status(bus_master_.status);
  bm->set_prdt_address(bus_master_.prdt_address);

  port_state->set_active_tf_index(active_tf_index_);
  port_state->set_control(control_);
  port_state->set_buffer_position(buffer_position_);
  port_state->set_buffer(buffer_);

  for (int i = 0; i < 2; i++) {
    auto tf = &task_files_[i];
    auto tfs = port_state->add_task_files();
    tfs->set_command(tf->command);
    tfs->set_control(tf->control);
    tfs->set_count0(tf->count0);
    tfs->set_count1(tf->count1);
    tfs->set_device(tf->device);
    tfs->set_error(tf->error);
    tfs->set_feature0(tf->feature0);
    tfs->set_feature1(tf->feature1);
    tfs->set_lba0(tf->lba0);
    tfs->set_lba1(tf->lba1);
    tfs->set_lba2(tf->lba2);
    tfs->set_lba3(tf->lba3);
    tfs->set_lba4(tf->lba4);
    tfs->set_lba5(tf->lba5);
    tfs->set_status(tf->status);
  }

}

void IdePort::LoadState(const IdeHostState_PortState* port_state) {
  auto &bm = port_state->bus_master();
  bus_master_.command = bm.command();
  bus_master_.status = bm.status();
  bus_master_.prdt_address = bm.prdt_address();

  active_tf_index_ = port_state->active_tf_index();
  control_ = port_state->control();
  buffer_position_ = port_state->buffer_position();
  buffer_ = port_state->buffer();

  for (int i = 0; i < 2; i++) {
    auto tf = &task_files_[i];
    auto &tfs = port_state->task_files(i);
    tf->command = tfs.command();
    tf->control = tfs.control();
    tf->count0 = tfs.count0();
    tf->count1 = tfs.count1();
    tf->device = tfs.device();
    tf->error = tfs.error();
    tf->feature0 = tfs.feature0();
    tf->feature1 = tfs.feature1();
    tf->lba0 = tfs.lba0();
    tf->lba1 = tfs.lba1();
    tf->lba2 = tfs.lba2();
    tf->lba3 = tfs.lba3();
    tf->lba4 = tfs.lba4();
    tf->lba5 = tfs.lba5();
    tf->status = tfs.status();
  }

  /* FIXME: should setup drive PIO, not finished now */
  for (int i = 0; i < 2; i++) {
    auto drive = drives_[i];
    if (drive) {
    }
  }
}
