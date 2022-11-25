/* 
 * MVisor Intel 82078
 * Copyright (C) 2022 Terrence <terrence@tenclass.com>
 * Reference: https://wiki.osdev.org/Floppy_Disk_Controller
 *            https://www.isdaman.com/alsos/hardware/fdc/floppy.htm
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

#include <deque>

#include "device.h"
#include "floppy.h"
#include "i8257_dma.h"
#include "i82078_fdc.pb.h"
#include "device_manager.h"
#include "device_interface.h"


#define REG_STATUS_A        0x3F0
#define REG_STATUS_B        0x3F1
#define REG_DIGITAL_OUTPUT  0x3F2
#define REG_TAPE_DRIVE      0x3F3
#define REG_MAIN_STATUS     0x3F4
#define REG_DATARATE_SELECT 0x3F4
#define REG_DATA_FIFO       0x3F5
#define REG_DIGITAL_INPUT   0x3F7
#define REG_CONFIG_CONTROL  0x3F7

#define DOR_RESET           (1<<2)
#define DOR_IRQ             (1<<3)
#define DOR_MOTOR0          (1<<4)

#define MSR_READY           (1<<7)
#define MSR_DIO             (1<<6)
#define MSR_NO_DMA          (1<<5)
#define MSR_COMMAND_BUSY    (1<<4)

#define SR0_HEAD            0x04
#define SR0_SEEK            0x20
#define SR0_READY_CHANGED   0xC0

#define SECTOR_SIZE         512


class I82078Fdc : public Device {
 private:
  uint8_t             digital_output_;
  uint8_t             main_status_;
  uint8_t             datarate_select_;
  uint8_t             error_status_[4];
  uint8_t             step_rate_;
  uint8_t             head_load_time_;
  uint8_t             lock_;
  uint8_t             reset_sense_remained_;
  uint8_t             config_;
  std::deque<uint8_t> fifo_;
  I8257Dma*           isa_dma_;
  Floppy*             drives_[2] = {0};
  uint8_t             drive_index_;

 public:
  I82078Fdc() {
    set_default_parent_class("Ich9Lpc", "Piix3");

    AddIoResource(kIoResourceTypePio, 0x3F0, 6, "Floppy");
    AddIoResource(kIoResourceTypePio, 0x3F7, 1, "Floppy Control");
  }

  void Connect() {
    Device::Connect();

    isa_dma_ = dynamic_cast<I8257Dma*>(manager_->LookupDeviceByClass("I8257Dma"));
    if (isa_dma_ == nullptr) {
      MV_PANIC("Floppy controller without ISA DMA is not implemented yet");
    }


    auto cmos = dynamic_cast<CmosDataInterface*>(manager_->LookupDeviceByClass("Cmos"));
    if (cmos) {
      uint8_t value = 0;
      for (size_t i = 0; i < children_.size() && i < 2; i++) {
        drives_[i] = dynamic_cast<Floppy*>(children_[i]);
        if (drives_[i]) {
          // Type 4 - 1.44MB, 3.5" - 2 heads, 80 tracks, 18 sectors
          value |= i == 0 ? 0x40 : 0x04;
        }
      }
      cmos->SetData(0x10, value);
    }
  }

  void Reset() {
    Device::Reset();

    main_status_ = MSR_READY;
    digital_output_ = 0;
    datarate_select_ = 0;
    drive_index_ = 0;
    step_rate_ = head_load_time_ = 0;
    lock_ = 0;
    config_ = 0;
    reset_sense_remained_ = 0;
    error_status_[0] = 0;
    error_status_[1] = 0;
    error_status_[2] = 0;
    error_status_[3] = 0;

    for (int i = 0; i < 2; i++) {
      if (drives_[i]) {
        drives_[i]->Reset();
      }
    }
    fifo_.clear();

    SetIrqLevel(0);
  }


  bool SaveState(MigrationWriter* writer) {
    I82078FdcState state;
    
    state.set_digital_output(digital_output_);
    state.set_main_status(main_status_);
    state.set_datarate_select(datarate_select_);
    state.set_error_status_0(error_status_[0]);
    state.set_config(config_);
    state.set_step_rate(step_rate_);
    state.set_head_load_time(head_load_time_);
    state.set_lock(lock_);
    state.set_drive_index(drive_index_);

    std::string fifo(fifo_.begin(), fifo_.end());
    state.set_fifo(fifo);

    for (auto i = 0; i < 2; i++) {
      if (drives_[i]) {
        auto pd = state.add_drives();
        pd->set_cylinder(drives_[i]->cylinder());
        pd->set_head(drives_[i]->head());
        pd->set_sector(drives_[i]->sector());
        pd->set_perpendicular(drives_[i]->perpendicular());
      }
    }

    writer->WriteProtobuf("FDC", state);
    return Device::SaveState(writer);
  }

  bool LoadState(MigrationReader* reader) {
    if (!Device::LoadState(reader)) {
      return false;
    }

    I82078FdcState state;
    if (!reader->ReadProtobuf("FDC", state)) {
      return false;
    }

    digital_output_ = state.digital_output();
    main_status_ = state.main_status();
    datarate_select_ = state.datarate_select();
    error_status_[0] = state.error_status_0();
    config_ = state.config();
    step_rate_ = state.step_rate();
    head_load_time_ = state.head_load_time();
    lock_ = state.lock();
    drive_index_ = state.drive_index();

    fifo_.clear();
    for (auto c : state.fifo()) {
      fifo_.push_back(c);
    }

    for (auto i = 0; i < state.drives_size() && i < 2; i++) {
      if (drives_[i]) {
        auto& d = state.drives(i);
        drives_[i]->set_perpendicular(d.perpendicular());
        drives_[i]->Seek(d.cylinder(), d.head(), d.sector());
      }
    }
    return true;
  }

  void SetIrqLevel(uint level) {
    manager_->SetGsiLevel(6, level);
    if (level == 0) {
      error_status_[0] = 0;
    }
  }

  void SenseDriveStatus() {
    auto drive_index = fifo_[1] & 3;
    auto drive = drives_[drive_index];
    auto head = (fifo_[1] >> 2) & 1;

    error_status_[3] = (1 << 5) | (1 << 3) | (head << 2) | drive_index;
    if (drive->image()->readonly()) {
      error_status_[3] |= 1 << 6;
    }
    if (drive->cylinder() == 0) {
      error_status_[3] |= 1 << 4;
    }

    fifo_.clear();
    fifo_.push_back(error_status_[3]);
    main_status_ |= MSR_DIO;
  }

  void SenseInterrupt() {
    auto drive = drives_[drive_index_];
    fifo_.clear();
    if (reset_sense_remained_ > 0) {
      fifo_.push_back(SR0_READY_CHANGED | (4 - reset_sense_remained_));
      reset_sense_remained_--;
    } else {
      fifo_.push_back(error_status_[0]);
    }
    fifo_.push_back(drive->cylinder());

    main_status_ |= MSR_DIO;
    SetIrqLevel(0);
    error_status_[0] = SR0_READY_CHANGED;
  }

  void Lock() {
    lock_ = (fifo_[0] & 0x80) ? 1 : 0;
    fifo_.clear();

    fifo_.push_back(lock_ << 4);

    main_status_ |= MSR_DIO;
  }

  void DumpRegisters() {
    auto drive = drives_[drive_index_];
    fifo_.clear();

    /* Drive positions */
    fifo_.push_back(drives_[0] ? drives_[0]->cylinder() : 0);
    fifo_.push_back(drives_[1] ? drives_[1]->cylinder() : 0);
    fifo_.push_back(0);
    fifo_.push_back(0);

    /* Timers */
    fifo_.push_back(step_rate_);
    fifo_.push_back(head_load_time_ << 1);
    fifo_.push_back(drive->sectors_per_cylinder());
    fifo_.push_back((lock_ << 7) | drive->perpendicular() << 2);
    fifo_.push_back(config_);
    fifo_.push_back(0);

    main_status_ |= MSR_DIO;
  }

  void Recalibrate() {
    drive_index_ = fifo_[1] & 3;
    auto drive = drives_[drive_index_];

    drive->Seek(0, 0, 1);
    fifo_.clear();

    error_status_[0] |= SR0_SEEK;
    main_status_ = MSR_READY;
    SetIrqLevel(1);
  }

  void Seek() {
    drive_index_ = fifo_[1] & 3;
    auto drive = drives_[drive_index_];

    drive->Seek(fifo_[2], (fifo_[1] >> 2) & 1, 1);
    fifo_.clear();

    error_status_[0] |= SR0_SEEK;
    main_status_ = MSR_READY;
    SetIrqLevel(1);
  }

  void StopTransfer() {
    auto drive = drives_[drive_index_];
    error_status_[0] &= ~7;
    error_status_[0] |= SR0_SEEK;
    error_status_[0] |= drive_index_;
    error_status_[0] |= drive->head() ? SR0_HEAD : 0;
    fifo_.push_back(error_status_[0]);
    fifo_.push_back(error_status_[1]);
    fifo_.push_back(error_status_[2]);
    fifo_.push_back(drive->cylinder());
    fifo_.push_back(drive->head());
    fifo_.push_back(drive->sector());
    fifo_.push_back(2);

    main_status_ |= MSR_READY | MSR_DIO;
    SetIrqLevel(1);
  }

  void ReadId() {
    drive_index_ = fifo_[1] & 3;
    auto drive = drives_[drive_index_];

    drive->set_head((fifo_[1] >> 2) & 1);
    fifo_.clear();

    StopTransfer();
  }

  void ReadWriteSector() {
    bool multi_track = fifo_[0] & 0x80;
    bool is_write = !(fifo_[0] & 2);
    drive_index_ = fifo_[1] & 3;
    auto drive = drives_[drive_index_];
  
    drive->Seek(fifo_[2], fifo_[3], fifo_[4]);
    
    auto count = fifo_[6];
    fifo_.clear();
    
    main_status_ &= ~MSR_READY;
    // read sector means write to dma buffer
    isa_dma_->WaitForChannel(!is_write, 2, [=](auto iov) {
      size_t bytes = count * SECTOR_SIZE;
      auto lba = drive->GetLba();
      auto image = drive->image();
      auto request = ImageIoRequest {
        .type = is_write ? kImageIoWrite : kImageIoRead,
        .position = lba * SECTOR_SIZE,
        .length = std::min(bytes, iov.iov_len),
      };
      request.vector.push_back(iov);

      image->QueueIoRequest(request, [this, drive, multi_track, is_write, lba, iov](auto ret) {
        if (ret < 0) {
          MV_PANIC("not implemented IO error");
          return;
        }

        if (multi_track) {
          drive->SetLba(lba + (ret / SECTOR_SIZE));
        } else {
          drive->Seek(drive->cylinder() + 1, drive->head(), 1);
        }
        StopTransfer();
      });
    });
  }

  void FormatTrack() {
    uint drive_index = fifo_[1] & 3;
    auto drive = drives_[drive_index];

    auto fill_byte = fifo_[5];
    auto count = fifo_[3];
    fifo_.clear();
    
    main_status_ &= ~MSR_READY;
    isa_dma_->WaitForChannel(false, 2, [=](auto iov) {
      MV_ASSERT(iov.iov_len >= count * 4);
      uint8_t buffer[SECTOR_SIZE];
      memset(buffer, fill_byte, SECTOR_SIZE);

      std::vector<ImageIoRequest> requests;

      auto image = drive->image();
      auto ptr = (uint8_t*)iov.iov_base;
      for (auto i = 0; i < count; i++) {
        drive->Seek(ptr[i * 4 + 0], ptr[i * 4 + 1], ptr[i * 4 + 2]);

        auto request = ImageIoRequest {
          .type = kImageIoWrite,
          .position = drive->GetLba() * SECTOR_SIZE,
          .length = SECTOR_SIZE,
        };
        request.vector.push_back(iovec {
          .iov_base = buffer,
          .iov_len = SECTOR_SIZE
        });
        requests.push_back(request);
      }


      image->QueueMultipleIoRequests(requests, [this, drive](auto ret) {
        if (ret < 0) {
          MV_PANIC("not implemented IO error");
          return;
        }

        drive->Seek(drive->cylinder() + 1, drive->head(), 1);
        StopTransfer();
      });
    });
  }

  void CheckCommand() {
    auto cmd = fifo_[0];
    auto length = fifo_.size();

    switch (cmd)
    {
    case 0x03:  // Specify
      if (length >= 3) {
        /* Step rate time, Head load time, Head unload time */
        step_rate_ = (fifo_[1] >> 4) & 0xF;
        head_load_time_ = fifo_[2] >> 1;
        fifo_.clear();
        main_status_ = MSR_READY;
      }
      break;
    case 0x04:  // Sense Drive Status
      if (length >= 2) {
        SenseDriveStatus();
      }
      break;
    case 0x45:  // Write Sector Without Multi-track
    case 0x46:  // Read Sector Without Multi-track
    case 0xC5:  // Write Sector
    case 0xE6:  // Read Sector
      if (length >= 9) {
        ReadWriteSector();
      }
      break;
    case 0x4d:  // Format Track Without Multi-track
    case 0xEd:  // Format Track
      if (length >= 6) {
        FormatTrack();
      }
      break;
    case 0x07:  // Recalibrate
      if (length >= 2) {
        Recalibrate();
      }
      break;
    case 0x08:  // Sense Interrupt Status
      SenseInterrupt();
      break;
    case 0x12:  // Perpendicular Mode
      if (length >= 2) {
        if (fifo_[1] & 0x80) {
          drives_[drive_index_]->set_perpendicular(fifo_[1] & 7);
        }
        fifo_.clear();
        main_status_ = MSR_READY;
      }
      break;
    case 0x13:  // Configure
      if (length >= 4) {
        config_ = fifo_[2];
        fifo_.clear();
        main_status_ = MSR_READY;
      }
      break;
    case 0x14:  // Lock And Unlock
    case 0x94:
      Lock();
      break;
    case 0x18:  // Part ID
      fifo_.clear();
      fifo_.push_back(0x41);
      main_status_ |= MSR_DIO;
      break;
    case 0x0A:  // Read ID
    case 0x4A:
      if (length >= 2) {
        ReadId();
      }
      break;
    case 0x0E:  // Dump Registers
      DumpRegisters();
      break;
    case 0x0F:  // Seek
      if (length >= 3) {
        Seek();
      }
      break;
    case 0x10:  // Version
      fifo_.clear();
      fifo_.push_back(0x90);
      main_status_ |= MSR_DIO;
      break;
    default:
      MV_PANIC("unimplemented FIFO command 0x%x", cmd);
    }
  }

  void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    auto port = resource->base + offset;
    auto value = data[0];

    if (debug_) {
      MV_LOG("write FDC 0x%x value=0x%x", port, value);
    }

    switch (port)
    {
    case REG_DIGITAL_OUTPUT:
      if ((value & DOR_RESET) && !(digital_output_ & DOR_RESET)) {
        Reset();
        reset_sense_remained_ = 4;
        /* Set interrupt code to 3: abnormal termination */
        error_status_[0] = SR0_READY_CHANGED;
        SetIrqLevel(1);
      }
      digital_output_ = value;
      drive_index_ = value & 3;
      break;
    case REG_DATA_FIFO:
      main_status_ |= MSR_COMMAND_BUSY;
      fifo_.push_back(value);
      CheckCommand();
      break;
    case REG_DATARATE_SELECT:
      if (value & 0x80) { // Soft Reset ??
        Reset();
        reset_sense_remained_ = 4;
        /* Set interrupt code to 3: abnormal termination */
        error_status_[0] = SR0_READY_CHANGED;
        SetIrqLevel(1);
      }
      datarate_select_ = value;
      break;
    case REG_CONFIG_CONTROL:
      datarate_select_ = (datarate_select_ & ~3) | (value & 3);
      break;
    default:
      Device::Write(resource, offset, data, size);
    }
  }

  void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    auto port = resource->base + offset;

    switch (port)
    {
    case REG_MAIN_STATUS:
      data[0] = main_status_;
      break;
    case REG_DATA_FIFO:
      if (!fifo_.empty()) {
        data[0] = fifo_.front();
        fifo_.pop_front();
      }
      if (fifo_.empty()) {
        main_status_ = MSR_READY;
        SetIrqLevel(0);
      }
      break;
    case REG_DIGITAL_INPUT:
      /* Or 0x80 if disk media changed */
      data[0] = 0x78;
      break;
    default:
      Device::Read(resource, offset, data, size);
    }

    if (debug_) {
      MV_LOG("read FDC 0x%x value=0x%x", port, data[0]);
    }
  }

};


DECLARE_DEVICE(I82078Fdc);
