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

#include <algorithm>

#include <cstring>
#include <ctime>
#include "logger.h"
#include "device_manager.h"
#include "pb/cmos.pb.h"

#define RTC_BASE_ADDRESS 0x70

/*
 * MC146818 RTC registers
 * http://www.bioscentral.com/misc/cmosmap.htm
 */
#define RTC_SECONDS       0x00
#define RTC_SECONDS_ALARM 0x01
#define RTC_MINUTES       0x02
#define RTC_MINUTES_ALARM 0x03
#define RTC_HOURS         0x04
#define RTC_HOURS_ALARM   0x05
#define RTC_DAY_OF_WEEK   0x06
#define RTC_DAY_OF_MONTH  0x07
#define RTC_MONTH         0x08
#define RTC_YEAR          0x09
#define RTC_CENTURY       0x32

#define RTC_IRQ           8

#define RTC_REG_A   0x0A
#define RTC_REG_B   0x0B
#define RTC_REG_C   0x0C
#define RTC_REG_D   0x0D


static inline unsigned char bin2bcd(unsigned val)
{
  return ((val / 10) << 4) + val % 10;
}


class Cmos : public Device {
 private:
  bool      non_maskable_interrupt_disabled_;
  uint8_t   cmos_index_;
  uint8_t   cmos_data_[128];
  IoTimer*  rtc_timer_ = nullptr;

 public:

  Cmos() {
    set_parent_name("ich9-lpc");

    AddIoResource(kIoResourceTypePio, RTC_BASE_ADDRESS, 2, "CMOS");
  }

  void Disconnect() {
    DisableTimer(&rtc_timer_);
    Device::Disconnect();
  }

  void DisableTimer(IoTimer** timer) {
    if (*timer) {
      manager_->io()->RemoveTimer(*timer);
      *timer = nullptr;
    }
  }

  void Reset() {
    DisableTimer(&rtc_timer_);
    non_maskable_interrupt_disabled_ = false;
    cmos_index_ = 0;
    bzero(cmos_data_, sizeof(cmos_data_));

    /* timer frequency = 32.768kHz, alarm frequency = 1.024Hz */
    cmos_data_[RTC_REG_A] = 0x26;
    /* 24 hour mode */
    cmos_data_[RTC_REG_B] = 0x02;
    /* interrupt flags */
    cmos_data_[RTC_REG_C] = 0x00;
    /* battery power good */ 
    cmos_data_[RTC_REG_D] = 0x80;

    /* set floppy type */
    if (manager_->LookupDeviceByClass("Floppy")) {
      // 4 - 1.44MB, 3.5" - 2 heads, 80 tracks, 18 sectors
      cmos_data_[0x10] = 0x40;
    }
  }

  bool SaveState(MigrationWriter* writer) {
    CmosState state;
    state.set_index(cmos_index_);
    state.set_data(cmos_data_, sizeof(cmos_data_));
    state.set_nmi_disabled(non_maskable_interrupt_disabled_);
    writer->WriteProtobuf("CMOS", state);
    return Device::SaveState(writer);
  } 

  bool LoadState(MigrationReader* reader) {
    if (!Device::LoadState(reader)) {
      return false;
    }
    CmosState state;
    if (!reader->ReadProtobuf("CMOS", state)) {
      return false;
    }
    non_maskable_interrupt_disabled_ = state.nmi_disabled();
    cmos_index_ = state.index();
    memcpy(cmos_data_, state.data().data(), sizeof(cmos_data_));
  
    UpdateRtcTimer();
    return true;
  }

  /* always get time from host system, guest os cannot change date time now */
  void UpdateFromHostTime() {
    time_t timestamp;
    time(&timestamp);

    struct tm* tm = nullptr;
    if (has_key("rtc") && !std::get<std::string>(key_values_["rtc"]).compare("gmtime")) {
      tm = gmtime(&timestamp);
    } else {
      tm = localtime(&timestamp);
    }
    MV_ASSERT(tm != nullptr);

    cmos_data_[RTC_SECONDS] = bin2bcd(tm->tm_sec);
    cmos_data_[RTC_MINUTES] = bin2bcd(tm->tm_min);
    cmos_data_[RTC_HOURS] = bin2bcd(tm->tm_hour);
    cmos_data_[RTC_DAY_OF_WEEK] = bin2bcd(tm->tm_wday + 1);
    cmos_data_[RTC_DAY_OF_MONTH] = bin2bcd(tm->tm_mday);
    cmos_data_[RTC_MONTH] = bin2bcd(tm->tm_mon + 1);

    int year = tm->tm_year + 1900;
    cmos_data_[RTC_YEAR] = bin2bcd(year % 100);
    cmos_data_[RTC_CENTURY] = bin2bcd(year / 100);
  }

  void OnRtcTimer() {
    /* set RTC interrupt flag */
    cmos_data_[RTC_REG_C] |= 0x40;
    /* check if RTC interrupt enabled */
    if (cmos_data_[RTC_REG_B] & 0x40) {
      /* set IRQ flag */
      if (!(cmos_data_[RTC_REG_C] & 0x80)) {
        cmos_data_[RTC_REG_C] |= 0x80;
        manager_->SetGsiLevel(RTC_IRQ, 1);
      }
    }
  }

  /* RTC timer has performance problem, lots of IO causes lots of vmexits */
  void UpdateRtcTimer() {
    uint period_code = cmos_data_[RTC_REG_A] & 0xF;
    if (!period_code) {
      return;
    }
    if (period_code <= 2) {
      period_code += 7;
    }
    uint period = 1 << (period_code - 1);
    uint64_t period_ms = std::max(1u, period * 10000 / 32768); // use 32k Hz clock rate

    if (cmos_data_[RTC_REG_B] & 0x40) {
      if (rtc_timer_ == nullptr) {
        rtc_timer_ = manager_->io()->AddTimer(period_ms, true, std::bind(&Cmos::OnRtcTimer, this));
      } else {
        manager_->io()->ModifyTimer(rtc_timer_, period_ms);
      }
    } else {
      DisableTimer(&rtc_timer_);
    }
  }

  void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    if (offset == 0) {
      data[0] = 0xFF;
      return;
    }

    switch (cmos_index_) {
    case RTC_REG_C:
      manager_->SetGsiLevel(RTC_IRQ, 0);
      data[0] = cmos_data_[RTC_REG_C];
      cmos_data_[RTC_REG_C] = 0;
      break;
    case RTC_SECONDS:
    case RTC_HOURS:
    case RTC_DAY_OF_WEEK:
    case RTC_DAY_OF_MONTH:
    case RTC_MONTH:
    case RTC_YEAR:
    case RTC_CENTURY:
      UpdateFromHostTime();
      /* fall through */
    default:
      data[0] = cmos_data_[cmos_index_];
      break;
    }
  }

  void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    if (offset == 0) { /* index register */
      uint8_t value = data[0];
      cmos_index_  = value & ~(1UL << 7);
      non_maskable_interrupt_disabled_ = value & (1UL << 7);
    } else { /* data register */
      switch (cmos_index_) {
      case RTC_REG_A:
        /* bit 7 (update in progress) is readonly */
        cmos_data_[RTC_REG_A] = (data[0] & ~0x80) | (cmos_data_[RTC_REG_A] & 0x80);
        UpdateRtcTimer();
        break;
      case RTC_REG_B:
        cmos_data_[RTC_REG_B] = data[0];
        /* check if RTC interrupt flag is enabled */
        UpdateRtcTimer();
        break;
      case RTC_REG_C:
      case RTC_REG_D:
        /* Read-only */
        break;
      default:
        cmos_data_[cmos_index_] = data[0];
        break;
      }
    }
  }


};

DECLARE_DEVICE(Cmos);
