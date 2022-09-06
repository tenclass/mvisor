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

#include "device_manager.h"
#include "logger.h"
#include "cmos_regs.h"
#include "cmos.pb.h"


static inline unsigned char bin2bcd(unsigned val) {
  return ((val / 10) << 4) + val % 10;
}

static inline unsigned int bcd2bin(uint8_t val) {
	return ((val) & 0x0f) + ((val) >> 4) * 10;
}

class Cmos : public Device {
 private:
  bool      non_maskable_interrupt_disabled_;
  uint8_t   cmos_index_;
  uint8_t   cmos_data_[128];
  IoTimer*  rtc_timer_ = nullptr;
  bool      rtc_timer_warned_ = false;
  IoTimer*  guest_timer_ = nullptr;
  time_t    guest_time_ = 0;
  bool      use_utc_time_ = false;

 public:

  Cmos() {
    set_parent_name("ich9-lpc");

    AddIoResource(kIoResourceTypePio, RTC_BASE_ADDRESS, 2, "CMOS");
  }

  void Connect() {
    Device::Connect();
    
    if (has_key("rtc") && std::get<std::string>(key_values_["rtc"]) == "gmtime") {
      use_utc_time_ = true;
    }
  
    /* Initialize guest time to host time or may be configured manually? */
    guest_time_ = time(nullptr);
  }

  void Disconnect() {
    DisableTimer(&rtc_timer_);
    DisableTimer(&guest_timer_);
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
    DisableTimer(&guest_timer_);
    non_maskable_interrupt_disabled_ = false;
    cmos_index_ = 0;
    bzero(cmos_data_, sizeof(cmos_data_));

    /* timer frequency = 32.768kHz, alarm frequency = 1.024Hz */
    cmos_data_[RTC_REG_A] = 0x26;
    /* 24 hour mode */
    cmos_data_[RTC_REG_B] = REG_B_24H;
    /* interrupt flags */
    cmos_data_[RTC_REG_C] = 0x00;
    /* battery power good */ 
    cmos_data_[RTC_REG_D] = REG_D_VALID_RAM;

    /* set floppy type */
    if (manager_->LookupDeviceByClass("Floppy")) {
      // 4 - 1.44MB, 3.5" - 2 heads, 80 tracks, 18 sectors
      cmos_data_[0x10] = 0x40;
    }

    EnableGuestTimer();
  }

  bool SaveState(MigrationWriter* writer) {
    /* force to update cmos from host time */
    SetGuestTimeToCmos();

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

    /* get guest_time_ from cmos at first */
    GetGuestTimeFromCmos();
  
    UpdateRtcTimer();
    return true;
  }

  void EnableGuestTimer() {
    MV_ASSERT(!guest_timer_);
    guest_timer_ = manager_->io()->AddTimer(NS_PER_SECOND, true, [this]() {
      /* To keep time, we either increase every second or synchronize the host time
       * This should be configurable */
      guest_time_++; 
    });
  }

  void GetGuestTimeFromCmos() {
    struct tm guest_tm = {0};
    guest_tm.tm_sec = bcd2bin(cmos_data_[RTC_SECONDS] & 0x7F);
    guest_tm.tm_min = bcd2bin(cmos_data_[RTC_MINUTES] & 0x7F);
    guest_tm.tm_hour = bcd2bin(cmos_data_[RTC_HOURS] & 0x3F);
    guest_tm.tm_mday = bcd2bin(cmos_data_[RTC_DAY_OF_MONTH] & 0x3F);
    guest_tm.tm_mon = bcd2bin(cmos_data_[RTC_MONTH] & 0x1F) - 1;
    guest_tm.tm_year = bcd2bin(cmos_data_[RTC_CENTURY]) * 100 + bcd2bin(cmos_data_[RTC_YEAR]) - 1900;
    guest_tm.tm_wday = bcd2bin(cmos_data_[RTC_DAY_OF_WEEK] & 0x07) - 1;
    guest_tm.tm_yday = 0;
    guest_tm.tm_isdst = 0;

    // FIXME: should check use_utc_time_ here
    guest_time_ = mktime(&guest_tm);
  }

  void SetGuestTimeToCmos() {
    struct tm tm;
    if (use_utc_time_) {
      gmtime_r(&guest_time_, &tm);
    } else {
      localtime_r(&guest_time_, &tm);
    }

    cmos_data_[RTC_SECONDS] = bin2bcd(tm.tm_sec);
    cmos_data_[RTC_MINUTES] = bin2bcd(tm.tm_min);
    cmos_data_[RTC_HOURS] = bin2bcd(tm.tm_hour);
    cmos_data_[RTC_DAY_OF_WEEK] = bin2bcd(tm.tm_wday + 1);
    cmos_data_[RTC_DAY_OF_MONTH] = bin2bcd(tm.tm_mday);
    cmos_data_[RTC_MONTH] = bin2bcd(tm.tm_mon + 1);

    int year = tm.tm_year + 1900;
    cmos_data_[RTC_YEAR] = bin2bcd(year % 100);
    cmos_data_[RTC_CENTURY] = bin2bcd(year / 100);
  }

  void PrintTicksPerSecond() {
    static time_t counter = 0, last_time =0;
    if (time(NULL) > last_time) {
      last_time = time(NULL);
      MV_LOG("ticks=%ld", counter);
      counter = 0;
    } else {
      counter++;
    }
  }

  void OnRtcTimer() {
    /* set RTC interrupt flag */
    cmos_data_[RTC_REG_C] |= REG_C_PF;
    /* check if RTC interrupt enabled */
    if (cmos_data_[RTC_REG_B] & REG_B_PIE) {
      /* set IRQ flag */
      if (!(cmos_data_[RTC_REG_C] & REG_C_IRQF)) {
        cmos_data_[RTC_REG_C] |= REG_C_IRQF;
        manager_->SetGsiLevel(RTC_IRQ, 1);
      }

      // PrintTicksPerSecond();
    }
  }

  void UpdateRtcTimer() {
    uint period_code = cmos_data_[RTC_REG_A] & 0xF;
    if (!period_code) {
      return;
    }
    if (period_code <= 2) {
      period_code += 7;
    }
    uint period = 1 << (period_code - 1);
    int64_t period_ns = NS_PER_SECOND * period / 32768; // use 32k Hz clock rate

    if (cmos_data_[RTC_REG_B] & REG_C_PF) {
      if (rtc_timer_ == nullptr) {
        rtc_timer_ = manager_->io()->AddTimer(period_ns, true, std::bind(&Cmos::OnRtcTimer, this));
        // Try to warn user if OS switches to RTC timer
        if (!rtc_timer_warned_) {
          MV_LOG("The OS is using RTC timer which has performance problem!");
          rtc_timer_warned_ = true;
        }
      } else {
        manager_->io()->ModifyTimer(rtc_timer_, period_ns);
      }
    } else {
      DisableTimer(&rtc_timer_);
    }
  }

  bool IsUpdateInProgress() {
    if (!guest_timer_) {
      return false;
    }
  
    /* fake UIP at last 1ms of every second */
    auto now = std::chrono::steady_clock::now();
    auto delta_ms = std::chrono::duration_cast<std::chrono::milliseconds>(guest_timer_->next_timepoint - now).count();
    if (delta_ms <= 1) {
      return true;
    }
    return false;
  }

  void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    MV_UNUSED(resource);
    MV_ASSERT(size == 1);

    if (offset == 0) {
      data[0] = 0xFF;
      return;
    }

    switch (cmos_index_) {
    case RTC_REG_A:
      data[0] = cmos_data_[cmos_index_];
      if (IsUpdateInProgress()) {
        data[0] |= REG_A_UIP;
      }
      break;
    case RTC_REG_C:
      manager_->SetGsiLevel(RTC_IRQ, 0);
      data[0] = cmos_data_[RTC_REG_C];
      cmos_data_[RTC_REG_C] = 0;
      break;
    case RTC_SECONDS:
    case RTC_MINUTES:
    case RTC_HOURS:
    case RTC_DAY_OF_WEEK:
    case RTC_DAY_OF_MONTH:
    case RTC_MONTH:
    case RTC_YEAR:
    case RTC_CENTURY:
      SetGuestTimeToCmos();
      /* fall through */
    default:
      data[0] = cmos_data_[cmos_index_];
      break;
    }
  }

  void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) {
    MV_UNUSED(resource);
    MV_ASSERT(size == 1);
    uint8_t value = data[0];

    if (offset == 0) { /* index register */
      cmos_index_  = value & ~(1UL << 7);
      non_maskable_interrupt_disabled_ = value & (1UL << 7);
    } else { /* data register */
      uint8_t diff = cmos_data_[cmos_index_] ^ value;
      switch (cmos_index_) {
      case RTC_SECONDS:
      case RTC_MINUTES:
      case RTC_HOURS:
      case RTC_DAY_OF_WEEK:
      case RTC_DAY_OF_MONTH:
      case RTC_MONTH:
      case RTC_YEAR:
      case RTC_CENTURY:
        cmos_data_[cmos_index_] = value;
        /* only update guest_time_ when REG_B_SET is not set */
        if (!(cmos_data_[RTC_REG_B] & REG_B_SET)) {
          GetGuestTimeFromCmos();
        }
        break;
      case RTC_REG_A:
        /* bit 7 (update in progress) is readonly */
        cmos_data_[RTC_REG_A] = (value & ~REG_A_UIP) | (cmos_data_[RTC_REG_A] & REG_A_UIP);
        /* periodic code changed */
        if (diff & 0xF) {
          UpdateRtcTimer();
        }
        break;
      case RTC_REG_B:
        cmos_data_[RTC_REG_B] = value;

        /* check if enable or disable update timer */
        if (diff & REG_B_SET) {
          if (value & REG_B_SET) {
            /* update cmos data before stopping timer */
            SetGuestTimeToCmos();
            DisableTimer(&guest_timer_);
            /* reset update flags */
            cmos_data_[RTC_REG_A] &= ~REG_A_UIP;
            cmos_data_[RTC_REG_B] &= ~REG_B_UIE;
          } else {
            /* update guset_time_ before starting timer */
            GetGuestTimeFromCmos();
            EnableGuestTimer();
          }
        }

        /* check if RTC interrupt is toggled */
        if (diff & REG_B_PIE) {
          UpdateRtcTimer();
        }
        if (diff & REG_B_AIE) {
          MV_PANIC("alarm is not implemented");
        }
        if (diff & REG_B_UIE) {
          MV_PANIC("update-ended interrupt is not implemented");
        }
        break;
      case RTC_REG_C:
      case RTC_REG_D:
        /* Read-only */
        break;
      default:
        cmos_data_[cmos_index_] = value;
        break;
      }
    }
  }
};

DECLARE_DEVICE(Cmos);
