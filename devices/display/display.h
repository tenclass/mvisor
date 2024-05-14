/* 
 * MVisor Display Base Class
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

#ifndef _MVISOR_DEVICES_DISPLAY_H
#define _MVISOR_DEVICES_DISPLAY_H

#include <mutex>
#include "device_interface.h"

enum DisplayMode {
  kDisplayModeUnknown = 0,
  kDisplayModeVga = 1,
  kDisplayModeQxl = 2,
  kDisplayModeVirtio = 3,
};

class Display : public DisplayInterface {
 protected:
  std::recursive_mutex                  display_mutex_;
  std::list<DisplayModeChangeListener>  display_mode_change_listeners_;
  std::list<DisplayUpdateListener>      display_update_listeners_;
  DisplayMode                           display_mode_ = kDisplayModeUnknown;

  virtual void NotifyDisplayModeChange();
  virtual void NotifyDisplayUpdate() = 0;

 public:
  virtual std::list<DisplayModeChangeListener>::iterator RegisterDisplayModeChangeListener(DisplayModeChangeListener callback);
  virtual std::list<DisplayUpdateListener>::iterator RegisterDisplayUpdateListener(DisplayUpdateListener callback);
  virtual void UnregisterDisplayModeChangeListener(std::list<DisplayModeChangeListener>::iterator listener);
  virtual void UnregisterDisplayUpdateListener(std::list<DisplayUpdateListener>::iterator listener);
};


#define _MB(x) (x * (1 << 20))

#endif // _MVISOR_DEVICES_DISPLAY_H
