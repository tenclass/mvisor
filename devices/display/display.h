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

#include "device_interface.h"

enum DisplayMode {
  kDisplayModeUnknown = 0,
  kDisplayModeVga = 1,
  kDisplayModeQxl = 2,
  kDisplayModeVirtio = 3,
};

class Display : public DisplayInterface {
 private:
  std::vector<DisplayModeChangeListener> display_mode_change_listeners_;
  std::vector<DisplayUpdateListener> display_update_listeners_;

 protected:
  DisplayMode display_mode_ = kDisplayModeUnknown;
  void NotifyDisplayModeChange();
  void NotifyDisplayUpdate();

 public:
  virtual void RegisterDisplayModeChangeListener(DisplayModeChangeListener callback);
  virtual void RegisterDisplayUpdateListener(DisplayUpdateListener callback);
};


#define _MB(x) (x * (1 << 20))

#endif // _MVISOR_DEVICES_DISPLAY_H
