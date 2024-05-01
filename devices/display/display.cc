#include "display.h"

void Display::NotifyDisplayModeChange() {
  for (auto &listener : display_mode_change_listeners_) {
    listener();
  }
}

void Display::NotifyDisplayUpdate() {
  for (auto &listener : display_update_listeners_) {
    listener();
  }
}

void Display::RegisterDisplayModeChangeListener(DisplayModeChangeListener callback) {
  display_mode_change_listeners_.push_back(callback);
}

void Display::RegisterDisplayUpdateListener(DisplayUpdateListener callback) {
  display_update_listeners_.push_back(callback);
}

