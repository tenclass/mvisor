#include "display.h"

void Display::NotifyDisplayModeChange() {
  std::lock_guard<std::recursive_mutex> lock(display_mutex_);
  for (auto &listener : display_mode_change_listeners_) {
    listener();
  }
}

std::list<DisplayModeChangeListener>::iterator Display::RegisterDisplayModeChangeListener(DisplayModeChangeListener callback) {
  std::lock_guard<std::recursive_mutex> lock(display_mutex_);
  return display_mode_change_listeners_.emplace(display_mode_change_listeners_.end(), callback);
}

std::list<DisplayUpdateListener>::iterator Display::RegisterDisplayUpdateListener(DisplayUpdateListener callback) {
  std::lock_guard<std::recursive_mutex> lock(display_mutex_);
  return display_update_listeners_.emplace(display_update_listeners_.end(), callback);
}

void Display::UnregisterDisplayModeChangeListener(std::list<DisplayModeChangeListener>::iterator listener) {
  std::lock_guard<std::recursive_mutex> lock(display_mutex_);
  display_mode_change_listeners_.erase(listener);
}

void Display::UnregisterDisplayUpdateListener(std::list<DisplayUpdateListener>::iterator listener) {
  std::lock_guard<std::recursive_mutex> lock(display_mutex_);
  display_update_listeners_.erase(listener);
}
