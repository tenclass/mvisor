#include <mutex>
#include "serial_port.h"


void SerialPort::Initialize(SerialDeviceInterface* device, uint32_t id) {
  device_ = device;
  real_device_ = dynamic_cast<Device*>(device);
  port_id_ = id;
}

std::list<SerialPortListener>::iterator SerialPort::RegisterSerialPortListener(SerialPortListener callback) {
  std::unique_lock<std::recursive_mutex> lock(real_device_->mutex());
  return callbacks_.emplace(callbacks_.end(), callback);
}

void SerialPort::UnregisterSerialPortListener(std::list<SerialPortListener>::iterator it) {
  std::unique_lock<std::recursive_mutex> lock(real_device_->mutex());
  callbacks_.erase(it);
}

void SerialPort::SendMessage(uint8_t* data, size_t size) {
  // Lock console device
  auto console_device = dynamic_cast<Device*>(device_);
  std::lock_guard<std::recursive_mutex> lock(console_device->mutex());

  device_->SendMessage(this, data, size);
}

void SerialPort::set_ready(bool ready) {
  std::unique_lock<std::recursive_mutex> lock(real_device_->mutex());
  ready_ = ready;
  for (auto& callback : callbacks_) {
    callback(kSerialPortStatusChanged, nullptr, 0);
  }
}

void SerialPort::OnWritable() {
  std::unique_lock<std::recursive_mutex> lock(real_device_->mutex());
  writable_ = true;
}

void SerialPort::OnMessage(uint8_t* data, size_t size) {
  std::unique_lock<std::recursive_mutex> lock(real_device_->mutex());
  for (auto& callback : callbacks_) {
    callback(kSerialPortData, data, size);
  }
}
