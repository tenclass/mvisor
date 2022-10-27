mvisor_sources += files(
  'configuration.cc',
  'device_manager.cc',
  'device.cc',
  'io_thread.cc',
  'machine.cc',
  'memory_manager.cc',
  'object.cc',
  'pci_device.cc',
  'vcpu.cc'
)

proto_sources += proto_gen.process(
  'pci_device.proto',
  'vcpu.proto',
  'dirty_memory.proto',
  'image.proto'
)

mvisor_deps += dependency('yaml-cpp')
