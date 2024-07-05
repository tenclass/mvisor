mvisor_sources += files(
  'configuration.cc',
  'device_manager.cc',
  'device.cc',
  'io_thread.cc',
  'machine.cc',
  'memory_manager.cc',
  'memory_region.cc',
  'object.cc',
  'pci_device.cc',
  'vcpu.cc',
  'vfio_manager.cc'
)

proto_sources += proto_gen.process(
  'pci_device.proto',
  'vcpu.proto',
  'memory_manager.proto',
  'io_thread.proto'
)

mvisor_deps += dependency('yaml-cpp')
