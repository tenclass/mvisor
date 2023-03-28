mvisor_sources += files(
  'virtio_block.cc',
  'virtio_console.cc',
  'virtio_fs.cc',
  'virtio_vgpu.cc',
  'virtio_network.cc',
  'virtio_pci.cc',
  'virtio_pci.h'
)

mvisor_deps += [
  dependency('virglrenderer')
]

proto_sources += proto_gen.process(
  'virtio_console.proto',
  'virtio_pci.proto',
  'virtio_vgpu.proto'
)

subdir('fuse')
