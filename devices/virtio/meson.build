mvisor_sources += files(
  'virtio_block.cc',
  'virtio_console.cc',
  'virtio_fs.cc',
  'virtio_network.cc',
  'virtio_pci.cc',
  'virtio_pci.h'
)

proto_sources += proto_gen.process(
  'virtio_console.proto',
  'virtio_pci.proto',
)

if get_option('vgpu')
  mvisor_sources += files(
    'virtio_vgpu.cc'
  )

  proto_sources += proto_gen.process(
    'virtio_vgpu.proto'
  )

  mvisor_deps += [
    dependency('virglrenderer')
  ]
endif

subdir('fuse')
