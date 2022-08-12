
mvisor_sources += files(
  'connection.cc',
  'connection.h',
  'display_encoder.cc',
  'display_encoder.h',
  'server.cc',
  'server.h'
)

proto_sources += proto_gen.process(
  'sweet.proto',
  preserve_path_from: meson.current_source_dir()
)

mvisor_deps += [
  dependency('libjpeg'),
  dependency('opus'),
  dependency('x264')
]

add_project_link_arguments(['-lyuv'], language: 'cpp')
