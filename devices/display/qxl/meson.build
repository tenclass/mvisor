mvisor_sources += files(
  'qxl.cc',
  'qxl.h',
  'qxl_render.cc',
  'qxl_render.h',
  'qxl_modes.h',
  'qxl_parser.cc',
  'qxl_parser.h'
)

proto_sources += proto_gen.process(
  'qxl.proto'
)

mvisor_deps += [
  dependency('zlib'),
  dependency('glib-2.0', version: '>= 2.38')
]

subdir('canvas')
