mvisor_sources += files(
  'debug_console.cc',
  'qxl.cc',
  'qxl.h',
  'vbe.h',
  'vga.cc',
  'vga.h'
)

proto_sources += proto_gen.process(
  'qxl.proto',
  'vga.proto'
)

mvisor_deps += dependency('zlib')
