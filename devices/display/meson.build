mvisor_sources += files(
  'vbe.h',
  'vga.cc',
  'vga.h'
)

proto_sources += proto_gen.process(
  'vga.proto'
)

if get_option('qxl')
  subdir('qxl')
endif
