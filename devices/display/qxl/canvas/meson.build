
mvisor_sources += files(
  'canvas_utils.c',
  'canvas_utils.h',
  'draw.h',
  'lines.c',
  'lines.h',
  'log.c',
  'log.h',
  'lz.c',
  'lz.h',
  'lz_common.h',
  'lz_config.h',
  'macros.h',
  'mem.c',
  'mem.h',
  'pixman_utils.c',
  'pixman_utils.h',
  'quic.c',
  'quic.h',
  'quic_config.h',
  'rect.h',
  'region.c',
  'region.h',
  'ring.h',
  'rop3.c',
  'rop3.h',
  'verify.h',
  'sw_canvas.c',
  'sw_canvas.h'
)

mvisor_deps += [
  dependency('pixman-1', version: '>= 0.17.7'),
  dependency('glib-2.0', version: '>= 2.38')
]

add_project_arguments('-Wno-unused-parameter', language: 'c')
