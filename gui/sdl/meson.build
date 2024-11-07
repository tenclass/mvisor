mvisor_sources += files(
  'viewer.cc',
  'viewer.h'
)

mvisor_deps += [
  dependency('alsa'),
  dependency('SDL2')
]

if get_option('mdebugger')
  if not get_option('gtk')
    error('mdebugger requires gtk')
  endif
  mvisor_version_data.set('HAS_MDEBUGGER', true)
endif
