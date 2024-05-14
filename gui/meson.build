mvisor_sources += files(
  'keymap.c',
  'keymap.h',
)

subdir('vnc')

if get_option('sdl')
  subdir('sdl')
  mvisor_version_data.set('HAS_SDL', true)
endif
if get_option('gtk')
  subdir('gtk')
  mvisor_version_data.set('HAS_GTK', true)
endif
