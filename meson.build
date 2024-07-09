project('mvisor', 'c', 'cpp',
  version: '2.7.0',
  license: 'GPLv3',
  default_options: [
    'buildtype=debugoptimized',
    'warning_level=2',
    'cpp_std=c++17',
    'werror=true'
  ]
)

mvisor_version_data = configuration_data()
mvisor_version_data.set_quoted('VERSION', meson.project_version())

add_project_arguments([
  '-Wno-address-of-packed-member',
  '-Wno-missing-field-initializers',
  '-mavx2'
], language: 'cpp')

add_project_link_arguments(['-lstdc++fs'], language: 'cpp')

protoc = find_program('protoc')
proto_gen = generator(protoc,
  output: ['@BASENAME@.pb.cc', '@BASENAME@.pb.h'],
  arguments: ['--proto_path=@CURRENT_SOURCE_DIR@', '--cpp_out=@BUILD_DIR@', '@INPUT@']
)
proto_sources = []

iasl = find_program('iasl', required: true)
iasl_gen = generator(iasl,
  output: ['@BASENAME@.hex', '@BASENAME@.aml'],
  arguments: ['-tc', '-vi', '-p', '@BUILD_DIR@/@BASENAME@', '@INPUT@']
)
iasl_sources = []

mvisor_include = [include_directories('include')]
mvisor_sources = ['main.cc']

openssl_dep = dependency('openssl', required : false)
if openssl_dep.found()
  mvisor_version_data.set('HAS_OPENSSL', true)
endif

mvisor_deps = [
  dependency('threads'),
  openssl_dep
]

subdir('core')
subdir('migration')
subdir('devices')
subdir('images')
subdir('networks')
subdir('utilities')
subdir('gui')

if get_option('sweet-server')
  subdir('sweet-server')
  mvisor_version_data.set('HAS_SWEET_SERVER', true)
endif

proto_interface = declare_dependency(
  sources: proto_sources,
  dependencies: dependency('protobuf')
)

mvisor_deps += proto_interface

iasl_interface = declare_dependency(
  sources: iasl_sources
)

mvisor_deps += iasl_interface

mvisor = executable('mvisor',
  sources: mvisor_sources,
  include_directories : mvisor_include,
  dependencies: mvisor_deps,
  install: true,
  install_dir: '/mnt/server/opt/mvisor/build/bin/'
)

configure_file(output: 'version.h',
  configuration: mvisor_version_data
)

summary({
  'sdl': get_option('sdl'),
  'vgpu': get_option('vgpu'),
  'sweet-server': get_option('sweet-server')
}, bool_yn: true, section: 'Options')


run_target('run', command: [mvisor])
run_target('debug', command: ['gdb', '-ex', 'handle SIG34 nostop pass', '-ex', 'run', mvisor])
run_target('load', command: [mvisor, '-config', '/tmp/save/configuration.yaml', '-load', '/tmp/save'])

