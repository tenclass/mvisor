
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

# Make sure libyuv with BT.709. if failed, checkout https://github.com/tenclass/libyuv
code = '''
#include <stdio.h>
#include <libyuv.h>
int main() {
  uint8_t test_argb[128] = { 0, 0, 255, 0 };
  uint8_t test_y[128] = {0}, test_u[128] = {0}, test_v[128] = {0};
  libyuv::ARGBToI420(test_argb, 128, test_y, 128, test_u, 128, test_v, 128, 32, 1);
  if (test_y[0] == 0x3f && test_u[0] == 0x73 && test_v[0] == 0xb8) {
    return 0;
  }
  fprintf(stderr, "failed checking libyuv with BT.709, yuv=0x%x 0x%x 0x%x",
    test_y[0], test_u[0], test_v[0]);
  return -1;
}
'''
compiler = meson.get_compiler('cpp', native: false)
libyuv = compiler.find_library('yuv')
if not libyuv.found()
  error('libyuv is not found')
endif

result = compiler.run(code, name: 'libyuv with BT.709', dependencies: [libyuv, dependency('libjpeg')])
if not result.compiled()
  error('libyuv is not found')
endif

if result.returncode() != 0
  warning('libyuv without BT.709 is used')
endif

add_project_link_arguments(['-lyuv'], language: 'cpp')

