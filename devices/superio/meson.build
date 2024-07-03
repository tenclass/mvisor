mvisor_sources += files(
  'cmos.cc',
  'debug_console.cc',
  'dummy.cc',
  'floppy.cc',
  'i8257_dma.cc',
  'i82078_fdc.cc',
  'ps2.cc',
  'uart.cc',
  'pvpanic.cc',
  'apple_smc.cc',
)

proto_sources += proto_gen.process(
  'cmos.proto',
  'i8257_dma.proto',
  'i82078_fdc.proto',
  'ps2.proto'
)

iasl_sources += iasl_gen.process(
  'apple_smc.asl'
)
