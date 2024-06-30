mvisor_sources += files(
  'firmware_config.cc',
  'firmware_config.h',
  'smbios.cc',
  'smbios.h',
  'acpi_builder.cc',
  'acpi_builder.h',
  'acpi_loader.h',
  'acpi_loader.cc',
  'acpi.h'
)

proto_sources += proto_gen.process(
  'firmware_config.proto'
)

subdir('acpi_dsdt')
