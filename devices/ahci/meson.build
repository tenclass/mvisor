mvisor_sources += files(
  'ahci_cdrom.cc',
  'ahci_disk.cc',
  'ahci_host.cc',
  'ahci_host.h',
  'ahci_internal.h',
  'ahci_port.cc',
  'ahci_port.h',
  'ata_internal.h',
  'ata_cdrom.cc',
  'ata_disk.cc',
  'ata_storage.cc',
  'ata_storage.h',
  'ich9_ahci.cc',
  'ide_cdrom.cc',
  'ide_disk.cc',
  'ide_host.cc',
  'ide_host.h',
  'ide_port.cc',
  'ide_port.h',
  'piix3_ide.cc'
)

proto_sources += proto_gen.process(
  'ahci_host.proto',
  'ata_cdrom.proto',
  'ide_host.proto'
)
