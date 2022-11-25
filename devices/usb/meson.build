
mvisor_sources += files(
  'piix3_uhci.cc',
  'uhci_host.cc',
  'uhci_host.h',
  'usb_descriptor.h',
  'usb_device.cc',
  'usb_device.h',
  'usb_hid.cc',
  'usb_hid.h',
  'usb_midi.cc',
  'usb_tablet.cc',
  'usb_wacom.cc',
  'usb.h',
  'xhci_host.cc',
  'xhci_internal.h',
)

proto_sources += proto_gen.process(
  'uhci_host.proto',
  'usb_device.proto',
  'xhci_host.proto'
)
