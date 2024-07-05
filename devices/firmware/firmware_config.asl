// SSDT for QEMU Firmware Configuration Interface
DefinitionBlock("firmware_config.aml", "SSDT", 1, "TENCLS", "MVPC", 1) {
  External(\_SB.PCI0, DeviceObj)
  Scope(\_SB.PCI0) {
    Device (FWCF)
    {
      Name (_HID, "QEMU0002") // _HID: Hardware ID
      Name (_STA, 0x0B)       // _STA: Status
      Name (_CRS, ResourceTemplate ()
      {
        IO (Decode16,
          0x0510,             // Range Minimum
          0x0510,             // Range Maximum
          0x01,               // Alignment
          0x0C,               // Length
            )
      })
    }
  }
}
