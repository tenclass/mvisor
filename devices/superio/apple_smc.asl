// SSDT for Apple SMC
DefinitionBlock("apple_smc.aml", "SSDT", 1, "TENCLS", "MVPC", 1) {
  External(\_SB.PCI0.ISA, DeviceObj)
  Scope(\_SB.PCI0.ISA) {
    Device (SMC)
    {
      Name (_HID, EisaId ("APP0001"))  // _HID: Hardware ID
      Name (_STA, 0x0B)  // _STA: Status
      Name (_CRS, ResourceTemplate ()  // _CRS: Current Resource Settings
      {
        IO (Decode16,
          0x0300,             // Range Minimum
          0x0300,             // Range Maximum
          0x01,               // Alignment
          0x20,               // Length
          )
        IRQNoFlags ()
          {6}
      })
    }
  }
}
