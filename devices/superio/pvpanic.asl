// SSDT for PvPanic device
DefinitionBlock("pvpanic.aml", "SSDT", 1, "TENCLS", "MVPC", 1) {
  External(\_SB.PCI0.ISA, DeviceObj)
  Scope(\_SB.PCI0.ISA) {
    Device(PEVT) {
      Name(_HID, "QEMU0001")
      /* PEST will be patched to be Zero if no such device */
      Name(PEST, 0xFFFF)
      OperationRegion(PEOR, SystemIO, PEST, 0x01)
      Field(PEOR, ByteAcc, NoLock, Preserve) {
        PEPT,   8,
      }

      Method(_STA, 0, NotSerialized) {
        Store(PEST, Local0)
        If (LEqual(Local0, Zero)) {
          Return (0x00)
        } Else {
          Return (0x0F)
        }
      }

      Method(RDPT, 0, NotSerialized) {
        Store(PEPT, Local0)
        Return (Local0)
      }

      Method(WRPT, 1, NotSerialized) {
        Store(Arg0, PEPT)
      }

      Name(_CRS, ResourceTemplate() {
        IO(Decode16, 0x00, 0x00, 0x01, 0x01, IO)
      })

      CreateWordField(_CRS, IO._MIN, IOMN)
      CreateWordField(_CRS, IO._MAX, IOMX)

      Method(_INI, 0, NotSerialized) {
        Store(PEST, IOMN)
        Store(PEST, IOMX)
      }
    }
  }
}
