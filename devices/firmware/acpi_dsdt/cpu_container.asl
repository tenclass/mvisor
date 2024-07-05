// SSDT for CPU Container Device
DefinitionBlock("cpu.aml", "SSDT", 1, "TENCLS", "MVPC", 1) {
  Device(\_SB.CPUS) {
    Name (_HID, "ACPI0010")         // Processor Container Device
    Name (_CID, EisaId ("PNP0A05")) // Generic Container Device
  }
}
