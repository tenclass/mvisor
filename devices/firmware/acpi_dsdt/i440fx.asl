DefinitionBlock(
    "i440fx.aml",      // Output file
    "DSDT",         // Table signature
    1,              // DSDT revision
    "TENCLS",       // OEM ID
    "MVPC",         // Table ID
    1               // OEM revision
) {

    // Device object for the PCI root bus
    Scope(\_SB) {
        Device(PCI0) {
            Name(_HID, EisaId("PNP0A03"))
            Name(_UID, 0x00)
        }
    }

    // PCI root bus resource template
    Scope(\_SB.PCI0) {
        Name (CRES, ResourceTemplate ()  // _CRS: Current Resource Settings
        {
            WordBusNumber (ResourceProducer, MinFixed, MaxFixed, PosDecode,
                0x0000,             // Granularity
                0x0000,             // Range Minimum
                0x00FF,             // Range Maximum
                0x0000,             // Translation Offset
                0x0100,             // Length
                ,, )
            IO (Decode16,
                0x0CF8,             // Range Minimum
                0x0CF8,             // Range Maximum
                0x01,               // Alignment
                0x08,               // Length
                )
            WordIO (ResourceProducer, MinFixed, MaxFixed, PosDecode, EntireRange,
                0x0000,             // Granularity
                0x0000,             // Range Minimum
                0x0CF7,             // Range Maximum
                0x0000,             // Translation Offset
                0x0CF8,             // Length
                ,, , TypeStatic, DenseTranslation)
            WordIO (ResourceProducer, MinFixed, MaxFixed, PosDecode, EntireRange,
                0x0000,             // Granularity
                0x0D00,             // Range Minimum
                0xFFFF,             // Range Maximum
                0x0000,             // Translation Offset
                0xF300,             // Length
                ,, , TypeStatic, DenseTranslation)
            DWordMemory (ResourceProducer, PosDecode, MinFixed, MaxFixed, Cacheable, ReadWrite,
                0x00000000,         // Granularity
                0x000A0000,         // Range Minimum
                0x000BFFFF,         // Range Maximum
                0x00000000,         // Translation Offset
                0x00020000,         // Length
                ,, , AddressRangeMemory, TypeStatic)
            DWordMemory (ResourceProducer, PosDecode, MinFixed, MaxFixed, NonCacheable, ReadWrite,
                0x00000000,         // Granularity
                0xE0000000,         // Range Minimum
                0xFEBFFFFF,         // Range Maximum
                0x00000000,         // Translation Offset
                0x1EC00000,         // Length
                ,, , AddressRangeMemory, TypeStatic)
        })

        Name(CR64, ResourceTemplate() {
            QWordMemory (ResourceProducer, PosDecode, MinFixed, MaxFixed, Cacheable, ReadWrite,
                0x0000000000000000, // Granularity
                0x0000380000000000, // Range Minimum
                0x00003807FFFFFFFF, // Range Maximum
                0x0000000000000000, // Translation Offset
                0x0000000800000000, // Length
                ,, , AddressRangeMemory, TypeStatic)
        })

        Method(_CRS, 0) {
            External(PR64, IntObj)
            If (LEqual(PR64, Zero)) {
                Return (CRES)
            }
            /* add pci64 and return result */
            ConcatenateResTemplate(CRES, CR64, Local1)
            Return (Local1)
        }
    }
    Scope(\_SB.PCI0) {
        Device(VGA) {
            Name(_ADR, 0x00020000)
            OperationRegion(PCIC, PCI_Config, Zero, 0x4)
            Field(PCIC, DWordAcc, NoLock, Preserve) {
                VEND, 32
            }
            Method(_S1D, 0, NotSerialized) {
                Return (0x00)
            }
            Method(_S2D, 0, NotSerialized) {
                Return (0x00)
            }
            Method(_S3D, 0, NotSerialized) {
                If (LEqual(VEND, 0x1001b36)) {
                    Return (0x03)           // QXL
                } Else {
                    Return (0x00)
                }
            }
        }
    }
    // PIIX4 PM
    Scope(\_SB.PCI0) {
        Device(PX13) {
            Name(_ADR, 0x00010003)
            OperationRegion(P13C, PCI_Config, 0x00, 0xFF)
        }
    }

    // PIIX3 PCI to ISA bridge
    Scope(\_SB.PCI0) {
      Device(ISA) {
          Name(_ADR, 0x00010000)

          /* PIIX PCI to ISA irq remapping */
          OperationRegion(P40C, PCI_Config, 0x60, 0x04)

          /* enable bits */
          Field(\_SB.PCI0.PX13.P13C, AnyAcc, NoLock, Preserve) {
              Offset(0x5F),
              , 7, LPEN, 1,         // LPT
              Offset(0x67),
              , 3, CAEN, 1,         // COM1
              , 3, CBEN, 1,         // COM2
          }
          Name(FDEN, 1)
      }
    }

    // ISA devices
    Scope(\_SB.PCI0.ISA) {
        Device(RTC) {
            Name(_HID, EisaId("PNP0B00"))
            Name(_CRS, ResourceTemplate() {
                IO(Decode16, 0x0070, 0x0070, 0x10, 0x02)
                IRQNoFlags() { 8 }
                IO(Decode16, 0x0072, 0x0072, 0x02, 0x06)
            })
        }

        Device(KBD) {
            Name(_HID, EisaId("PNP0303"))
            Method(_STA, 0, NotSerialized) {
                Return (0x0f)
            }
            Name(_CRS, ResourceTemplate() {
                IO(Decode16, 0x0060, 0x0060, 0x01, 0x01)
                IO(Decode16, 0x0064, 0x0064, 0x01, 0x01)
                IRQNoFlags() { 1 }
            })
        }

        Device(MOU) {
            Name(_HID, EisaId("PNP0F13"))
            Method(_STA, 0, NotSerialized) {
                Return (0x0f)
            }
            Name(_CRS, ResourceTemplate() {
                IRQNoFlags() { 12 }
            })
        }

        Device(FDC0) {
            Name(_HID, EisaId("PNP0700"))
            Method(_STA, 0, NotSerialized) {
                Store(FDEN, Local0)
                If (LEqual(Local0, 0)) {
                    Return (0x00)
                } Else {
                    Return (0x0F)
                }
            }
            Name(_CRS, ResourceTemplate() {
                IO(Decode16, 0x03F2, 0x03F2, 0x00, 0x04)
                IO(Decode16, 0x03F7, 0x03F7, 0x00, 0x01)
                IRQNoFlags() { 6 }
                DMA(Compatibility, NotBusMaster, Transfer8) { 2 }
            })
        }

        Device(LPT) {
            Name(_HID, EisaId("PNP0400"))
            Method(_STA, 0, NotSerialized) {
                Store(LPEN, Local0)
                If (LEqual(Local0, 0)) {
                    Return (0x00)
                } Else {
                    Return (0x0F)
                }
            }
            Name(_CRS, ResourceTemplate() {
                IO(Decode16, 0x0378, 0x0378, 0x08, 0x08)
                IRQNoFlags() { 7 }
            })
        }

        Device(COM1) {
            Name(_HID, EisaId("PNP0501"))
            Name(_UID, 0x01)
            Method(_STA, 0, NotSerialized) {
                Store(CAEN, Local0)
                If (LEqual(Local0, 0)) {
                    Return (0x00)
                } Else {
                    Return (0x0F)
                }
            }
            Name(_CRS, ResourceTemplate() {
                IO(Decode16, 0x03F8, 0x03F8, 0x00, 0x08)
                IRQNoFlags() { 4 }
            })
        }

        Device(COM2) {
            Name(_HID, EisaId("PNP0501"))
            Name(_UID, 0x02)
            Method(_STA, 0, NotSerialized) {
                Store(CBEN, Local0)
                If (LEqual(Local0, 0)) {
                    Return (0x00)
                } Else {
                    Return (0x0F)
                }
            }
            Name(_CRS, ResourceTemplate() {
                IO(Decode16, 0x02F8, 0x02F8, 0x00, 0x08)
                IRQNoFlags() { 3 }
            })
        }
    }

    // PCI IRQ routing
    Scope(\_SB) {
        Scope(PCI0) {
            Name(_PRT, Package() {
                Package() { 0x0000ffff, 0, LNKD, 0 }, Package() { 0x0000ffff, 1, LNKA, 0 }, Package() { 0x0000ffff, 2, LNKB, 0 }, Package() { 0x0000ffff, 3, LNKC, 0 },
                Package() { 0x0001ffff, 0, LNKS, 0 }, Package() { 0x0001ffff, 1, LNKB, 0 }, Package() { 0x0001ffff, 2, LNKC, 0 }, Package() { 0x0001ffff, 3, LNKD, 0 },
                Package() { 0x0002ffff, 0, LNKB, 0 }, Package() { 0x0002ffff, 1, LNKC, 0 }, Package() { 0x0002ffff, 2, LNKD, 0 }, Package() { 0x0002ffff, 3, LNKA, 0 },
                Package() { 0x0003ffff, 0, LNKC, 0 }, Package() { 0x0003ffff, 1, LNKD, 0 }, Package() { 0x0003ffff, 2, LNKA, 0 }, Package() { 0x0003ffff, 3, LNKB, 0 },
                Package() { 0x0004ffff, 0, LNKD, 0 }, Package() { 0x0004ffff, 1, LNKA, 0 }, Package() { 0x0004ffff, 2, LNKB, 0 }, Package() { 0x0004ffff, 3, LNKC, 0 },
                Package() { 0x0005ffff, 0, LNKA, 0 }, Package() { 0x0005ffff, 1, LNKB, 0 }, Package() { 0x0005ffff, 2, LNKC, 0 }, Package() { 0x0005ffff, 3, LNKD, 0 },
                Package() { 0x0006ffff, 0, LNKB, 0 }, Package() { 0x0006ffff, 1, LNKC, 0 }, Package() { 0x0006ffff, 2, LNKD, 0 }, Package() { 0x0006ffff, 3, LNKA, 0 },
                Package() { 0x0007ffff, 0, LNKC, 0 }, Package() { 0x0007ffff, 1, LNKD, 0 }, Package() { 0x0007ffff, 2, LNKA, 0 }, Package() { 0x0007ffff, 3, LNKB, 0 },
                Package() { 0x0008ffff, 0, LNKD, 0 }, Package() { 0x0008ffff, 1, LNKA, 0 }, Package() { 0x0008ffff, 2, LNKB, 0 }, Package() { 0x0008ffff, 3, LNKC, 0 },
                Package() { 0x0009ffff, 0, LNKA, 0 }, Package() { 0x0009ffff, 1, LNKB, 0 }, Package() { 0x0009ffff, 2, LNKC, 0 }, Package() { 0x0009ffff, 3, LNKD, 0 },
                Package() { 0x000affff, 0, LNKB, 0 }, Package() { 0x000affff, 1, LNKC, 0 }, Package() { 0x000affff, 2, LNKD, 0 }, Package() { 0x000affff, 3, LNKA, 0 },
                Package() { 0x000bffff, 0, LNKC, 0 }, Package() { 0x000bffff, 1, LNKD, 0 }, Package() { 0x000bffff, 2, LNKA, 0 }, Package() { 0x000bffff, 3, LNKB, 0 },
                Package() { 0x000cffff, 0, LNKD, 0 }, Package() { 0x000cffff, 1, LNKA, 0 }, Package() { 0x000cffff, 2, LNKB, 0 }, Package() { 0x000cffff, 3, LNKC, 0 },
                Package() { 0x000dffff, 0, LNKA, 0 }, Package() { 0x000dffff, 1, LNKB, 0 }, Package() { 0x000dffff, 2, LNKC, 0 }, Package() { 0x000dffff, 3, LNKD, 0 },
                Package() { 0x000effff, 0, LNKB, 0 }, Package() { 0x000effff, 1, LNKC, 0 }, Package() { 0x000effff, 2, LNKD, 0 }, Package() { 0x000effff, 3, LNKA, 0 },
                Package() { 0x000fffff, 0, LNKC, 0 }, Package() { 0x000fffff, 1, LNKD, 0 }, Package() { 0x000fffff, 2, LNKA, 0 }, Package() { 0x000fffff, 3, LNKB, 0 },
                Package() { 0x0010ffff, 0, LNKD, 0 }, Package() { 0x0010ffff, 1, LNKA, 0 }, Package() { 0x0010ffff, 2, LNKB, 0 }, Package() { 0x0010ffff, 3, LNKC, 0 },
                Package() { 0x0011ffff, 0, LNKA, 0 }, Package() { 0x0011ffff, 1, LNKB, 0 }, Package() { 0x0011ffff, 2, LNKC, 0 }, Package() { 0x0011ffff, 3, LNKD, 0 },
                Package() { 0x0012ffff, 0, LNKB, 0 }, Package() { 0x0012ffff, 1, LNKC, 0 }, Package() { 0x0012ffff, 2, LNKD, 0 }, Package() { 0x0012ffff, 3, LNKA, 0 },
                Package() { 0x0013ffff, 0, LNKC, 0 }, Package() { 0x0013ffff, 1, LNKD, 0 }, Package() { 0x0013ffff, 2, LNKA, 0 }, Package() { 0x0013ffff, 3, LNKB, 0 },
                Package() { 0x0014ffff, 0, LNKD, 0 }, Package() { 0x0014ffff, 1, LNKA, 0 }, Package() { 0x0014ffff, 2, LNKB, 0 }, Package() { 0x0014ffff, 3, LNKC, 0 },
                Package() { 0x0015ffff, 0, LNKA, 0 }, Package() { 0x0015ffff, 1, LNKB, 0 }, Package() { 0x0015ffff, 2, LNKC, 0 }, Package() { 0x0015ffff, 3, LNKD, 0 },
                Package() { 0x0016ffff, 0, LNKB, 0 }, Package() { 0x0016ffff, 1, LNKC, 0 }, Package() { 0x0016ffff, 2, LNKD, 0 }, Package() { 0x0016ffff, 3, LNKA, 0 },
                Package() { 0x0017ffff, 0, LNKC, 0 }, Package() { 0x0017ffff, 1, LNKD, 0 }, Package() { 0x0017ffff, 2, LNKA, 0 }, Package() { 0x0017ffff, 3, LNKB, 0 },
                Package() { 0x0018ffff, 0, LNKD, 0 }, Package() { 0x0018ffff, 1, LNKA, 0 }, Package() { 0x0018ffff, 2, LNKB, 0 }, Package() { 0x0018ffff, 3, LNKC, 0 },
                Package() { 0x0019ffff, 0, LNKA, 0 }, Package() { 0x0019ffff, 1, LNKB, 0 }, Package() { 0x0019ffff, 2, LNKC, 0 }, Package() { 0x0019ffff, 3, LNKD, 0 },
                Package() { 0x001affff, 0, LNKB, 0 }, Package() { 0x001affff, 1, LNKC, 0 }, Package() { 0x001affff, 2, LNKD, 0 }, Package() { 0x001affff, 3, LNKA, 0 },
                Package() { 0x001bffff, 0, LNKC, 0 }, Package() { 0x001bffff, 1, LNKD, 0 }, Package() { 0x001bffff, 2, LNKA, 0 }, Package() { 0x001bffff, 3, LNKB, 0 },
                Package() { 0x001cffff, 0, LNKD, 0 }, Package() { 0x001cffff, 1, LNKA, 0 }, Package() { 0x001cffff, 2, LNKB, 0 }, Package() { 0x001cffff, 3, LNKC, 0 },
                Package() { 0x001dffff, 0, LNKA, 0 }, Package() { 0x001dffff, 1, LNKB, 0 }, Package() { 0x001dffff, 2, LNKC, 0 }, Package() { 0x001dffff, 3, LNKD, 0 },
                Package() { 0x001effff, 0, LNKB, 0 }, Package() { 0x001effff, 1, LNKC, 0 }, Package() { 0x001effff, 2, LNKD, 0 }, Package() { 0x001effff, 3, LNKA, 0 },
                Package() { 0x001fffff, 0, LNKC, 0 }, Package() { 0x001fffff, 1, LNKD, 0 }, Package() { 0x001fffff, 2, LNKA, 0 }, Package() { 0x001fffff, 3, LNKB, 0 },
            })
        }

        Field(PCI0.ISA.P40C, ByteAcc, NoLock, Preserve) {
            PRQ0, 8,
            PRQ1, 8,
            PRQ2, 8,
            PRQ3, 8
        }

        Method(IQST, 1, NotSerialized) {
            If (And(0x80, Arg0)) {
                Return (0x09)
            }
            Return (0x0B)
        }

        Method(IQCR, 1, Serialized) {
            Name(PRR0, ResourceTemplate() {
                Interrupt(, Level, ActiveHigh, Shared) { 0 }
            })
            CreateDWordField(PRR0, 0x05, PRRI)
            If (LLess(Arg0, 0x80)) {
                Store(Arg0, PRRI)
            }
            Return (PRR0)
        }

        Device(LNKB) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 1)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 5, 10, 11 } })
            Method(_STA, 0, NotSerialized) { Return (IQST(PRQ1)) }
            Method(_DIS, 0, NotSerialized) { Or(PRQ1, 0x80, PRQ1) }
            Method(_CRS, 0, NotSerialized) { Return (IQCR(PRQ1)) }
            Method(_SRS, 1, NotSerialized) { CreateDWordField(Arg0, 0x05, PRRI) Store(PRRI, PRQ1) }
        }
        Device(LNKA) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 0)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 5, 10, 11 } })
            Method(_STA, 0, NotSerialized) { Return (IQST(PRQ0)) }
            Method(_DIS, 0, NotSerialized) { Or(PRQ0, 0x80, PRQ0) }
            Method(_CRS, 0, NotSerialized) { Return (IQCR(PRQ0)) }
            Method(_SRS, 1, NotSerialized) { CreateDWordField(Arg0, 0x05, PRRI) Store(PRRI, PRQ0) }
        }
        Device(LNKC) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 2)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 5, 10, 11 } })
            Method(_STA, 0, NotSerialized) { Return (IQST(PRQ2)) }
            Method(_DIS, 0, NotSerialized) { Or(PRQ2, 0x80, PRQ2) }
            Method(_CRS, 0, NotSerialized) { Return (IQCR(PRQ2)) }
            Method(_SRS, 1, NotSerialized) { CreateDWordField(Arg0, 0x05, PRRI) Store(PRRI, PRQ2) }
        }
        Device(LNKD) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 3)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 5, 10, 11 } })
            Method(_STA, 0, NotSerialized) { Return (IQST(PRQ3)) }
            Method(_DIS, 0, NotSerialized) { Or(PRQ3, 0x80, PRQ3) }
            Method(_CRS, 0, NotSerialized) { Return (IQCR(PRQ3)) }
            Method(_SRS, 1, NotSerialized) { CreateDWordField(Arg0, 0x05, PRRI) Store(PRRI, PRQ3) }
        }
        Device(LNKS) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 4)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 9 } })
            Method(_STA, 0, NotSerialized) { Return (0x0b) }
            Method(_DIS, 0, NotSerialized) { }
            Method(_CRS, 0, NotSerialized) { Return (_PRS) }
            Method(_SRS, 1, NotSerialized) { }
        }
    }
}
