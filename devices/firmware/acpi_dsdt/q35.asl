DefinitionBlock(
    "q35.aml",      // Output file
    "DSDT",         // Table signature
    1,              // DSDT revision
    "TENCLS",       // OEM ID
    "MVPC",         // Table ID
    1               // OEM revision
) {

    // Device object for the PCI root bus
    Scope(\_SB) {
        Device(PCI0) {
            Name(_HID, EisaId("PNP0A08"))
            Name(_CID, EisaId("PNP0A03"))
            Name(_UID, 0x00)
        }
    }

    // PCI root bus resource template
    Scope(\_SB.PCI0) {
        Name (_CRS, ResourceTemplate ()  // _CRS: Current Resource Settings
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
                0x80000000,         // Range Minimum
                0xDFFFFFFF,         // Range Maximum
                0x00000000,         // Translation Offset
                0x60000000,         // Length
                ,, , AddressRangeMemory, TypeStatic)
            DWordMemory (ResourceProducer, PosDecode, MinFixed, MaxFixed, NonCacheable, ReadWrite,
                0x00000000,         // Granularity
                0xF0000000,         // Range Minimum
                0xFEBFFFFF,         // Range Maximum
                0x00000000,         // Translation Offset
                0x0EC00000,         // Length
                ,, , AddressRangeMemory, TypeStatic)
            QWordMemory (ResourceProducer, PosDecode, MinFixed, MaxFixed, Cacheable, ReadWrite,
                0x0000000000000000, // Granularity
                0x0000380000000000, // Range Minimum
                0x00003807FFFFFFFF, // Range Maximum
                0x0000000000000000, // Translation Offset
                0x0000000800000000, // Length
                ,, , AddressRangeMemory, TypeStatic)
        })
    }

    // PCI D31:F0 LPC ISA bridge
    Scope(\_SB.PCI0) {
        Device(ISA) {
            Name(_ADR, 0x001F0000)

            OperationRegion(PIRQ, PCI_Config, 0x60, 0x0C)
            OperationRegion(LPCD, PCI_Config, 0x80, 0x02)
            Field(LPCD, AnyAcc, NoLock, Preserve) {
                COMA, 3,, 1,
                COMB, 3,, 1,
                LPTD, 2,, 2,
                FDCD, 2,, 2
            }
            OperationRegion(LPCE, PCI_Config, 0x82, 0x02)
            Field(LPCE, AnyAcc, NoLock, Preserve) {
                CAEN, 1,
                CBEN, 1,
                LPEN, 1,
                FDEN, 1
            }
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


    // Zero => PIC mode, One => APIC Mode
    Name(\PICF, Zero)
    Method(\_PIC, 1, NotSerialized) {
        Store(Arg0, \PICF)
    }

    // PCI IRQ routing
    Scope(\_SB) {
        Scope(PCI0) {
            Name(PRTP, package() {
                Package() { 0x0000ffff, 0, LNKE, 0 }, Package() { 0x0000ffff, 1, LNKF, 0 }, Package() { 0x0000ffff, 2, LNKG, 0 }, Package() { 0x0000ffff, 3, LNKH, 0 },
                Package() { 0x0001ffff, 0, LNKF, 0 }, Package() { 0x0001ffff, 1, LNKG, 0 }, Package() { 0x0001ffff, 2, LNKH, 0 }, Package() { 0x0001ffff, 3, LNKE, 0 },
                Package() { 0x0002ffff, 0, LNKG, 0 }, Package() { 0x0002ffff, 1, LNKH, 0 }, Package() { 0x0002ffff, 2, LNKE, 0 }, Package() { 0x0002ffff, 3, LNKF, 0 },
                Package() { 0x0003ffff, 0, LNKH, 0 }, Package() { 0x0003ffff, 1, LNKE, 0 }, Package() { 0x0003ffff, 2, LNKF, 0 }, Package() { 0x0003ffff, 3, LNKG, 0 },
                Package() { 0x0004ffff, 0, LNKE, 0 }, Package() { 0x0004ffff, 1, LNKF, 0 }, Package() { 0x0004ffff, 2, LNKG, 0 }, Package() { 0x0004ffff, 3, LNKH, 0 },
                Package() { 0x0005ffff, 0, LNKF, 0 }, Package() { 0x0005ffff, 1, LNKG, 0 }, Package() { 0x0005ffff, 2, LNKH, 0 }, Package() { 0x0005ffff, 3, LNKE, 0 },
                Package() { 0x0006ffff, 0, LNKG, 0 }, Package() { 0x0006ffff, 1, LNKH, 0 }, Package() { 0x0006ffff, 2, LNKE, 0 }, Package() { 0x0006ffff, 3, LNKF, 0 },
                Package() { 0x0007ffff, 0, LNKH, 0 }, Package() { 0x0007ffff, 1, LNKE, 0 }, Package() { 0x0007ffff, 2, LNKF, 0 }, Package() { 0x0007ffff, 3, LNKG, 0 },
                Package() { 0x0008ffff, 0, LNKE, 0 }, Package() { 0x0008ffff, 1, LNKF, 0 }, Package() { 0x0008ffff, 2, LNKG, 0 }, Package() { 0x0008ffff, 3, LNKH, 0 },
                Package() { 0x0009ffff, 0, LNKF, 0 }, Package() { 0x0009ffff, 1, LNKG, 0 }, Package() { 0x0009ffff, 2, LNKH, 0 }, Package() { 0x0009ffff, 3, LNKE, 0 },
                Package() { 0x000affff, 0, LNKG, 0 }, Package() { 0x000affff, 1, LNKH, 0 }, Package() { 0x000affff, 2, LNKE, 0 }, Package() { 0x000affff, 3, LNKF, 0 },
                Package() { 0x000bffff, 0, LNKH, 0 }, Package() { 0x000bffff, 1, LNKE, 0 }, Package() { 0x000bffff, 2, LNKF, 0 }, Package() { 0x000bffff, 3, LNKG, 0 },
                Package() { 0x000cffff, 0, LNKE, 0 }, Package() { 0x000cffff, 1, LNKF, 0 }, Package() { 0x000cffff, 2, LNKG, 0 }, Package() { 0x000cffff, 3, LNKH, 0 },
                Package() { 0x000dffff, 0, LNKF, 0 }, Package() { 0x000dffff, 1, LNKG, 0 }, Package() { 0x000dffff, 2, LNKH, 0 }, Package() { 0x000dffff, 3, LNKE, 0 },
                Package() { 0x000effff, 0, LNKG, 0 }, Package() { 0x000effff, 1, LNKH, 0 }, Package() { 0x000effff, 2, LNKE, 0 }, Package() { 0x000effff, 3, LNKF, 0 },
                Package() { 0x000fffff, 0, LNKH, 0 }, Package() { 0x000fffff, 1, LNKE, 0 }, Package() { 0x000fffff, 2, LNKF, 0 }, Package() { 0x000fffff, 3, LNKG, 0 },
                Package() { 0x0010ffff, 0, LNKE, 0 }, Package() { 0x0010ffff, 1, LNKF, 0 }, Package() { 0x0010ffff, 2, LNKG, 0 }, Package() { 0x0010ffff, 3, LNKH, 0 },
                Package() { 0x0011ffff, 0, LNKF, 0 }, Package() { 0x0011ffff, 1, LNKG, 0 }, Package() { 0x0011ffff, 2, LNKH, 0 }, Package() { 0x0011ffff, 3, LNKE, 0 },
                Package() { 0x0012ffff, 0, LNKG, 0 }, Package() { 0x0012ffff, 1, LNKH, 0 }, Package() { 0x0012ffff, 2, LNKE, 0 }, Package() { 0x0012ffff, 3, LNKF, 0 },
                Package() { 0x0013ffff, 0, LNKH, 0 }, Package() { 0x0013ffff, 1, LNKE, 0 }, Package() { 0x0013ffff, 2, LNKF, 0 }, Package() { 0x0013ffff, 3, LNKG, 0 },
                Package() { 0x0014ffff, 0, LNKE, 0 }, Package() { 0x0014ffff, 1, LNKF, 0 }, Package() { 0x0014ffff, 2, LNKG, 0 }, Package() { 0x0014ffff, 3, LNKH, 0 },
                Package() { 0x0015ffff, 0, LNKF, 0 }, Package() { 0x0015ffff, 1, LNKG, 0 }, Package() { 0x0015ffff, 2, LNKH, 0 }, Package() { 0x0015ffff, 3, LNKE, 0 },
                Package() { 0x0016ffff, 0, LNKG, 0 }, Package() { 0x0016ffff, 1, LNKH, 0 }, Package() { 0x0016ffff, 2, LNKE, 0 }, Package() { 0x0016ffff, 3, LNKF, 0 },
                Package() { 0x0017ffff, 0, LNKH, 0 }, Package() { 0x0017ffff, 1, LNKE, 0 }, Package() { 0x0017ffff, 2, LNKF, 0 }, Package() { 0x0017ffff, 3, LNKG, 0 },
                Package() { 0x0018ffff, 0, LNKE, 0 }, Package() { 0x0018ffff, 1, LNKF, 0 }, Package() { 0x0018ffff, 2, LNKG, 0 }, Package() { 0x0018ffff, 3, LNKH, 0 },
                Package() { 0x0019ffff, 0, LNKA, 0 }, Package() { 0x0019ffff, 1, LNKB, 0 }, Package() { 0x0019ffff, 2, LNKC, 0 }, Package() { 0x0019ffff, 3, LNKD, 0 },
                Package() { 0x001affff, 0, LNKA, 0 }, Package() { 0x001affff, 1, LNKB, 0 }, Package() { 0x001affff, 2, LNKC, 0 }, Package() { 0x001affff, 3, LNKD, 0 },
                Package() { 0x001bffff, 0, LNKA, 0 }, Package() { 0x001bffff, 1, LNKB, 0 }, Package() { 0x001bffff, 2, LNKC, 0 }, Package() { 0x001bffff, 3, LNKD, 0 },
                Package() { 0x001cffff, 0, LNKA, 0 }, Package() { 0x001cffff, 1, LNKB, 0 }, Package() { 0x001cffff, 2, LNKC, 0 }, Package() { 0x001cffff, 3, LNKD, 0 },
                Package() { 0x001dffff, 0, LNKA, 0 }, Package() { 0x001dffff, 1, LNKB, 0 }, Package() { 0x001dffff, 2, LNKC, 0 }, Package() { 0x001dffff, 3, LNKD, 0 },
                Package() { 0x001effff, 0, LNKE, 0 }, Package() { 0x001effff, 1, LNKF, 0 }, Package() { 0x001effff, 2, LNKG, 0 }, Package() { 0x001effff, 3, LNKH, 0 },
                Package() { 0x001fffff, 0, LNKA, 0 }, Package() { 0x001fffff, 1, LNKB, 0 }, Package() { 0x001fffff, 2, LNKC, 0 }, Package() { 0x001fffff, 3, LNKD, 0 }
            })
            Name(PRTA, package() {
                Package() { 0x0000ffff, 0, GSIE, 0 }, Package() { 0x0000ffff, 1, GSIF, 0 }, Package() { 0x0000ffff, 2, GSIG, 0 }, Package() { 0x0000ffff, 3, GSIH, 0 },
                Package() { 0x0001ffff, 0, GSIF, 0 }, Package() { 0x0001ffff, 1, GSIG, 0 }, Package() { 0x0001ffff, 2, GSIH, 0 }, Package() { 0x0001ffff, 3, GSIE, 0 },
                Package() { 0x0002ffff, 0, GSIG, 0 }, Package() { 0x0002ffff, 1, GSIH, 0 }, Package() { 0x0002ffff, 2, GSIE, 0 }, Package() { 0x0002ffff, 3, GSIF, 0 },
                Package() { 0x0003ffff, 0, GSIH, 0 }, Package() { 0x0003ffff, 1, GSIE, 0 }, Package() { 0x0003ffff, 2, GSIF, 0 }, Package() { 0x0003ffff, 3, GSIG, 0 },
                Package() { 0x0004ffff, 0, GSIE, 0 }, Package() { 0x0004ffff, 1, GSIF, 0 }, Package() { 0x0004ffff, 2, GSIG, 0 }, Package() { 0x0004ffff, 3, GSIH, 0 },
                Package() { 0x0005ffff, 0, GSIF, 0 }, Package() { 0x0005ffff, 1, GSIG, 0 }, Package() { 0x0005ffff, 2, GSIH, 0 }, Package() { 0x0005ffff, 3, GSIE, 0 },
                Package() { 0x0006ffff, 0, GSIG, 0 }, Package() { 0x0006ffff, 1, GSIH, 0 }, Package() { 0x0006ffff, 2, GSIE, 0 }, Package() { 0x0006ffff, 3, GSIF, 0 },
                Package() { 0x0007ffff, 0, GSIH, 0 }, Package() { 0x0007ffff, 1, GSIE, 0 }, Package() { 0x0007ffff, 2, GSIF, 0 }, Package() { 0x0007ffff, 3, GSIG, 0 },
                Package() { 0x0008ffff, 0, GSIE, 0 }, Package() { 0x0008ffff, 1, GSIF, 0 }, Package() { 0x0008ffff, 2, GSIG, 0 }, Package() { 0x0008ffff, 3, GSIH, 0 },
                Package() { 0x0009ffff, 0, GSIF, 0 }, Package() { 0x0009ffff, 1, GSIG, 0 }, Package() { 0x0009ffff, 2, GSIH, 0 }, Package() { 0x0009ffff, 3, GSIE, 0 },
                Package() { 0x000affff, 0, GSIG, 0 }, Package() { 0x000affff, 1, GSIH, 0 }, Package() { 0x000affff, 2, GSIE, 0 }, Package() { 0x000affff, 3, GSIF, 0 },
                Package() { 0x000bffff, 0, GSIH, 0 }, Package() { 0x000bffff, 1, GSIE, 0 }, Package() { 0x000bffff, 2, GSIF, 0 }, Package() { 0x000bffff, 3, GSIG, 0 },
                Package() { 0x000cffff, 0, GSIE, 0 }, Package() { 0x000cffff, 1, GSIF, 0 }, Package() { 0x000cffff, 2, GSIG, 0 }, Package() { 0x000cffff, 3, GSIH, 0 },
                Package() { 0x000dffff, 0, GSIF, 0 }, Package() { 0x000dffff, 1, GSIG, 0 }, Package() { 0x000dffff, 2, GSIH, 0 }, Package() { 0x000dffff, 3, GSIE, 0 },
                Package() { 0x000effff, 0, GSIG, 0 }, Package() { 0x000effff, 1, GSIH, 0 }, Package() { 0x000effff, 2, GSIE, 0 }, Package() { 0x000effff, 3, GSIF, 0 },
                Package() { 0x000fffff, 0, GSIH, 0 }, Package() { 0x000fffff, 1, GSIE, 0 }, Package() { 0x000fffff, 2, GSIF, 0 }, Package() { 0x000fffff, 3, GSIG, 0 },
                Package() { 0x0010ffff, 0, GSIE, 0 }, Package() { 0x0010ffff, 1, GSIF, 0 }, Package() { 0x0010ffff, 2, GSIG, 0 }, Package() { 0x0010ffff, 3, GSIH, 0 },
                Package() { 0x0011ffff, 0, GSIF, 0 }, Package() { 0x0011ffff, 1, GSIG, 0 }, Package() { 0x0011ffff, 2, GSIH, 0 }, Package() { 0x0011ffff, 3, GSIE, 0 },
                Package() { 0x0012ffff, 0, GSIG, 0 }, Package() { 0x0012ffff, 1, GSIH, 0 }, Package() { 0x0012ffff, 2, GSIE, 0 }, Package() { 0x0012ffff, 3, GSIF, 0 },
                Package() { 0x0013ffff, 0, GSIH, 0 }, Package() { 0x0013ffff, 1, GSIE, 0 }, Package() { 0x0013ffff, 2, GSIF, 0 }, Package() { 0x0013ffff, 3, GSIG, 0 },
                Package() { 0x0014ffff, 0, GSIE, 0 }, Package() { 0x0014ffff, 1, GSIF, 0 }, Package() { 0x0014ffff, 2, GSIG, 0 }, Package() { 0x0014ffff, 3, GSIH, 0 },
                Package() { 0x0015ffff, 0, GSIF, 0 }, Package() { 0x0015ffff, 1, GSIG, 0 }, Package() { 0x0015ffff, 2, GSIH, 0 }, Package() { 0x0015ffff, 3, GSIE, 0 },
                Package() { 0x0016ffff, 0, GSIG, 0 }, Package() { 0x0016ffff, 1, GSIH, 0 }, Package() { 0x0016ffff, 2, GSIE, 0 }, Package() { 0x0016ffff, 3, GSIF, 0 },
                Package() { 0x0017ffff, 0, GSIH, 0 }, Package() { 0x0017ffff, 1, GSIE, 0 }, Package() { 0x0017ffff, 2, GSIF, 0 }, Package() { 0x0017ffff, 3, GSIG, 0 },
                Package() { 0x0018ffff, 0, GSIE, 0 }, Package() { 0x0018ffff, 1, GSIF, 0 }, Package() { 0x0018ffff, 2, GSIG, 0 }, Package() { 0x0018ffff, 3, GSIH, 0 },
                Package() { 0x0019ffff, 0, GSIA, 0 }, Package() { 0x0019ffff, 1, GSIB, 0 }, Package() { 0x0019ffff, 2, GSIC, 0 }, Package() { 0x0019ffff, 3, GSID, 0 },
                Package() { 0x001affff, 0, GSIA, 0 }, Package() { 0x001affff, 1, GSIB, 0 }, Package() { 0x001affff, 2, GSIC, 0 }, Package() { 0x001affff, 3, GSID, 0 },
                Package() { 0x001bffff, 0, GSIA, 0 }, Package() { 0x001bffff, 1, GSIB, 0 }, Package() { 0x001bffff, 2, GSIC, 0 }, Package() { 0x001bffff, 3, GSID, 0 },
                Package() { 0x001cffff, 0, GSIA, 0 }, Package() { 0x001cffff, 1, GSIB, 0 }, Package() { 0x001cffff, 2, GSIC, 0 }, Package() { 0x001cffff, 3, GSID, 0 },
                Package() { 0x001dffff, 0, GSIA, 0 }, Package() { 0x001dffff, 1, GSIB, 0 }, Package() { 0x001dffff, 2, GSIC, 0 }, Package() { 0x001dffff, 3, GSID, 0 },
                Package() { 0x001effff, 0, GSIE, 0 }, Package() { 0x001effff, 1, GSIF, 0 }, Package() { 0x001effff, 2, GSIG, 0 }, Package() { 0x001effff, 3, GSIH, 0 },
                Package() { 0x001fffff, 0, GSIA, 0 }, Package() { 0x001fffff, 1, GSIB, 0 }, Package() { 0x001fffff, 2, GSIC, 0 }, Package() { 0x001fffff, 3, GSID, 0 }
            })
            Method(_PRT, 0, NotSerialized) {
                If (LEqual(\PICF, Zero)) {
                    Return (PRTP)
                } Else {
                    Return (PRTA)
                }
            }
        }

        Field(PCI0.ISA.PIRQ, ByteAcc, NoLock, Preserve) {
            PRQA, 8,
            PRQB, 8,
            PRQC, 8,
            PRQD, 8,
            Offset(0x08),
            PRQE, 8,
            PRQF, 8,
            PRQG, 8,
            PRQH, 8
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
            Store(And(Arg0, 0x0F), PRRI)
            Return (PRR0)
        }
        
        Device(LNKA) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 0)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 5, 10, 11 } })
            Method(_STA, 0, NotSerialized) { Return (IQST(PRQA)) }
            Method(_DIS, 0, NotSerialized) { Or(PRQA, 0x80, PRQA) }
            Method(_CRS, 0, NotSerialized) { Return (IQCR(PRQA)) }
            Method(_SRS, 1, NotSerialized) { CreateDWordField(Arg0, 0x05, PRRI) Store(PRRI, PRQA) }
        }
        Device(LNKB) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 1)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 5, 10, 11 } })
            Method(_STA, 0, NotSerialized) { Return (IQST(PRQB)) }
            Method(_DIS, 0, NotSerialized) { Or(PRQB, 0x80, PRQB) }
            Method(_CRS, 0, NotSerialized) { Return (IQCR(PRQB)) }
            Method(_SRS, 1, NotSerialized) { CreateDWordField(Arg0, 0x05, PRRI) Store(PRRI, PRQB) }
        }
        Device(LNKC) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 2)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 5, 10, 11 } })
            Method(_STA, 0, NotSerialized) { Return (IQST(PRQC)) }
            Method(_DIS, 0, NotSerialized) { Or(PRQC, 0x80, PRQC) }
            Method(_CRS, 0, NotSerialized) { Return (IQCR(PRQC)) }
            Method(_SRS, 1, NotSerialized) { CreateDWordField(Arg0, 0x05, PRRI) Store(PRRI, PRQC) }
        }
        Device(LNKD) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 3)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 5, 10, 11 } })
            Method(_STA, 0, NotSerialized) { Return (IQST(PRQD)) }
            Method(_DIS, 0, NotSerialized) { Or(PRQD, 0x80, PRQD) }
            Method(_CRS, 0, NotSerialized) { Return (IQCR(PRQD)) }
            Method(_SRS, 1, NotSerialized) { CreateDWordField(Arg0, 0x05, PRRI) Store(PRRI, PRQD) }
        }
        Device(LNKE) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 4)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 5, 10, 11 } })
            Method(_STA, 0, NotSerialized) { Return (IQST(PRQE)) }
            Method(_DIS, 0, NotSerialized) { Or(PRQE, 0x80, PRQE) }
            Method(_CRS, 0, NotSerialized) { Return (IQCR(PRQE)) }
            Method(_SRS, 1, NotSerialized) { CreateDWordField(Arg0, 0x05, PRRI) Store(PRRI, PRQE) }
        }
        Device(LNKF) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 5)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 5, 10, 11 } })
            Method(_STA, 0, NotSerialized) { Return (IQST(PRQF)) }
            Method(_DIS, 0, NotSerialized) { Or(PRQF, 0x80, PRQF) }
            Method(_CRS, 0, NotSerialized) { Return (IQCR(PRQF)) }
            Method(_SRS, 1, NotSerialized) { CreateDWordField(Arg0, 0x05, PRRI) Store(PRRI, PRQF) }
        }
        Device(LNKG) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 6)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 5, 10, 11 } })
            Method(_STA, 0, NotSerialized) { Return (IQST(PRQG)) }
            Method(_DIS, 0, NotSerialized) { Or(PRQG, 0x80, PRQG) }
            Method(_CRS, 0, NotSerialized) { Return (IQCR(PRQG)) }
            Method(_SRS, 1, NotSerialized) { CreateDWordField(Arg0, 0x05, PRRI) Store(PRRI, PRQG) }
        }
        Device(LNKH) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 7)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 5, 10, 11 } })
            Method(_STA, 0, NotSerialized) { Return (IQST(PRQH)) }
            Method(_DIS, 0, NotSerialized) { Or(PRQH, 0x80, PRQH) }
            Method(_CRS, 0, NotSerialized) { Return (IQCR(PRQH)) }
            Method(_SRS, 1, NotSerialized) { CreateDWordField(Arg0, 0x05, PRRI) Store(PRRI, PRQH) }
        }
        Device(GSIA) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 0)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 0x10 } })
            Name(_CRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 0x10 } })
            Method(_SRS, 1, NotSerialized) { }
        }
        Device(GSIB) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 0)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 0x11 } })
            Name(_CRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 0x11 } })
            Method(_SRS, 1, NotSerialized) { }
        }
        Device(GSIC) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 0)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 0x12 } })
            Name(_CRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 0x12 } })
            Method(_SRS, 1, NotSerialized) { }
        }
        Device(GSID) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 0)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 0x13 } })
            Name(_CRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 0x13 } })
            Method(_SRS, 1, NotSerialized) { }
        }
        Device(GSIE) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 0)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 0x14 } })
            Name(_CRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 0x14 } })
            Method(_SRS, 1, NotSerialized) { }
        }
        Device(GSIF) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 0)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 0x15 } })
            Name(_CRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 0x15 } })
            Method(_SRS, 1, NotSerialized) { }
        }
        Device(GSIG) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 0)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 0x16 } })
            Name(_CRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 0x16 } })
            Method(_SRS, 1, NotSerialized) { }
        }
        Device(GSIH) {
            Name(_HID, EISAID("PNP0C0F"))
            Name(_UID, 0)
            Name(_PRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 0x17 } })
            Name(_CRS, ResourceTemplate() { Interrupt(, Level, ActiveHigh, Shared) { 0x17 } })
            Method(_SRS, 1, NotSerialized) { }
        }
    }
}
