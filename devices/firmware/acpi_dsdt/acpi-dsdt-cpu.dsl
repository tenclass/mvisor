
Scope (_SB)
{
    Device (\_SB.PCI0.PRES)
    {
        Name (_HID, EisaId ("PNP0A06") /* Generic Container Device */)  // _HID: Hardware ID
        Name (_UID, "CPU Hotplug resources")  // _UID: Unique ID
        Mutex (CPLK, 0x00)
        Name (_CRS, ResourceTemplate ()  // _CRS: Current Resource Settings
        {
            IO (Decode16,
                0x0CD8,             // Range Minimum
                0x0CD8,             // Range Maximum
                0x01,               // Alignment
                0x0C,               // Length
                )
        })
        OperationRegion (PRST, SystemIO, 0x0CD8, 0x0C)
        Field (PRST, ByteAcc, NoLock, WriteAsZeros)
        {
            Offset (0x04), 
            CPEN,   1, 
            CINS,   1, 
            CRMV,   1, 
            CEJ0,   1, 
            CEJF,   1, 
            Offset (0x05), 
            CCMD,   8
        }

        Field (PRST, DWordAcc, NoLock, Preserve)
        {
            CSEL,   32, 
            Offset (0x08), 
            CDAT,   32
        }

        Method (_INI, 0, Serialized)  // _INI: Initialize
        {
            CSEL = Zero
        }
    }

    Device (\_SB.CPUS)
    {
        Name (_HID, "ACPI0010" /* Processor Container Device */)  // _HID: Hardware ID
        Name (_CID, EisaId ("PNP0A05") /* Generic Container Device */)  // _CID: Compatible ID
        Method (CTFY, 2, NotSerialized)
        {
            If ((Arg0 == Zero))
            {
                Notify (C000, Arg1)
            }

            If ((Arg0 == One))
            {
                Notify (C001, Arg1)
            }

            If ((Arg0 == 0x02))
            {
                Notify (C002, Arg1)
            }

            If ((Arg0 == 0x03))
            {
                Notify (C003, Arg1)
            }

            If ((Arg0 == 0x04))
            {
                Notify (C004, Arg1)
            }

            If ((Arg0 == 0x05))
            {
                Notify (C005, Arg1)
            }

            If ((Arg0 == 0x06))
            {
                Notify (C006, Arg1)
            }

            If ((Arg0 == 0x07))
            {
                Notify (C007, Arg1)
            }
        }

        Method (CSTA, 1, Serialized)
        {
            Acquire (\_SB.PCI0.PRES.CPLK, 0xFFFF)
            \_SB.PCI0.PRES.CSEL = Arg0
            Local0 = Zero
            If ((\_SB.PCI0.PRES.CPEN == One))
            {
                Local0 = 0x0F
            }

            Release (\_SB.PCI0.PRES.CPLK)
            Return (Local0)
        }

        Method (CEJ0, 1, Serialized)
        {
            Acquire (\_SB.PCI0.PRES.CPLK, 0xFFFF)
            \_SB.PCI0.PRES.CSEL = Arg0
            \_SB.PCI0.PRES.CEJ0 = One
            Release (\_SB.PCI0.PRES.CPLK)
        }

        Method (CSCN, 0, Serialized)
        {
            Acquire (\_SB.PCI0.PRES.CPLK, 0xFFFF)
            Name (CNEW, Package (0xFF) {})
            Local3 = Zero
            Local4 = One
            While ((Local4 == One))
            {
                Local4 = Zero
                Local0 = One
                Local1 = Zero
                While (((Local0 == One) && (Local3 < 0x08)))
                {
                    Local0 = Zero
                    \_SB.PCI0.PRES.CSEL = Local3
                    \_SB.PCI0.PRES.CCMD = Zero
                    If ((\_SB.PCI0.PRES.CDAT < Local3))
                    {
                        Break
                    }

                    If ((Local1 == 0xFF))
                    {
                        Local4 = One
                        Break
                    }

                    Local3 = \_SB.PCI0.PRES.CDAT
                    If ((\_SB.PCI0.PRES.CINS == One))
                    {
                        CNEW [Local1] = Local3
                        Local1++
                        Local0 = One
                    }
                    ElseIf ((\_SB.PCI0.PRES.CRMV == One))
                    {
                        CTFY (Local3, 0x03)
                        \_SB.PCI0.PRES.CRMV = One
                        Local0 = One
                    }

                    Local3++
                }

                Local2 = Zero
                While ((Local2 < Local1))
                {
                    Local3 = DerefOf (CNEW [Local2])
                    CTFY (Local3, One)
                    Debug = Local3
                    \_SB.PCI0.PRES.CSEL = Local3
                    \_SB.PCI0.PRES.CINS = One
                    Local2++
                }
            }

            Release (\_SB.PCI0.PRES.CPLK)
        }

        Method (COST, 4, Serialized)
        {
            Acquire (\_SB.PCI0.PRES.CPLK, 0xFFFF)
            \_SB.PCI0.PRES.CSEL = Arg0
            \_SB.PCI0.PRES.CCMD = One
            \_SB.PCI0.PRES.CDAT = Arg1
            \_SB.PCI0.PRES.CCMD = 0x02
            \_SB.PCI0.PRES.CDAT = Arg2
            Release (\_SB.PCI0.PRES.CPLK)
        }

        Processor (C000, 0x00, 0x00000000, 0x00)
        {
            Method (_STA, 0, Serialized)  // _STA: Status
            {
                Return (CSTA (Zero))
            }

            Name (_MAT, Buffer (0x08)  // _MAT: Multiple APIC Table Entry
            {
                  0x00, 0x08, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00   // ........
            })
            Method (_OST, 3, Serialized)  // _OST: OSPM Status Indication
            {
                COST (Zero, Arg0, Arg1, Arg2)
            }
        }

        Processor (C001, 0x01, 0x00000000, 0x00)
        {
            Method (_STA, 0, Serialized)  // _STA: Status
            {
                Return (CSTA (One))
            }

            Name (_MAT, Buffer (0x08)  // _MAT: Multiple APIC Table Entry
            {
                  0x00, 0x08, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00   // ........
            })
            Method (_EJ0, 1, NotSerialized)  // _EJx: Eject Device, x=0-9
            {
                CEJ0 (One)
            }

            Method (_OST, 3, Serialized)  // _OST: OSPM Status Indication
            {
                COST (One, Arg0, Arg1, Arg2)
            }
        }

        Processor (C002, 0x02, 0x00000000, 0x00)
        {
            Method (_STA, 0, Serialized)  // _STA: Status
            {
                Return (CSTA (0x02))
            }

            Name (_MAT, Buffer (0x08)  // _MAT: Multiple APIC Table Entry
            {
                  0x00, 0x08, 0x02, 0x02, 0x01, 0x00, 0x00, 0x00   // ........
            })
            Method (_EJ0, 1, NotSerialized)  // _EJx: Eject Device, x=0-9
            {
                CEJ0 (0x02)
            }

            Method (_OST, 3, Serialized)  // _OST: OSPM Status Indication
            {
                COST (0x02, Arg0, Arg1, Arg2)
            }
        }

        Processor (C003, 0x03, 0x00000000, 0x00)
        {
            Method (_STA, 0, Serialized)  // _STA: Status
            {
                Return (CSTA (0x03))
            }

            Name (_MAT, Buffer (0x08)  // _MAT: Multiple APIC Table Entry
            {
                  0x00, 0x08, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00   // ........
            })
            Method (_EJ0, 1, NotSerialized)  // _EJx: Eject Device, x=0-9
            {
                CEJ0 (0x03)
            }

            Method (_OST, 3, Serialized)  // _OST: OSPM Status Indication
            {
                COST (0x03, Arg0, Arg1, Arg2)
            }
        }

        Processor (C004, 0x04, 0x00000000, 0x00)
        {
            Method (_STA, 0, Serialized)  // _STA: Status
            {
                Return (CSTA (0x04))
            }

            Name (_MAT, Buffer (0x08)  // _MAT: Multiple APIC Table Entry
            {
                  0x00, 0x08, 0x04, 0x04, 0x01, 0x00, 0x00, 0x00   // ........
            })
            Method (_EJ0, 1, NotSerialized)  // _EJx: Eject Device, x=0-9
            {
                CEJ0 (0x04)
            }

            Method (_OST, 3, Serialized)  // _OST: OSPM Status Indication
            {
                COST (0x04, Arg0, Arg1, Arg2)
            }
        }

        Processor (C005, 0x05, 0x00000000, 0x00)
        {
            Method (_STA, 0, Serialized)  // _STA: Status
            {
                Return (CSTA (0x05))
            }

            Name (_MAT, Buffer (0x08)  // _MAT: Multiple APIC Table Entry
            {
                  0x00, 0x08, 0x05, 0x05, 0x01, 0x00, 0x00, 0x00   // ........
            })
            Method (_EJ0, 1, NotSerialized)  // _EJx: Eject Device, x=0-9
            {
                CEJ0 (0x05)
            }

            Method (_OST, 3, Serialized)  // _OST: OSPM Status Indication
            {
                COST (0x05, Arg0, Arg1, Arg2)
            }
        }

        Processor (C006, 0x06, 0x00000000, 0x00)
        {
            Method (_STA, 0, Serialized)  // _STA: Status
            {
                Return (CSTA (0x06))
            }

            Name (_MAT, Buffer (0x08)  // _MAT: Multiple APIC Table Entry
            {
                  0x00, 0x08, 0x06, 0x06, 0x01, 0x00, 0x00, 0x00   // ........
            })
            Method (_EJ0, 1, NotSerialized)  // _EJx: Eject Device, x=0-9
            {
                CEJ0 (0x06)
            }

            Method (_OST, 3, Serialized)  // _OST: OSPM Status Indication
            {
                COST (0x06, Arg0, Arg1, Arg2)
            }
        }

        Processor (C007, 0x07, 0x00000000, 0x00)
        {
            Method (_STA, 0, Serialized)  // _STA: Status
            {
                Return (CSTA (0x07))
            }

            Name (_MAT, Buffer (0x08)  // _MAT: Multiple APIC Table Entry
            {
                  0x00, 0x08, 0x07, 0x07, 0x01, 0x00, 0x00, 0x00   // ........
            })
            Method (_EJ0, 1, NotSerialized)  // _EJx: Eject Device, x=0-9
            {
                CEJ0 (0x07)
            }

            Method (_OST, 3, Serialized)  // _OST: OSPM Status Indication
            {
                COST (0x07, Arg0, Arg1, Arg2)
            }
        }
    }
}

Scope (\_GPE)
{
    Method (\_E02, 0, NotSerialized)  // _Exx: Edge-Triggered GPE, xx=0x00-0xFF
    {
        \_SB.CPUS.CSCN ()
    }
}
