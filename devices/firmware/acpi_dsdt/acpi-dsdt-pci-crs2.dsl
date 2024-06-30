
Scope (\_SB.PCI0)
{
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
