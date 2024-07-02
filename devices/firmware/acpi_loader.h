#ifndef _MVISOR_ACPI_TABLE_LOADER_H
#define _MVISOR_ACPI_TABLE_LOADER_H


#include <stdint.h>
#include <vector>
#include <string>
#include "firmware_config.h"

/*
 * Linker/loader is a paravirtualized interface that passes commands to guest.
 * The commands can be used to request guest to
 * - allocate memory chunks and initialize them from QEMU FW CFG files
 * - link allocated chunks by storing pointer to one chunk into another
 * - calculate ACPI checksum of part of the chunk and store into same chunk
 */
#define LOADER_FILESZ FW_CFG_MAX_FILE_PATH

struct LoaderEntry {
  uint32_t command;
  union {
    /*
      * COMMAND_ALLOCATE - allocate a table from @alloc.file
      * subject to @alloc.align alignment (must be power of 2)
      * and @alloc.zone (can be HIGH or FSEG) requirements.
      *
      * Must appear exactly once for each file, and before
      * this file is referenced by any other command.
      */
    struct {
      char file[LOADER_FILESZ];
      uint32_t align;
      uint8_t zone;
    } alloc;

    /*
      * COMMAND_ADD_POINTER - patch the table (originating from
      * @dest_file) at @pointer.offset, by adding a pointer to the table
      * originating from @src_file. 1,2,4 or 8 byte unsigned
      * addition is used depending on @pointer.size.
      */
    struct {
      char dest_file[LOADER_FILESZ];
      char src_file[LOADER_FILESZ];
      uint32_t offset;
      uint8_t size;
    } pointer;

    /*
      * COMMAND_ADD_CHECKSUM - calculate checksum of the range specified by
      * @cksum_start and @cksum_length fields,
      * and then add the value at @cksum.offset.
      * Checksum simply sums -X for each byte X in the range
      * using 8-bit math.
      */
    struct {
      char file[LOADER_FILESZ];
      uint32_t offset;
      uint32_t start;
      uint32_t length;
    } cksum;

    /*
      * COMMAND_WRITE_POINTER - write the fw_cfg file (originating from
      * @dest_file) at @wr_pointer.offset, by adding a pointer to
      * @src_offset within the table originating from @src_file.
      * 1,2,4 or 8 byte unsigned addition is used depending on
      * @wr_pointer.size.
      */
    struct {
      char dest_file[LOADER_FILESZ];
      char src_file[LOADER_FILESZ];
      uint32_t dst_offset;
      uint32_t src_offset;
      uint8_t size;
    } wr_pointer;

    /* padding */
    char pad[124];
  };
} __attribute__((packed));

enum {
  COMMAND_ALLOCATE          = 0x1,
  COMMAND_ADD_POINTER       = 0x2,
  COMMAND_ADD_CHECKSUM      = 0x3,
  COMMAND_WRITE_POINTER     = 0x4,
};

enum {
  ALLOC_ZONE_HIGH = 0x1,
  ALLOC_ZONE_FSEG = 0x2,
};


class AcpiLoader {
 private:
  std::vector<LoaderEntry> commands_; 
  bool Exists(LoaderEntry& entry);

 public:
  AcpiLoader();
  void AddAllocateCommand(const char* file, uint32_t align, uint8_t zone);
  void AddAddPointerCommand(const char* dest_file, const char* src_file, uint32_t offset, uint8_t size);
  void AddChecksumCommand(const char* file, uint32_t offset, uint32_t start, uint32_t length);

  std::string GetCommands();
};

#endif // _MVISOR_ACPI_TABLE_LOADER_H_
