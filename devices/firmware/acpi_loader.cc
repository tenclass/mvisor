#include "acpi_loader.h"

AcpiLoader::AcpiLoader() {
}

// Add a command to allocate a table from @file
void AcpiLoader::AddAllocateCommand(const char* file, uint32_t align, uint8_t zone) {
  LoaderEntry entry = {};
  entry.command = COMMAND_ALLOCATE;
  entry.alloc.align = align;
  entry.alloc.zone = zone;
  strncpy(entry.alloc.file, file, LOADER_FILESZ - 1);
  if (!Exists(entry)) {
    commands_.push_back(entry);
  }
}

// Add a command to modify a table by adding a pointer to another table
void AcpiLoader::AddAddPointerCommand(const char* dest_file, const char* src_file, uint32_t offset, uint8_t size) {
  LoaderEntry entry = {};
  entry.command = COMMAND_ADD_POINTER;
  entry.pointer.size = size;
  entry.pointer.offset = offset;
  strncpy(entry.pointer.dest_file, dest_file, LOADER_FILESZ - 1);
  strncpy(entry.pointer.src_file, src_file, LOADER_FILESZ - 1);
  if (!Exists(entry)) {
    commands_.push_back(entry);
  }
}

// Add a command to modify a table by adding a checksum
void AcpiLoader::AddChecksumCommand(const char* file, uint32_t offset, uint32_t start, uint32_t length) {
  LoaderEntry entry = {};
  entry.command = COMMAND_ADD_CHECKSUM;
  entry.cksum.offset = offset;
  entry.cksum.start = start;
  entry.cksum.length = length;
  strncpy(entry.cksum.file, file, LOADER_FILESZ - 1);
  if (!Exists(entry)) {
    commands_.push_back(entry);
  }
}

std::string AcpiLoader::GetCommands() {
  // sort commands by type, make sure allocation commands come first
  std::sort(commands_.begin(), commands_.end(), [](LoaderEntry& a, LoaderEntry& b) {
    return a.command < b.command;
  });

  std::string blob;
  for (auto& entry : commands_) {
    blob.append((char*)&entry, sizeof(entry));
  }
  return blob;
}

bool AcpiLoader::Exists(LoaderEntry& entry) {
  for (auto& command : commands_) {
    if (memcmp(&command, &entry, sizeof(entry)) == 0) {
      return true;
    }
  }
  return false;
}
