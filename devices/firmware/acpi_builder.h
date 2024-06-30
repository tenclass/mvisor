#ifndef _MVISOR_ACPI_BUILDER_H_
#define _MVISOR_ACPI_BUILDER_H_

#include <map>
#include <vector>
#include <string>

#include "acpi.h"
#include "acpi_loader.h"

class Machine;
class AcpiBuilder {
 private:
  Machine*  machine_;
  std::map<std::string, std::string>  tables_;
  AcpiLoader loader_;
  bool IsQ35();

  void BuildRsdp();
  void BuildRsdt();
  void BuildWaet();
  void BuildApic();
  void BuildFacp();
  void BuildFacs();
  void BuildMcfg();
  void BuildDsdt();
  void BuildTableLoader();
  void BuildAcpiTableHeader(const std::string table_name, void* data, size_t size, uint8_t revision = 1);

 public:
  AcpiBuilder(Machine* machine);
  std::vector<std::string> GetFileNames();
  std::string GetTable(const std::string table_name);
};

#endif // _MVISOR_ACPI_BUILDER_H_
