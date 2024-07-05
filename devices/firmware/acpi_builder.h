#ifndef _MVISOR_ACPI_BUILDER_H_
#define _MVISOR_ACPI_BUILDER_H_

#include <map>
#include <vector>
#include <string>

#include "device_interface.h"
#include "acpi.h"
#include "acpi_loader.h"


class Machine;
class AcpiBuilder {
 private:
  Machine*  machine_;
  std::map<std::string, AcpiTableInterface*> tables_;
  AcpiLoader loader_;
  bool IsQ35();

  std::string BuildRsdp();
  std::string BuildRsdt();
  std::string BuildWaet();
  std::string BuildApic();
  std::string BuildFacp();
  std::string BuildFacs();
  std::string BuildMcfg();
  std::string BuildDsdt();
  std::string BuildCpuSsdt();
  void BuildAcpiTableHeader(const std::string table_name, void* data, size_t size, uint8_t revision = 1);

 public:
  AcpiBuilder(Machine* machine);
  std::vector<std::string> GetTableNames();
  std::string GetTable(const std::string table_name);
  std::string GetTableLoader();
};

#endif // _MVISOR_ACPI_BUILDER_H_
