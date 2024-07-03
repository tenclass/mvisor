#include "acpi_builder.h"
#include "logger.h"
#include "machine.h"
#include "i440fx.hex"
#include "q35.hex"
#include "cpu_container.hex"
#include "../chipset/pmio.h"

AcpiBuilder::AcpiBuilder(Machine* machine) : machine_(machine) {
}

bool AcpiBuilder::IsQ35() {
  return machine_->LookupObjectByClass("Q35Host") != nullptr;
}

std::string AcpiBuilder::GetTableLoader() {
  return loader_.GetCommands();
}

std::vector<std::string> AcpiBuilder::GetTableNames() {
  std::vector<std::string> table_names;
  table_names.push_back("etc/acpi/rsdp");
  table_names.push_back("etc/acpi/rsdt");
  table_names.push_back("etc/acpi/facp");
  table_names.push_back("etc/acpi/facs");
  table_names.push_back("etc/acpi/apic");
  if (IsQ35()) {
    table_names.push_back("etc/acpi/mcfg");
  }
  table_names.push_back("etc/acpi/waet");
  table_names.push_back("etc/acpi/dsdt");
  table_names.push_back("etc/acpi/cpus");

  // Look for class specific tables
  auto objects = machine_->LookupObjects([](auto obj) {
    return dynamic_cast<AcpiTableInterface*>(obj) != nullptr;
  });
  for (auto obj : objects) {
    char name[200];
    sprintf(name, "etc/acpi/%s", obj->name());
    tables_[name] = dynamic_cast<AcpiTableInterface*>(obj);
    table_names.push_back(name);
  }
  return table_names;
}

std::string AcpiBuilder::GetTable(const std::string table_name) {
  if (table_name == "etc/acpi/rsdp") {
    return BuildRsdp();
  } else if (table_name == "etc/acpi/rsdt") {
    return BuildRsdt();
  } else if (table_name == "etc/acpi/facp") {
    return BuildFacp();
  } else if (table_name == "etc/acpi/facs") {
    return BuildFacs();
  } else if (table_name == "etc/acpi/apic") {
    return BuildApic();
  } else if (table_name == "etc/acpi/mcfg") {
    return BuildMcfg();
  } else if (table_name == "etc/acpi/waet") {
    return BuildWaet();
  } else if (table_name == "etc/acpi/dsdt") {
    return BuildDsdt();
  } else if (table_name == "etc/acpi/cpus") {
    return BuildCpuSsdt();
  } else {
    auto it = tables_.find(table_name);
    if (it != tables_.end()) {
      loader_.AddAllocateCommand(it->first.c_str(), 16, ALLOC_ZONE_HIGH);
      return it->second->GetAcpiTable();
    }
  }
  MV_PANIC("Unknown table name: %s\n", table_name.c_str());
  return "";
}

// Build Root System Descriptor Pointer
std::string AcpiBuilder::BuildRsdp() {
  acpi_rdsp_table rsdp = {};
  memcpy(rsdp.signature, "RSD PTR ", 8);
  memcpy(rsdp.oem_id, "TENCLS", 6);

  loader_.AddAllocateCommand("etc/acpi/rsdp", 16, ALLOC_ZONE_FSEG);
  loader_.AddAddPointerCommand("etc/acpi/rsdp", "etc/acpi/rsdt", 16, 4);
  loader_.AddChecksumCommand("etc/acpi/rsdp", 8, 0, 20);
  return std::string((char*)&rsdp, sizeof(rsdp));
}

// Build Firmware ACPI Control Structure
std::string AcpiBuilder::BuildFacs() {
  acpi_facs_table facs = {};
  facs.signature = *(uint32_t*)"FACS";
  facs.length = sizeof(acpi_facs_table);
  facs.hardware_signature = 0;

  loader_.AddAllocateCommand("etc/acpi/facs", 64, ALLOC_ZONE_HIGH);
  return std::string((char*)&facs, sizeof(facs));
}

// Build common ACPI table header
void AcpiBuilder::BuildAcpiTableHeader(const std::string table_name, void* data, size_t size, uint8_t revision) {
  auto header = (acpi_table_header*)data;
  header->signature = *(uint32_t*)table_name.data();
  header->length = size;
  header->revision = revision;
  memcpy(header->oem_id, "TENCLS", 6);
  memcpy(header->oem_table_id, "MVPC    ", 8);
  header->oem_revision = 1;
  memcpy(header->asl_compiler_id, "INTL", 4);
  header->asl_compiler_revision = 0x20210604;
}

// Build Root System Descriptor Table
std::string AcpiBuilder::BuildRsdt() {
  std::vector<std::string> table_names = {
    "etc/acpi/facp",
    "etc/acpi/apic",
    "etc/acpi/waet",
    "etc/acpi/cpus"
  };
  if (IsQ35()) {
    table_names.push_back("etc/acpi/mcfg");
  }
  for(auto& table : tables_) {
    table_names.push_back(table.first);
  }

  std::string rsdt;
  acpi_table_header header = {};
  BuildAcpiTableHeader("RSDT", &header, sizeof(header));
  rsdt.append((char*)&header, sizeof(header));

  loader_.AddAllocateCommand("etc/acpi/rsdt", 16, ALLOC_ZONE_HIGH);
  for (auto& table_name : table_names) {
    loader_.AddAddPointerCommand("etc/acpi/rsdt", table_name.c_str(), rsdt.size(), 4);
    rsdt.append("\x00\x00\x00\x00", 4);
  }
  loader_.AddChecksumCommand("etc/acpi/rsdt", 9, 0, rsdt.size());
  return rsdt;
}

// Build Fixed ACPI Description Table
std::string AcpiBuilder::BuildFacp() {
  // find Pmio class object
  auto objects = machine_->LookupObjects([](auto obj) {
    return dynamic_cast<Pmio*>(obj) != nullptr;
  });
  MV_ASSERT(objects.size() == 1);
  auto pmio = dynamic_cast<Pmio*>(objects[0]);

  acpi_fadt_table facp = {};
  facp.interrupt_model = 1; // Multiple APIC
  facp.sci_int = 9; // SCI interrupt
  facp.smi_cmd = 0xB2; // SMI command port
  facp.acpi_enable = 2; // Value to write to smi_cmd to enable ACPI
  facp.acpi_disable = 3; // Value to write to smi_cmd to disable ACPI
  facp.pm1a_evt_blk = pmio->pmio_base(); // PM1a event block
  facp.pm1a_cnt_blk = pmio->pmio_base() + 4; // PM1a control block
  facp.pm_tmr_blk = pmio->pmio_base() + 8; // PM timer block
  facp.gpe0_blk = pmio->pmio_base() + 0x20; // GPE0 block
  facp.pm1_evt_len = 4; // PM1 event length
  facp.pm1_cnt_len = 2; // PM1 control length
  facp.pm_tmr_len = 4; // PM timer length
  facp.gpe0_blk_len = 0x10; // GPE0 block length
  facp.plvl2_lat = 0xFFF; // PLVL2 latency
  facp.plvl3_lat = 0xFFF; // PLVL3 latency
  facp.century = 0x32; // Century support in RTC
  facp.iapc_boot_arch = 2; // Must be 2 for ACPI 2.0+
  facp.flags =  ACPI_FADT_F_WBINVD | ACPI_FADT_F_PROC_C1 |
                ACPI_FADT_F_SLP_BUTTON | ACPI_FADT_F_RTC_S4 |
                ACPI_FADT_F_USE_PLATFORM_CLOCK | ACPI_FADT_F_RESET_REG_SUP;
  
  acpi_20_generic_address reset_reg = { .address_space_id = 0x1, .bit_width = 8, .address = 0xCF9 };
  facp.reset_reg = reset_reg;
  facp.reset_value = 0xF;

  // Currently we use the version 3.0 of the FADT table
  BuildAcpiTableHeader("FACP", &facp, sizeof(facp), 3);

  loader_.AddAllocateCommand("etc/acpi/facp", 16, ALLOC_ZONE_HIGH);
  loader_.AddAddPointerCommand("etc/acpi/facp", "etc/acpi/facs", 0x24, 4);
  loader_.AddAddPointerCommand("etc/acpi/facp", "etc/acpi/dsdt", 0x28, 4);
  loader_.AddChecksumCommand("etc/acpi/facp", 9, 0, sizeof(facp));
  return std::string((char*)&facp, sizeof(facp));
}

// Build Multiple APIC Description Table
std::string AcpiBuilder::BuildApic() {
  std::string buffer;
  struct multiple_apic_table madt = {};
  madt.local_apic_address = 0xFEE00000; // Local APIC address
  madt.flags = 1; // PC-AT compatible
  BuildAcpiTableHeader("APIC", &madt, sizeof(madt));
  buffer.append((char*)&madt, sizeof(madt));

  // Processor Local APIC
  for (int i = 0; i < machine_->num_vcpus(); i++) {
    madt_processor_apic apic = {};
    apic.type = APIC_PROCESSOR;
    apic.length = sizeof(apic);
    apic.processor_id = i;
    apic.local_apic_id = i;
    apic.flags = 1; // Enabled
    buffer.append((char*)&apic, sizeof(apic));
  }

  // IO APIC
  madt_io_apic io_apic = {};
  io_apic.type = APIC_IO;
  io_apic.length = sizeof(io_apic);
  io_apic.io_apic_id = 0;
  io_apic.address = 0xFEC00000;
  buffer.append((char*)&io_apic, sizeof(io_apic));

  // IRQ0 override
  madt_intsrcovr irq0 = {};
  irq0.type = APIC_XRUPT_OVERRIDE;
  irq0.length = sizeof(irq0);
  irq0.source = 0;
  irq0.gsi = 2;
  irq0.flags = 0; // confirms to bus specifcation
  buffer.append((char*)&irq0, sizeof(irq0));

  uint8_t pci_irqs[] = { 5, 9, 10, 11 };
  for (size_t i = 0; i < sizeof(pci_irqs); i++) {
    madt_intsrcovr irq = {};
    irq.type = APIC_XRUPT_OVERRIDE;
    irq.length = sizeof(irq);
    irq.source = pci_irqs[i];
    irq.gsi = pci_irqs[i];
    irq.flags = 0xD; // Active high, level triggered
    buffer.append((char*)&irq, sizeof(irq));
  }

  // Local APIC NMI
  madt_local_nmi nmi = {};
  nmi.type = APIC_LOCAL_NMI;
  nmi.length = sizeof(nmi);
  nmi.processor_id = 0xFF; // All processors
  nmi.flags = 0; // confirm to bus specification
  nmi.lint = 1;
  buffer.append((char*)&nmi, sizeof(nmi));

  // fix length
  *(uint32_t*)&buffer.data()[4] = buffer.size();
  loader_.AddAllocateCommand("etc/acpi/apic", 16, ALLOC_ZONE_HIGH);
  loader_.AddChecksumCommand("etc/acpi/apic", 9, 0, buffer.size());
  return buffer;
}

// Build PCI MMCFG Base Address Description Table
std::string AcpiBuilder::BuildMcfg() {
  auto object = machine_->LookupObjectByClass("Q35Host");
  if (object == nullptr) {
    return "";
  }
  auto q35 = dynamic_cast<PciDevice*>(object);

  // Read MCFG base address from Q35 PCI config space
  uint32_t mcfg_base = 0;
  q35->ReadPciConfigSpace(0x60, (uint8_t*)&mcfg_base, 4);
  mcfg_base &= 0xFFFFFFF0;

  acpi_mcfg_table mcfg = {};
  mcfg.allocation[0].address = mcfg_base;
  mcfg.allocation[0].pci_segment = 0;
  mcfg.allocation[0].start_bus_number = 0;
  mcfg.allocation[0].end_bus_number = 0xFF;
  BuildAcpiTableHeader("MCFG", &mcfg, sizeof(mcfg));
  loader_.AddAllocateCommand("etc/acpi/mcfg", 16, ALLOC_ZONE_HIGH);
  loader_.AddChecksumCommand("etc/acpi/mcfg", 9, 0, sizeof(mcfg));
  return std::string((char*)&mcfg, sizeof(mcfg));
}

// Build Windows ACPI Emulated Timer
std::string AcpiBuilder::BuildWaet() {
  acpi_waet_table waet = {};
  waet.emulated_device_flags = 1 << 1; /* ACPI PM timer good */
  BuildAcpiTableHeader("WAET", &waet, sizeof(waet));
  loader_.AddAllocateCommand("etc/acpi/waet", 16, ALLOC_ZONE_HIGH);
  loader_.AddChecksumCommand("etc/acpi/waet", 9, 0, sizeof(waet));
  return std::string((char*)&waet, sizeof(waet));
}

// Build Differentiated System Description Table
std::string AcpiBuilder::BuildDsdt() {
  std::string dsdt;
  if (IsQ35()) {
    dsdt = std::string((char*)q35_aml_code, sizeof(q35_aml_code));
  } else {
    dsdt = std::string((char*)i440fx_aml_code, sizeof(i440fx_aml_code));
  }
  dsdt[9] = 0; // reset checksum
  loader_.AddAllocateCommand("etc/acpi/dsdt", 16, ALLOC_ZONE_HIGH);
  loader_.AddChecksumCommand("etc/acpi/dsdt", 9, 0, dsdt.size());
  return dsdt;
}

std::string AcpiBuilder::BuildCpuSsdt() {
  std::string ssdt = std::string((char*)cpu_container_aml_code, sizeof(cpu_container_aml_code));
  if (machine_->num_vcpus() > 0xFF) {
    MV_ERROR("Too many CPUs: %d\n", machine_->num_vcpus());
    return "";
  }
  
  // Add ProcessorOp
  for (auto vcpu : machine_->vcpus()) {
    char name[5];
    snprintf(name, sizeof(name), "C0%02X", vcpu->vcpu_id());
    ssdt.append("\x5B\x83\x0B", 3);
    ssdt.append(name, 4);
    ssdt.push_back(vcpu->vcpu_id());
    ssdt.append("\x00\x00\x00\x00", 4);
    ssdt.append("\x00", 1);
  }
  *(uint32_t*)&ssdt[4] = ssdt.size();
  ssdt[9] = 0; // reset checksum
  loader_.AddAllocateCommand("etc/acpi/cpus", 16, ALLOC_ZONE_HIGH);
  loader_.AddChecksumCommand("etc/acpi/cpus", 9, 0, ssdt.size());
  return ssdt;
}
