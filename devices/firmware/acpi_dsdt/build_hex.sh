#!/bin/sh
cpp -P -o acpi-dsdt.dsl.i acpi-dsdt.dsl
iasl -tc -p acpi_dsdt acpi-dsdt.dsl.i
rm acpi_dsdt.aml acpi-dsdt.dsl.i
mv acpi_dsdt.hex acpi_dsdt.hex.h

cpp -P -o q35-acpi-dsdt.dsl.i q35-acpi-dsdt.dsl
iasl -tc -p q35_acpi_dsdt q35-acpi-dsdt.dsl.i
rm q35_acpi_dsdt.aml q35-acpi-dsdt.dsl.i
mv q35_acpi_dsdt.hex q35_acpi_dsdt.hex.h

