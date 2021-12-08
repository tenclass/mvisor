/* 
 * MVisor SMBIOS Support
 * Copyright (C) 2021 Terrence <terrence@tenclass.com>
 * Copyright (C) 2009 Hewlett-Packard Development Company, L.P.
 * Authors:
 *  Alex Williamson <alex.williamson@hp.com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "smbios.h"
#include <cstring>
#include "logger.h"

Smbios::Smbios(Machine* machine) : machine_(machine) {
  SetupEntryPoint();
}

void Smbios::GetTables(std::string& anchor, std::string& tables) {
  /* FIXME: this tables should be implemented */
  tables = std::string("\x7F\x04\x00\x7F\x00\x00", 6);
  entry_point_.max_structure_size = 6;
  entry_point_.structure_table_length = tables.length();
  entry_point_.number_of_structures = 1;

  anchor = std::string((char*)&entry_point_, sizeof(entry_point_));
}

void Smbios::SetupEntryPoint() {
  bzero(&entry_point_, sizeof(entry_point_));
  entry_point_.length = sizeof(smbios_21_entry_point);

  memcpy(entry_point_.anchor_string, "_SM_", 4);
  memcpy(entry_point_.intermediate_anchor_string, "_DMI_", 5);

  /* smbios spec v2.8 */
  entry_point_.smbios_major_version = 2;
  entry_point_.smbios_minor_version = 8;
  entry_point_.smbios_bcd_revision = 0x28;
}

