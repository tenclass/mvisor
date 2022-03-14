/* 
 * MVisor
 * Copyright (C) 2021 Terrence <terrence@tenclass.com>
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

#ifndef _MVISOR_CONFIG_H
#define _MVISOR_CONFIG_H

#include <string>
#include <map>
#include <set>
#include <cstdio>
#include <yaml-cpp/yaml.h>
#include "object.h"

class Device;
class Machine;
class Configuration {
 public:
  Configuration(Machine* machine);
  bool Load(std::string path);
  bool Save(std::string path);
  std::string FindPath(std::string path) const;

  inline bool snapshot() const { return snapshot_; }
  inline const std::string& path() const { return path_; }

 private:
  void InitializePaths();
  bool LoadFile(std::string path);
  void LoadMachine(const YAML::Node& node);
  void LoadObjects(const YAML::Node& node);
  void SaveMachine(YAML::Node& node);

  Machine*    machine_;
  Device*     root_;
  std::set<std::string> directories_;
  bool        snapshot_;
  std::string path_;
};

#endif // _MVISOR_CONFIG_H
