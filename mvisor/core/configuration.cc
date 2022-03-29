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

#include "configuration.h"

#include <libgen.h>
#include <unistd.h>
#include <cstring>

#include <fstream>

#include "machine.h"
#include "logger.h"

using namespace std;

Configuration::Configuration(Machine* machine) : machine_(machine) {
  InitializePaths();
}

/* Apparently this only works under Linux */
void Configuration::InitializePaths() {
  char temp[1024];
  bzero(temp, sizeof(temp));
  getcwd(temp, sizeof(temp) - 1);
  directories_.insert(temp);

  bzero(temp, sizeof(temp));
  if (readlink("/proc/self/exe", temp, sizeof(temp) - 1) > 0) {
    directories_.insert(temp);
  }
}

/* Try to find file in current path and executable path */
std::string Configuration::FindPath(std::string path) const {
  if (access(path.c_str(), F_OK) == 0) {
    return path;
  }
  if (path[0] != '/') {
    for (auto &dir : directories_) {
      auto new_path = dir + "/" + path;
      if (access(new_path.c_str(), F_OK) == 0) {
        return new_path;
      }
    }
  }
  /* Path not found */
  MV_PANIC("Path not found %s", path.c_str());
  return path;
}

/* Load configuration from file */
bool Configuration::Load(std::string path) {
  if (!machine_->objects_.empty()) {
    MV_PANIC("Machine already loaded");
    return false;
  }
  return LoadFile(path);
}

/* Load configuration file *.yaml */
bool Configuration::LoadFile(std::string path) {
  path_ = FindPath(path);
  YAML::Node config = YAML::LoadFile(path_);
  if (config["base"]) {
    /* Add the current config directory to directories and load base file */
    char temp[1024] = { 0 };
    strcpy(temp, path.c_str());
    directories_.insert(dirname(temp));
    Load(config["base"].as<string>());
  }

  if (config["machine"]) {
    LoadMachine(config["machine"]);
  }

  if (config["objects"]) {
    LoadObjects(config["objects"]);
  }

  if (config["snapshot"]) {
    snapshot_ = config["snapshot"].as<bool>();
  } else {
    snapshot_ = false;
  }
  return true;
}

Object* Configuration::CreateObject(std::string class_name, std::string name) {
  auto &objects = machine_->objects_;
  MV_ASSERT(objects.find(name) == objects.end());

  auto object = Object::Create(class_name.c_str());
  object->set_name(name.c_str());
  objects[name] = object;
  return object;
}

Object* Configuration::GetOrCreateObject(std::string class_name, std::string name) {
  auto &objects = machine_->objects_;
  auto it = objects.find(name);
  Object* object;
  if (it == objects.end()) {
    object = CreateObject(class_name, name);
  } else {
    object = it->second;
  }
  return object;
}

std::string Configuration::GenerateObjectName(std::string class_name) {
  auto &objects = machine_->objects_;

  /* try to use class name as name */
  if(objects.find(class_name) == objects.end()) {
    return class_name;
  }

  for (int index = 1; index < 256; index++) {
    char name[100];
    sprintf(name, "%s-%d", class_name.c_str(), index);
    if (objects.find(name) == objects.end()) {
      return name;
    }
  }
  /* never get here */
  MV_PANIC("failed to set object name of %s", class_name.c_str());
  return class_name;
}

/* Make sure all objects are linked. If parent not found, try to create one */
void Configuration::CreateParents(Object* object) {
  auto &objects = machine_->objects_;

  MV_ASSERT(!object->parent());
  if (!object->parent_name()[0]) {
    return;
  }
  
  auto it = objects.find(object->parent_name());
  if (it == objects.end()) {
    auto parent = CreateObject(object->parent_name(), object->parent_name());
    parent->AddChild(object);
    if (!parent->parent()) {
      CreateParents(parent);
    }
  } else {
    it->second->AddChild(object);
  }
}

void Configuration::SetObjectKeyValue(Object* object, std::string key, const YAML::Node& value) {
  /* try type uint64_t or bool or string */
  try {
    auto uint_value = value.as<uint64_t>();
    (*object)[key] = uint_value;
    return;
  } catch (const YAML::BadConversion& e) { }
  try {
    auto bool_value = value.as<bool>();
    (*object)[key] = bool_value;
    return;
  } catch (const YAML::BadConversion& e) { }
  try {
    auto string_value = value.as<string>();
    (*object)[key] = string_value;
    return;
  } catch (const YAML::BadConversion& e) { }
  MV_PANIC("Object %s has invalid key %s", object->name(), key.c_str());
}

/* Extract machine configs */
void Configuration::LoadMachine(const YAML::Node& node) {
  if (node["memory"]) {
    string memory = node["memory"].as<string>();
    MV_ASSERT(memory.back() == 'G');
    machine_->ram_size_ = (1UL << 30) * atol(memory.substr(0, memory.length() - 1).c_str());
  }
  if (node["vcpu"]) {
    machine_->num_vcpus_ = node["vcpu"].as<uint64_t>();
  }
  if (node["bios"]) {
    bios_path_ = FindPath(node["bios"].as<string>());
  }
  if (node["debug"]) {
    machine_->debug_ = node["debug"].as<bool>();
  }
  if (node["hypervisor"]) {
    machine_->hypervisor_ = node["hypervisor"].as<bool>();
  }
}

void Configuration::LoadObjects(const YAML::Node& objects_node) {
  std::vector<Object*> created;

  /* Create objects */
  for (auto it = objects_node.begin(); it != objects_node.end(); it++) {
    auto node = *it;
    string class_name = node["class"].as<string>();
    string name;
    if (node["name"]) {
      name = node["name"].as<string>();
    } else {
      name = GenerateObjectName(class_name);
    }
    auto object = GetOrCreateObject(class_name, name);

    /* Load object properties */
    for (auto it2 = node.begin(); it2 != node.end(); it2++) {
      string key = it2->first.as<string>();
      auto value = it2->second;
      if (key == "name" || key == "class") {
        continue;
      }
      if (key == "debug") {
        object->set_debug(value.as<bool>());
        continue;
      } else if (key == "parent") {
        object->set_parent_name(value.as<string>().c_str());
        continue;
      }
      SetObjectKeyValue(object, key, value);
    }

    created.push_back(object);
  }

  for (auto object : created) {
    CreateParents(object);
  }
}

/* Save configuration to file */
bool Configuration::Save(std::string path) {
  std::ofstream ofs(path, std::ios::out);
  if (!ofs.is_open())
    return false;

  auto root = YAML::Node();
  root["snapshot"] = true;
  root["version"] = 1;

  /* Build time */
  auto now = time(nullptr);
  char temp[100];
  strftime(temp, 100, "%Y-%m-%d %H:%M:%S", gmtime(&now));
  root["create_time"] = temp;
  
  /* Build machine */
  auto machine_node = root["machine"];
  SaveMachine(machine_node);

  /* Build objects */
  std::vector<Object*> objects;
  for (auto it = machine_->objects_.begin(); it != machine_->objects_.end(); it++) {
    objects.push_back(it->second);
  }
  /* Sort by id */
  std::sort(objects.begin(), objects.end(), [](Object* a, Object* b) {
    return a->id() < b->id();
  });

  auto objects_node = root["objects"];
  for (auto o : objects) {
    auto node = YAML::Node();
    node["name"] = o->name();
    node["class"] = o->classname();
    node["debug"] = o->debug();
    if (o->parent()) {
      node["parent"] = o->parent()->name();
    }
    auto &kv = o->key_values();
    for (auto it = kv.begin(); it != kv.end(); it++) {
      auto& value = it->second;
      switch (value.index())
      {
      case 0:
        node[it->first] = std::get<bool>(it->second);
        break;
      case 1:
        node[it->first] = std::get<uint64_t>(it->second);
        break;
      case 2:
        node[it->first] = std::get<std::string>(it->second);
        break;
      }
    }
    objects_node.push_back(node);
  }

  ofs << root << std::endl;
  return true;
}

/* Save machine configs */
void Configuration::SaveMachine(YAML::Node& node) {
  std::stringstream ss;
  ss << machine_->ram_size_ / (1UL << 30) << "G";
  node["memory"] = ss.str();
  node["vcpu"] = machine_->num_vcpus_;
  node["debug"] = machine_->debug_;
  node["bios"] = bios_path_;
}
