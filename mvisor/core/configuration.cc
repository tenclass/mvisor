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
}

void Configuration::LoadObjects(const YAML::Node& objects_node) {
  auto &objects = machine_->objects_;
  struct NodeObject {
    YAML::Node node;
    Object* object;
  };
  std::vector<NodeObject> v;

  /* Create objects */
  for (auto it = objects_node.begin(); it != objects_node.end(); it++) {
    auto node = *it;
    string class_name = node["class"].as<string>();
    string name;
    if (node["name"]) {
      name = node["name"].as<string>();
    } else {
      name = class_name;
    }
    auto objects_it = objects.find(name);
    Object* object;
    if (objects_it == objects.end()) {
      object = Object::Create(class_name.c_str());
      object->set_name(name.c_str());
      objects[name] = object;
    } else {
      object = objects_it->second;
    }
    v.emplace_back(NodeObject { .node = node, .object = object });

    /* Load object properties */
    for (auto it2 = node.begin(); it2 != node.end(); it2++) {
      string key = it2->first.as<string>();
      auto value = it2->second;
      if (key == "name" || key == "class" || key == "children" || key == "parent") {
        continue;
      }
      if (key == "debug") {
        object->set_debug(value.as<bool>());
        continue;
      }
      /* try type uint64_t or bool or string */
      try {
        auto uint_value = value.as<uint64_t>();
        (*object)[key] = uint_value;
        continue;
      } catch (const YAML::BadConversion& e) { }
      try {
        auto bool_value = value.as<bool>();
        (*object)[key] = bool_value;
        continue;
      } catch (const YAML::BadConversion& e) { }
      try {
        auto string_value = value.as<string>();
        (*object)[key] = string_value;
        continue;
      } catch (const YAML::BadConversion& e) { }
      MV_PANIC("Object %s has invalid key %s", name.c_str(), key.c_str());
    }
  }
  
  /* Add children */
  for (auto &node_object : v) {
    if (node_object.node["children"]) {
      auto children_node = node_object.node["children"];
      for (auto it = children_node.begin(); it != children_node.end(); it++) {
        /* If object by name not found, try to create one with class name */
        auto name = it->as<string>();
        auto objects_it = objects.find(name);
        Object* object;
        if (objects_it == objects.end()) {
          object = Object::Create(name.c_str());
          objects[name] = object;
        } else {
          object = objects_it->second;
        }
        node_object.object->AddChild(object);
      }
    }
  }

  /* Add to parent */
  for (auto &node_object : v) {
    if (node_object.node["parent"]) {
      auto parent = node_object.node["parent"].as<string>();
      auto objects_it = objects.find(parent);
      if (objects_it == objects.end()) {
        MV_PANIC("Object %s has invalid parent value %s", node_object.object->name(), parent.c_str());
      } else {
        objects_it->second->AddChild(node_object.object);
      }
    }
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
