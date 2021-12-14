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

#include "utilities.h"
#include "logger.h"
#include <cstdlib>
#include <string>
#include <cstring>
#include <map>
#include "object.h"

#define MAX_LEVEL 10

static std::map<std::string, ClassItem*>* classes = nullptr;

/* Alias PciHost to pci-host */
std::string get_alias(const char* name) {
  std::string ret;
  for (size_t i = 0; i < strlen(name); i++) {
    if (isupper(name[i])) {
      if (i > 0) {
        ret.push_back('-');
      }
      ret.push_back(tolower(name[i]));
    } else {
      ret.push_back(name[i]);
    }
  }
  return ret;
}

void register_class(int type, const char* name, const char* source_path, ClassCreator create) {
  ClassItem* item = new ClassItem;
  item->class_type = type;
  item->class_name = name;
  item->class_file = source_path;
  item->create = create;

  if (!classes) {
    classes = new std::map<std::string, ClassItem*>;
  }
  MV_ASSERT(classes->find(name) == classes->end());
  (*classes)[name] = item;
  (*classes)[get_alias(name)] = item;
  // MV_LOG("register class %s", name);
}


Object* realize_class(const char* name) {
  auto it = classes->find(name);
  if (it == classes->end()) {
    MV_PANIC("class not found %s", name);
  }
  Object* o = it->second->create();
  o->set_classname(it->second->class_name);
  o->set_name(name);
  return o;
}

