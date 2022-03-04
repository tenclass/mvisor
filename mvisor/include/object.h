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

#ifndef _MVISOR_OBJECT_H
#define _MVISOR_OBJECT_H

#include <variant>
#include <vector>
#include <string>
#include <map>

#define OBJECT_MAX_NAME_LENGTH 100

typedef std::variant<std::string, bool, uint64_t> Value;

class Object {
 public:
  static Object* Create(const char* class_name);

  Object();
  virtual ~Object();

  virtual void AddChild(Object* device);
  virtual void RemoveChild(Object* object);

  const char* name() { return name_; }
  const char* classname() { return classname_; }
  bool debug() { return debug_; }
  void set_debug(bool debug);
  void set_name(const char* name);
  void set_classname(const char* classname);
  const Object* parent() { return parent_; }
  const std::vector<Object*>& children() { return children_; }
  
  Value& operator[](std::string key) { return key_values_[key]; }
  bool has_key(std::string key) { return key_values_.find(key) != key_values_.end(); }

 protected:
  bool debug_ = false;
  char name_[OBJECT_MAX_NAME_LENGTH];
  char classname_[OBJECT_MAX_NAME_LENGTH];
  std::map<std::string, Value> key_values_;

  /* Object topology */
  Object* parent_;
  std::vector<Object*> children_;
};

#endif // _MVISOR_OBJECT_H

