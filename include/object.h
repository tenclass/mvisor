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

typedef std::variant<bool, uint64_t, std::string> Value;

class Object {
 public:
  static Object* Create(const char* class_name);

  Object();
  virtual ~Object();

  virtual void AddChild(Object* device);
  virtual void RemoveChild(Object* object);

  inline uint id() const { return id_; }
  inline const char* name() const { return name_; }
  inline const char* classname() const { return classname_; }
  inline bool debug() const { return debug_; }
  inline const Object* parent() const { return parent_; }
  inline const char* parent_name() const { return parent_name_; }
  inline const std::vector<std::string>& default_parent_classes() const { return default_parent_classes_; }
  inline const std::vector<Object*>& children() const { return children_; }
  inline bool has_key(std::string key) const { return key_values_.find(key) != key_values_.end(); }
  inline Value& operator[](std::string key) { return key_values_[key]; }
  inline const std::map<std::string, Value>& key_values() const { return key_values_; }

  void set_id(uint id) { id_ = id; }
  void set_debug(bool debug) { debug_ = debug; }
  void set_name(const char* name);
  void set_classname(const char* classname);
  void set_parent_name(const char* parent_name);
  void set_default_parent_class(const char* primary, const char* secondary = nullptr);

 protected:
  uint id_ = 0;
  bool debug_ = false;
  char name_[OBJECT_MAX_NAME_LENGTH];
  char classname_[OBJECT_MAX_NAME_LENGTH];
  char parent_name_[OBJECT_MAX_NAME_LENGTH];
  std::map<std::string, Value> key_values_;

  /* Object topology */
  Object* parent_ = nullptr;
  std::vector<Object*> children_;
  std::vector<std::string> default_parent_classes_;
};

#endif // _MVISOR_OBJECT_H

