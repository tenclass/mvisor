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


#include "object.h"
#include <cstring>
#include <algorithm>
#include "logger.h"
#include "utilities.h"

Object* Object::Create(const char* class_name) {
  return realize_class(class_name);
}

void Object::set_name(const char* name) {
  strncpy(name_, name, OBJECT_MAX_NAME_LENGTH - 1);
}

void Object::set_classname(const char* classname) {
  strncpy(classname_, classname, OBJECT_MAX_NAME_LENGTH - 1);
}

void Object::set_parent_name(const char* parent_name) {
  strncpy(parent_name_, parent_name, OBJECT_MAX_NAME_LENGTH - 1);
}

Object::Object() {

}

Object::~Object() {

}

void Object::AddChild(Object* object) {
  object->parent_ = this;
  if (std::find(children_.begin(), children_.end(), object) == children_.end()) {
    children_.push_back(object);
  }
}

void Object::RemoveChild(Object* object) {
  object->parent_ = nullptr;
  auto it = std::find(children_.begin(), children_.end(), object);
  if (it != children_.end()) {
    children_.erase(it);
  }
}
