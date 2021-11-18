#include "object.h"
#include <cstring>
#include "logger.h"

const char* Object::name() {
  return name_;
}

void Object::set_name(const char* name) {
  strncpy(name_, name, 99);
}

Object::Object() {

}

Object::~Object() {

}
