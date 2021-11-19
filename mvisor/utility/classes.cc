#include "utility.h"
#include "logger.h"
#include <cstdlib>
#include <string>
#include <map>
#include "object.h"

#define MAX_LEVEL 10

static std::map<std::string, ClassItem*>* classes = nullptr;

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
  MV_LOG("register device class %s", name);
}


Object* realize_class(const char* name) {
  auto it = classes->find(name);
  if (it == classes->end()) {
    MV_PANIC("class not found %s", name);
  }
  Object* o = it->second->create();
  return o;
}

