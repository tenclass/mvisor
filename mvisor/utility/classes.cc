#include "utility.h"
#include "logger.h"
#include <cstdlib>
#include <string>
#include <map>
#include "object.h"

#define MAX_LEVEL 10

static std::map<std::string, ClassItem*> classes;

void register_class(int type, const char* name, const char* source_path, ClassCreator create) {
  ClassItem* item = new ClassItem;
  item->class_type = type;
  item->class_name = name;
  item->class_file = source_path;
  item->create = create;

  MV_ASSERT(classes.find(name) == classes.end());
  classes[name] = item;
}


Object* realize_class(const char* name) {
  auto it = classes.find(name);
  if (it == classes.end()) {
    MV_PANIC("class not found %s", name);
  }
  Object* o = it->second->create();
  return o;
}

