#ifndef _MVISOR_UTILITY_H
#define _MVISOR_UTILITY_H

class Object;
typedef Object* (*ClassCreator) (void);

struct ClassItem {
  int class_type;
  const char* class_name;
  const char* class_file;
  ClassCreator create;
};

#include "logger.h"
/* Initialize device classes and add to device management for later use */
void register_class(int type, const char* name, const char* source_path, ClassCreator create);
Object* realize_class(const char* name);

#define __register_class(cb, type)	\
static Object* __create__##cb() { \
  auto o = new cb; \
  o->set_name(#cb); \
  return o; \
} \
static void __attribute__ ((constructor)) __init__##cb(void) \
{	\
  register_class(type, #cb, __FILE__, __create__##cb); \
}

#endif // _MVISOR_UTILITY_H
