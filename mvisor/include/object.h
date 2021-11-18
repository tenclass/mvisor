#ifndef _MVISOR_OBJECT_H
#define _MVISOR_OBJECT_H

class Object {
 public:
  Object();
  virtual ~Object();

  const char* name();
  void set_name(const char* name);

 protected:
  char name_[100];
};

#endif // _MVISOR_OBJECT_H

