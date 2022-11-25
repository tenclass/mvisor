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

#ifndef _MVISOR_UTILITY_H
#define _MVISOR_UTILITY_H

#include <unistd.h>
#include <stdio.h>
#include <string>

class Object;
typedef Object* (*ClassCreator) (void);

struct ClassItem {
  int class_type;
  const char* class_name;
  const char* class_file;
  ClassCreator create;
};

/* Initialize device classes and add to device management for later use */
void register_class(int type, const char* name, const char* source_path, ClassCreator create);
Object* realize_class(const char* name);
std::string get_class_alias(const char* name);

#define __register_class(cb, type) \
static Object* __create__##cb() { \
  auto o = new cb; \
  return o; \
} \
static void __attribute__ ((constructor)) __init__##cb(void) \
{ \
  register_class(type, #cb, __FILE__, __create__##cb); \
}


/* Use this macro at the end of .cc source file to declare your device */
#define DECLARE_DEVICE(classname)       __register_class(classname, 2)
#define DECLARE_NETWORK(classname)      __register_class(classname, 3)
#define DECLARE_DISK_IMAGE(classname)   __register_class(classname, 4)

/* Close fd and set to -1 */
static inline void safe_close(int *fd) {
  if (*fd == 0) {
    fprintf(stderr, "unable to close stdout\n");
    return;
  } else if (*fd > 0) {
    close(*fd);
    *fd = -1;
  }
}

/* Test if buffer is all zero.
 * buffer must be 256 bytes alignment */
bool test_zero(const void* buffer, size_t length);

/* Z standard compress / decompress
 * https://github.com/facebook/zstd
 */
ssize_t zstd_decompress(const void* src,  size_t src_size, void* dest, size_t dest_size);

#endif // _MVISOR_UTILITY_H
