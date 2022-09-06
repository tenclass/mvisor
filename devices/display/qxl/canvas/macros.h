/* -*- Mode: C; c-basic-offset: 4; indent-tabs-mode: nil -*- */
/*
   Copyright (C) 2009 Red Hat, Inc.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef H_SPICE_COMMON_MACROS
#define H_SPICE_COMMON_MACROS

#include "verify.h"

#ifdef __GNUC__
#define SPICE_CONSTRUCTOR_FUNC(func_name) \
    static void __attribute__((constructor)) func_name(void)
#define SPICE_DESTRUCTOR_FUNC(func_name) \
    static void __attribute__((destructor)) func_name(void)
#elif defined(_MSC_VER)
#define SPICE_CONSTRUCTOR_FUNC(func_name) \
    static void func_name(void); \
    static int func_name ## _wrapper(void) { func_name(); return 0; } \
    __pragma(section(".CRT$XCU",read)) \
    __declspec(allocate(".CRT$XCU")) static int (* _array ## func_name)(void) = func_name ## _wrapper; \
    static void func_name(void)
#define SPICE_DESTRUCTOR_FUNC(func_name) \
    static void func_name(void); \
    static int func_name ## _wrapper(void) { func_name(); return 0; } \
    __pragma(section(".CRT$XPU",read)) \
    __declspec(allocate(".CRT$XPU")) static int (* _array ## func_name)(void) = func_name ## _wrapper; \
    static void func_name(void)
#else
#error Please implement SPICE_CONSTRUCTOR_FUNC and SPICE_DESTRUCTOR_FUNC for this compiler
#endif

#define SPICE_VERIFY(cond) verify_expr(cond, (void)1)

#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 5)
#define SPICE_UNREACHABLE __builtin_unreachable()
#elif defined(_MSC_VER)
#define SPICE_UNREACHABLE __assume(0)
#else
#define SPICE_UNREACHABLE for(;;) continue
#endif

#endif // H_SPICE_COMMON_MACROS
