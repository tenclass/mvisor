/*
   Copyright (C) 2012 Red Hat, Inc.

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

#ifndef H_SPICE_COMMON_LOG
#define H_SPICE_COMMON_LOG

#include <stdarg.h>
#include <stdio.h>
#include <glib.h>
#include <spice/macros.h>

#include "macros.h"

SPICE_BEGIN_DECLS

#ifdef SPICE_LOG_DOMAIN
#error Do not use obsolete SPICE_LOG_DOMAIN macro, is currently unused
#endif

#define SPICE_STRLOC  __FILE__ ":" G_STRINGIFY (__LINE__)

void spice_log(GLogLevelFlags log_level,
               const char *strloc,
               const char *function,
               const char *format,
               ...) G_GNUC_PRINTF(4, 5);

/* FIXME: name is misleading, this aborts.. */
#define spice_return_if_fail(x) G_STMT_START {                          \
    if G_LIKELY(x) { } else {                                           \
        spice_critical("condition `%s' failed", #x);                    \
        return;                                                         \
    }                                                                   \
} G_STMT_END

/* FIXME: name is misleading, this aborts.. */
#define spice_return_val_if_fail(x, val) G_STMT_START {                 \
    if G_LIKELY(x) { } else {                                           \
        spice_critical("condition `%s' failed", #x);                    \
        return (val);                                                   \
    }                                                                   \
} G_STMT_END

#define spice_warn_if_reached() G_STMT_START {                          \
    spice_log(G_LOG_LEVEL_WARNING, SPICE_STRLOC, __FUNCTION__, "should not be reached"); \
} G_STMT_END

#define spice_info(...) G_STMT_START {                                  \
    spice_log(G_LOG_LEVEL_INFO, SPICE_STRLOC, __FUNCTION__, "" __VA_ARGS__); \
} G_STMT_END

#define spice_debug(...) G_STMT_START {                                 \
    spice_log(G_LOG_LEVEL_DEBUG, SPICE_STRLOC, __FUNCTION__, "" __VA_ARGS__); \
} G_STMT_END

#define spice_warning(...) G_STMT_START {                               \
    spice_log(G_LOG_LEVEL_WARNING, SPICE_STRLOC, __FUNCTION__, "" __VA_ARGS__); \
} G_STMT_END

#define spice_critical(...) G_STMT_START {                              \
    spice_log(G_LOG_LEVEL_CRITICAL, SPICE_STRLOC, __FUNCTION__, "" __VA_ARGS__); \
    SPICE_UNREACHABLE;                                                  \
} G_STMT_END

#define spice_error(...) G_STMT_START {                                 \
    spice_log(G_LOG_LEVEL_ERROR, SPICE_STRLOC, __FUNCTION__, "" __VA_ARGS__); \
    SPICE_UNREACHABLE;                                                  \
} G_STMT_END

#define spice_warn_if_fail(x) G_STMT_START {            \
    if G_LIKELY(x) { } else {                           \
        spice_warning("condition `%s' failed", #x);     \
    }                                                   \
} G_STMT_END

#define spice_assert(x) G_STMT_START {                  \
    if G_LIKELY(x) { } else {                           \
        spice_error("assertion `%s' failed", #x);       \
    }                                                   \
} G_STMT_END

#if ENABLE_EXTRA_CHECKS
enum { spice_extra_checks = 1 };
#else
enum { spice_extra_checks = 0 };
#endif

#define spice_extra_assert(x) G_STMT_START {            \
    if (!spice_extra_checks || G_LIKELY(x)) { } else {  \
        spice_error("assertion `%s' failed", #x);       \
    }                                                   \
} G_STMT_END

SPICE_END_DECLS

#endif // H_SPICE_COMMON_LOG
