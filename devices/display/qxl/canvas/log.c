/*
   Copyright (C) 2012-2015 Red Hat, Inc.

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

#include "config.h"

#include <glib.h>
#include <stdlib.h>

#include "log.h"

G_GNUC_PRINTF(5, 0)
static void spice_logv(const char *log_domain,
                       GLogLevelFlags log_level,
                       const char *strloc,
                       const char *function,
                       const char *format,
                       va_list args)
{
    GString *log_msg;

    log_msg = g_string_new(NULL);
    if (strloc && function) {
        g_string_append_printf(log_msg, "%s:%s: ", strloc, function);
    }
    if (format) {
        g_string_append_vprintf(log_msg, format, args);
    }
    g_log(log_domain, log_level, "%s", log_msg->str);
    g_string_free(log_msg, TRUE);

    if ((log_level & G_LOG_LEVEL_CRITICAL) != 0) {
        abort();
    }
}

void spice_log(GLogLevelFlags log_level,
               const char *strloc,
               const char *function,
               const char *format,
               ...)
{
    va_list args;

    va_start (args, format);
    spice_logv (G_LOG_DOMAIN, log_level, strloc, function, format, args);
    va_end (args);
}
