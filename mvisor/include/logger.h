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

#ifndef MVISOR_LOGGER_H
#define MVISOR_LOGGER_H

#include <stddef.h>

enum LogType {
  kLogTypeDebug,
  kLogTypeError,
  kLogTypePanic
};

#define MV_LOG(fmt, ...) Log(kLogTypeDebug, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)
#define MV_ERROR(fmt, ...) Log(kLogTypeError, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)
#define MV_PANIC(fmt, ...) Log(kLogTypePanic, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)
#define MV_ASSERT(condition) \
  (__builtin_expect(!!(condition), 1) ? (void)0 : MV_PANIC("Assertion failed, "#condition))

void Log(LogType type, const char* file, int line, const char* function, const char* format, ...);
void SaveToFile(const char* path, void* data, size_t size);
void DumpHex(void* data, size_t size);
void PrintRegisters(struct kvm_regs& regs, struct kvm_sregs& sregs);
void SetThreadName(const char* name);

#endif // MVISOR_LOGGER_H
