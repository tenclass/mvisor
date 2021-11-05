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
  ((condition) ? (void)0 : MV_PANIC("Assertion failed, "#condition))

void Log(LogType type, const char* file, int line, const char* function, const char* format, ...);
void SaveToFile(const char* path, void* data, size_t size);
void DumpHex(void* data, size_t size);
void PrintRegisters(struct kvm_regs& regs, struct kvm_sregs& sregs);
void SetThreadName(const char* name);

#endif // MVISOR_LOGGER_H
