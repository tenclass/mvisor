#include "logger.h"
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <cerrno>
#include <cstdlib>
#include <cstdint>

void Log(LogType type, const char* file, int line, const char* function, const char* format, ...)
{
  char message[512];
  va_list args;
  va_start(args, format);
  vsnprintf(message, 512, format, args);
  va_end(args);

  time_t now = time(NULL);
  struct tm* tm_now;
  char timestr[100];
  tm_now = localtime(&now);
  strftime(timestr, 100, "%Y-%m-%d %H:%M:%S", tm_now);

  if (type == kLogTypeDebug) {
    printf("[%s] %s:%d %s() %s\n", timestr, file, line, function, message);
  } else if (type == kLogTypeError) {
    fprintf(stderr,"[%s] %s:%d %s() %s\n", timestr, file, line, function, message);
  } else if (type == kLogTypePanic) {
    fprintf(stderr,"[%s] %s:%d %s() fatal error: %s\n", timestr, file, line, function, message);
    if (errno != 0) {
      fprintf(stderr, "errno=%d, %s\n", errno, strerror(errno));
    }
    exit(1);
  }
}

void SaveToFile(const char* path, void* data, size_t size) {
  FILE* fp = fopen(path, "wb");
  fwrite(data, size, 1, fp);
  fclose(fp);
}

void DumpHex(void* data, size_t size) {
  uint8_t* ptr = (uint8_t*)data;
  printf("%08x  ", 0);
  for (int i = 0; i < (int)size;) {
    printf("%02x ", ptr[i++]);
    if (i % 16 == 0) {
      printf("\n%08x  ", i);
    } else if (i % 8 == 0) {
      printf(" ");
    }
  }
  if (size % 16 != 0) {
    printf("\n");
  }
}
