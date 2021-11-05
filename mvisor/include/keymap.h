#ifndef _MVSIOR_KEY_MAP_H
#define _MVSIOR_KEY_MAP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
int TranslateScancode(uint8_t scancode, int pressed, uint8_t transcoded_size[10]);
#ifdef __cplusplus
}
#endif

#endif // _MVSIOR_KEY_MAP_H
 