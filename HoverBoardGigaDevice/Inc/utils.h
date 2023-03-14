#ifndef _UTILS_H_
#define _UTILS_H_

#include <stddef.h>
#include "gd32f1x0.h"

#ifndef nullptr
#define nullptr 0
#endif

#ifndef false
#define false 0
#endif

#ifndef true
#define true 1
#endif

size_t s2s(char* buffer, const char* source);
void reverse_string(char* buffer, size_t len);
size_t i2s(char* buffer, int num, size_t s);
size_t f2s(char* buffer, float num, size_t decimals);
uint8_t get_crc(const uint8_t* buffer, size_t size);

#endif // _UTILS_H_