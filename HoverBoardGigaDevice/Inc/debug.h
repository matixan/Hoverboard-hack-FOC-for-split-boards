#include <string.h>


#define SERIAL_DEBUG 1

#if SERIAL_DEBUG < 1
#define DEBUG_println(...) 
#define DEBUG_print(...) 
#define DEBUG_printf(...) 
#else
#define DEBUG_printf(...) debug_printf(__VA_ARGS__)
#define DEBUG_print(arg) debug_print(arg)
#define DEBUG_println(arg) debug_println(arg)
#endif

size_t debug_printf(const char * format, ...)  __attribute__ ((format (printf, 1, 2)));
void sendDebug(const char* buffer, size_t n);
void debug_print(const char* buffer);
void debug_println(const char* buffer);


#define FST (const char *)
#define PROGMEM

