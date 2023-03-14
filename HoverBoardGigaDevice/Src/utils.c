#include "utils.h"
#include "debug.h"

uint8_t get_crc(const uint8_t* buffer, size_t size) {
    uint8_t crc = 0xFF;
    while (size--) {
        crc += *buffer++;
    }
    return crc;
}


// Copy string. (Returns len as opposed to strcpy
size_t s2s(char* buffer, const char* source) {
    size_t n = 0;
    while (source[n]) {
        buffer[n] = source[n];
        n++;
    }
    buffer[n] = '\0';
    return n;
}


// Reverses a string
void reverse_string(char* buffer, size_t len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = buffer[i];
        buffer[i] = buffer[j];
        buffer[j] = temp;
        i++;
        j--;
    }
}



// Integer to string
size_t i2s(char* buffer, int num, size_t s) {
    size_t n = 0;
    if (num == 0) { buffer[n++] = '0'; }
    else {
        bool sign = num < 0;
        if (sign) { num = -num; }
        while (num) {
            buffer[n++] = (num % 10) + '0';
            num = num / 10;
        }
        if (sign) { buffer[n++] = '-'; }
    }
  
    // Pad string as necessary
    while (n < s) { buffer[n++] = ' '; }
  
    reverse_string(buffer, n);
    buffer[n] = '\0';
    return n;
}
  

// Float to string
size_t f2s(char* buffer, float num, size_t decimals) {
    int32_t ip = (int32_t) num;
    size_t n = 0;
    if (ip==0 && num < 0.0) { buffer[n++] = '-'; buffer[n++] = '0'; }
    else { 
        n = i2s(buffer, ip, 0); 
        num -= (float)ip;
    }
    if (num < 0.0) { num = -num; }
    if (decimals == 0) { return n; }
    buffer[n++] = '.';
    while (decimals--) {
        num = num * 10.0;
        ip = (int32_t) num;
        num -= ip;
        buffer[n++] = ip + '0';
    }
    buffer[n] = '\0';
    return n;
}

/*
    char bb[64];
    i2s(bb, 0, 0); DEBUG_println(bb);
    i2s(bb, 12345, 0); DEBUG_println(bb);
    i2s(bb, -12345, 0); DEBUG_println(bb);
    i2s(bb, 12345, 9); DEBUG_println(bb);
    i2s(bb, -12345, 9); DEBUG_println(bb);
    f2s(bb, 12345.0, 3); DEBUG_println(bb);
    f2s(bb, -12345.0, 3); DEBUG_println(bb);
    f2s(bb, 12345.98765, 3); DEBUG_println(bb);
    f2s(bb, -12345.98765, 3); DEBUG_println(bb);
    f2s(bb, 12345.98765, 0); DEBUG_println(bb);
    f2s(bb, -12345.98765, 0); DEBUG_println(bb);
    f2s(bb, 0.98765, 3); DEBUG_println(bb);
    f2s(bb, -0.98765, 3); DEBUG_println(bb);
    f2s(bb, 0.003, 3); DEBUG_println(bb);
    f2s(bb, -0.003, 3); DEBUG_println(bb);
    f2s(bb, 0.0, 3); DEBUG_println(bb);
    f2s(bb, -0.0, 0); DEBUG_println(bb);
*/