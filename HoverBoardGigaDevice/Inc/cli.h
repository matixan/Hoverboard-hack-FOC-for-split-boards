/*

 */

#ifndef CLI_H
#define CLI_H

#include "gd32f1x0.h"
#include <string.h>


#ifndef CLI_RX_BUFFER_SIZE
#define CLI_RX_BUFFER_SIZE 64
#endif

#ifndef COMMAND_BUFFER_SIZE
#define COMMAND_BUFFER_SIZE 64
#endif


typedef struct CliPort {
  uint32_t serial_port;
  volatile char rx_buffer[CLI_RX_BUFFER_SIZE];
  char command[COMMAND_BUFFER_SIZE];
  char buffer[64];
  volatile size_t rx_count;
  volatile size_t rx_write_index;
  size_t rx_read_index;
  size_t command_index;
  const char* error;
} CliPort;

typedef enum CliResult {
    CR_OK,
    CR_ERROR,
    CR_BAD_MAGIC,
    CR_BAD_CRC,
    CR_BAD_VALUE,
    CR_UNKNOWN_CMD,
    CR_UNEXPECTED_RESPONSE,
} CliResult;

typedef enum PType {pt_bool=0, pt_int, pt_float, pt_str} PType;

typedef struct PConfig {
    const char* name;
    const char* info;
    PType type;
    size_t fmt;
    void* variable;
    const char* (*callback)(void* variable);
} PConfig;

extern CliPort control_cli;
extern CliPort proxy_cli;

void cliReceive(CliPort* cli, char c);
void cliRun();
void cliRunPort(CliPort* cli);

const char* cliExecute(CliPort* cli, const char* cmd);
CliResult cliExecuteBinary(CliPort* cli, const char* cmd);
CliResult cliHandleResponse(CliPort* cli, const char* cmd);
bool isWhiteSpace(char c);
bool isWhiteSpaceOrEnd(char c);
const char* cliSetError(CliPort* cli, const char* text);

const char* help(CliPort* port);
const char* restart(CliPort* port);
const char* controlMotors(CliPort* port, const char* cmd);
const char* controlParameter(CliPort* cli, int target, size_t pn, const char* cmd);

const char* setBoolPar(size_t pn, int target, bool value);
const char* setIntPar(size_t pn, int target, int value);
const char* setFloatPar(size_t pn, int target, float value);

size_t readBool(CliPort* cli, const char* cmd, bool* result);
size_t readInteger(CliPort* cli, const char* cmd, int* result);
size_t readFloat(CliPort* cli, const char* cmd, float* result);
size_t readWord(CliPort* cli, const char* cmd, char* result, size_t size);
size_t readBinary(CliPort* cli, const char* cmd, int* result, bool is_negative);
size_t readHex(CliPort* cli, const char* cmd, int* result, bool is_negative);
size_t tryRead(CliPort* cli, const char* str, const char* cmd);

const char* changeControlMode(void* ptr);

#endif // CLI_H
