/*
  The same command line interface is used for human friendly ASCII comands and binary commands.
  This means the CLI for control and proxy serial port are the same as the CLI supports both at the
  same time.
 */
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>
#include "defines.h"
#include "config.h"
#include "cli.h"
#include "bldc.h"
#include "pid.h"
#include "utils.h"
#include "comms.h"
#include "debug.h"

#include "steerAngle.h" 

uint32_t millis();

#define BINARY_WRITE_MAGIC 0xAA
#define BINARY_READ_MAGIC 0xA5
#define BINARY_ACK_MAGIC 42
#define BINARY_NACK_MAGIC 23
#define BINARY_RESPONSE_MAGIC 0x55

CliPort control_cli = {
    USART0,
    "",
    "",
    "",
    0,
    0,
    0,
    0,
    nullptr
};

CliPort proxy_cli = {
    USART1,
    "",
    "",
    "",
    0,
    0,
    0,
    0,
    nullptr
};

typedef struct ProxyReadRequest {
  uint32_t ts;
  size_t pn;
  bool is_binary;
  int target;
} ProxyReadRequest;

#ifndef PROXY_READ_QUEUE_SIZE
#define PROXY_READ_QUEUE_SIZE 8
#endif
ProxyReadRequest read_queue[PROXY_READ_QUEUE_SIZE] = {0};
size_t prq_write = 0;
size_t prq_read = 0;
size_t prq_count = 0;


static const char OK_TEXT[] PROGMEM = "OK";

static const char TYPE_TEXT[][6] PROGMEM = {"bool", "int", "float", "str"};


extern uint8_t bldc_enable;
extern float batt_u_calibrated;
extern float batt_percent;


const PConfig parameters[] PROGMEM = {
  {FST("mct"), FST("motor control type: 0=power 1=speed 3=angle"), pt_int, 0, &control_type, changeControlMode},
  {FST("pwm"), FST("target PWM"), pt_float, 0, &target_pwm, nullptr},
  {FST("pwm_cr"), FST("PWM change rate"), pt_float, 6, &max_pwm_change_rate, nullptr},
  {FST("pwm_max"), FST("max PWM value"), pt_float, 0 , &max_pwm, nullptr},
  {FST("ts"), FST("target_speed"), pt_float, 1, &speedPid.set_point, nullptr},
  {FST("spid_p"), FST("speed PID P gain"), pt_float, 3, &speedPid.p_gain, nullptr},
  {FST("spid_i"), FST("speed PID I gain"), pt_float, 3, &speedPid.i_gain, nullptr},
  {FST("spid_d"), FST("speed PID D gain"), pt_float, 3, &speedPid.d_gain, nullptr},
  {FST("ta"), FST("target_angle"), pt_float, 1, &anglePid.set_point, nullptr},
  {FST("apid_p"), FST("angle PID P gain"), pt_float, 3, &anglePid.p_gain, nullptr},
  {FST("apid_i"), FST("angle PID I gain"), pt_float, 3, &anglePid.i_gain, nullptr},
  {FST("apid_d"), FST("angle PID D gain"), pt_float, 3, &anglePid.d_gain, nullptr},
  {FST("pos"), FST("current position in degree"), pt_int, 3, &wheel_angle, nullptr},
  {FST("rpm"), FST("current speed in RPM"), pt_float, 3, &rpm_filtered, nullptr},
  {FST("mpcr"), FST("max PWM change rate"), pt_float, 3, &max_pwm_change_rate, nullptr},
  {FST("rev"), FST("reverse motor"), pt_bool, 0, &reverse_motor, nullptr},
  {FST("batt"), FST("battery voltage"), pt_float, 2, &batt_u_calibrated, nullptr},
  {FST("battp"), FST("battery percentage"), pt_float, 2, &batt_percent, nullptr},
  {FST("temp"), FST("board (MCU) temperature"), pt_float, 2, &board_temp_c, nullptr},
  {FST("en"), FST("BLDC enable"), pt_bool, 0, &bldc_enable, nullptr},
  {FST("error"), FST("system error"), pt_int, 0, &system_error, nullptr},
  //{FST("motor"), FST("enable/disable motor"), pt_bool, 0, nullptr, controlServo},
};
static const size_t MAX_PARAMETER = sizeof(parameters)/sizeof(PConfig);

//set <left> <right>              Set left and right set point\r\n\

const char* help_text = FST("\
set <left> <right>              Set left and right set point\r\n\
debug [0|1|on|off|true|false]   Enable or disable debug logging\r\n\
help                            Print help\r\n\
?                               Print help\r\n\
\r\n");


// Receive UART bytes from interrupt into circular buffer
// As this is executed from interrupt it must return ASAP
void cliReceive(CliPort* cli, char c) {
    if (cli->rx_count >= CLI_RX_BUFFER_SIZE) { return; }
    cli->rx_buffer[cli->rx_write_index++] = c;
    if (cli->rx_write_index >= CLI_RX_BUFFER_SIZE) { cli->rx_write_index = 0; }
    cli->rx_count++;
}

size_t cliPrint(CliPort* cli, const char* buffer) {
  size_t n = strlen(buffer);  
  SendBuffer(cli->serial_port, buffer, n);
  return n;
}

size_t cliPrintln(CliPort* cli, const char* buffer) {
  size_t n = 0;
  if (buffer) {
    n = strlen(buffer);  
    SendBuffer(cli->serial_port, buffer, n);
  }
  SendBuffer(cli->serial_port, "\r\n", 2);
  return n+1;
}

/* Printf uses too much memory
size_t cliPrintf(CliPort* cli, const char* format, ...) {
  char buffer[128];
  va_list args;
  va_start(args, format);
  size_t n = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  SendBuffer(cli->serial_port, buffer, n);
  return n;
}
*/

void cliRun() {
  if (prq_count) {
    if (read_queue[prq_read].ts + 100 < millis() ) {
      DEBUG_println(FST("Proxy Read Timeout"));
      prq_read++;
      if (prq_read >= PROXY_READ_QUEUE_SIZE) { prq_read = 0; }
      prq_count--;
      setError(EC_PROXY_READ_TIMEOUT);
    }
  }
  cliRunPort(&control_cli);
  cliRunPort(&proxy_cli);
}

char old_c = 0;

void cliRunPort(CliPort* cli) {
  while (cli->rx_count) {
    char c;
    {
        __disable_irq();
        c = cli->rx_buffer[cli->rx_read_index++];
        if (cli->rx_read_index >= CLI_RX_BUFFER_SIZE) { cli->rx_read_index = 0; }
        cli->rx_count--;
        __enable_irq();
    }
    cli->command[cli->command_index++] = c;

    if (c==0xDE && old_c==0xDE) {
      debug_println("RR");
    }
    old_c = c;

    char fc = cli->command[0];
    if ((fc & 0xF0) == (BINARY_CONTROL_MAGIC & 0xF0)) {
      if (handleRemoteData(cli->command, cli->command_index)) {
        cli->command_index = 0;
      }
    }
    else if (fc == BINARY_WRITE_MAGIC || fc == BINARY_READ_MAGIC) { // Binary command
        if (cli->command_index > 3 && cli->command_index == (cli->command[2] + 4)){
            cli->command_index = 0;
            CliResult status = cliExecuteBinary(cli, cli->command);
            char resp[2];
            if (status != CR_OK) {
                resp[0] = BINARY_NACK_MAGIC;
                resp[1] = status;
                SendBuffer(USART1, resp, 2);
            } else if (fc != BINARY_READ_MAGIC) {
                resp[0] = BINARY_ACK_MAGIC;
                SendBuffer(USART1, resp, 1);
            }
        }
        
    } else if (fc == BINARY_RESPONSE_MAGIC) {
      if (cli->command_index > 3 && cli->command_index == (cli->command[2] + 4)) {
        cli->command_index = 0;
        CliResult cr = cliHandleResponse(cli, cli->command);
        DEBUG_printf(FST("Got Response - %d\n"), cr);
      }
    } else if (fc == BINARY_ACK_MAGIC) {
      //DEBUG_println(FST("ACK"));
      cli->command_index = 0;
    } else if (fc == BINARY_NACK_MAGIC) {
      if (cli->command_index == 2) {
        DEBUG_printf(FST("NACK Error: %d\n"), cli->command[1]);
        cli->command_index = 0;
      }
    } else if (fc=='\r' || fc=='\n' || fc=='?' || (fc>='A' && fc<='Z') || (fc>='a' && fc<='z')) { // ASCII command
        if (c=='\r' || c=='\n' || c=='?'  || c==' '  || c=='.' || c==',' || c=='+' || c=='-' || (c>='A' && c<='Z') || (c>='a' && c<='z')  || (c>='0' && c<='9')) {
          if (c == '\n' || c == '\r') { 
              cli->command[cli->command_index-1] = '\0';
              cli->command_index = 0;
              DEBUG_print(FST("Execute:"));
              DEBUG_println(cli->command);
              DEBUG_println(cliExecute(cli, cli->command));
          }
        } else {
          cli->command_index = 0;
        }
    } else {
      cli->command_index = 0;
    }
    if (cli->command_index > COMMAND_BUFFER_SIZE-1) {
        cli->command_index = 0;
    }
  }
}

void proxy_value(size_t pn, uint32_t value, uint8_t magic) {
    uint8_t buffer[8];
    buffer[0] = magic;
    buffer[1] = pn;
    buffer[2] = 4;  // Size of value
    buffer[3] = value >> 24;
    buffer[4] = value >> 16;
    buffer[5] = value >>  8;
    buffer[6] = value;
    buffer[7] = get_crc(buffer, 7);
    SendBuffer(USART1, (char*)buffer, sizeof(buffer));
}

void proxy_read(size_t pn, int target,  bool is_binary) {
    if (prq_count >= PROXY_READ_QUEUE_SIZE) {
      DEBUG_println(FST("Proxy Read Queue is full!"));
    }
    read_queue[prq_write].ts = millis();
    read_queue[prq_write].pn = pn;
    read_queue[prq_write].target = target;
    read_queue[prq_write].is_binary = is_binary;
    prq_write++;
    if (prq_write >= PROXY_READ_QUEUE_SIZE) { prq_write = 0; }
    prq_count++;
    uint8_t buffer[4];
    buffer[0] = BINARY_READ_MAGIC;
    buffer[1] = pn;
    buffer[2] = 0;  // Size of value
    buffer[3] = get_crc(buffer, 3);
    SendBuffer(USART1, (char*)buffer, sizeof(buffer));
}


CliResult cliHandleResponse(CliPort* cli, const char* cmd) {
  // DEBUG_print(FST("Binary: ")); DEBUG_printInt(cli->command[1]);
  cli->error = nullptr;
  size_t pn = cmd[1];
  if (pn >= MAX_PARAMETER) {
    return CR_UNKNOWN_CMD;    
  }
  if (prq_count == 0 || read_queue[prq_read].pn != pn) {
    prq_read = prq_write;
    prq_count = 0;
    return CR_UNEXPECTED_RESPONSE;    
  }
  uint32_t value = 0;
  if (cmd[2] <= 4) {
      for (size_t n=0; n<cmd[2]; n++) {
        value = (value << 8 ) | ((uint32_t)cmd[n+3]);
      }
  } else {
    return CR_BAD_VALUE;    
  }

  int target = read_queue[prq_read].target;
  const PConfig* p = &parameters[pn];
  size_t n = 0;
  switch ( p->type ) {
    case pt_bool: {
        if (!p->variable) { return nullptr; }
        if (target & 1) {
          n += i2s(cli->buffer, *((bool*)p->variable), p->fmt);
          cli->buffer[n++] = ',';
          cli->buffer[n++] = ' ';
        }
        n += i2s(cli->buffer+n, (bool)value, p->fmt);
        break;
    }

    case pt_int: {
        if (!p->variable) { return nullptr; }
        if (target & 1) {
          n += i2s(cli->buffer, *((int*)p->variable), p->fmt);
          cli->buffer[n++] = ',';
          cli->buffer[n++] = ' ';
        }
        n += i2s(cli->buffer+n, value, p->fmt);
        break;
    }

    case pt_float: {
        if (!p->variable) { return nullptr; }
        if (target & 1) {
          n += f2s(cli->buffer, *((float*)p->variable), p->fmt);
          cli->buffer[n++] = ',';
          cli->buffer[n++] = ' ';
        }
        n += f2s(cli->buffer+n, *((float*)&value), p->fmt);
        break;
    }

    default:
      cli->buffer[n++] = "B";
      cli->buffer[n++] = "A";
      cli->buffer[n++] = "D";
  }

  cli->buffer[n++] = '\n';
  cli->buffer[n] = '0';
  SendBuffer(control_cli.serial_port, cli->buffer, n);

  prq_read++;
  if (prq_read >= PROXY_READ_QUEUE_SIZE) { prq_read = 0; }
  prq_count--;
  return CR_OK;    
}

CliResult cliExecuteBinary(CliPort* cli, const char* cmd) {
  // DEBUG_print(FST("Binary: ")); DEBUG_printInt(cli->command[1]);
  cli->error = nullptr;
  if (cmd[0] != BINARY_WRITE_MAGIC && cmd[0] != BINARY_READ_MAGIC) {
    return CR_BAD_MAGIC;    
  }
  size_t pn = cmd[1];
  if (pn >= MAX_PARAMETER) {
    return CR_UNKNOWN_CMD;    
  }
  uint8_t crc = get_crc(cmd, cmd[2]+3);
  if (cmd[cmd[2]+3] != crc) {
    return CR_BAD_CRC;    
  }
  uint32_t value = 0;
  if (cmd[2] <= 4) {
      for (size_t n=0; n<cmd[2]; n++) {
        value = (value << 8 ) | ((uint32_t)cmd[n+3]);
      }
  } else {
    return CR_BAD_VALUE;    
  }
  
  const PConfig* p = &parameters[pn];
  if (cmd[0] == BINARY_READ_MAGIC) { // Read
    if (p->variable) {
        switch ( p->type ) {
        case pt_bool: {
            uint32_t tmp = *((bool*)p->variable);
            proxy_value(pn, tmp, BINARY_RESPONSE_MAGIC);
            break;
        }

        case pt_int: {
            int32_t tmp = *((int32_t*)p->variable);
            proxy_value(pn, *((uint32_t*)&tmp), BINARY_RESPONSE_MAGIC);
            break;
        }

        case pt_float: {
            float tmp = *((float*)p->variable);
            proxy_value(pn, *((uint32_t*)&tmp), BINARY_RESPONSE_MAGIC);
          break;
        }
      }
    }
  } else { // Write
      switch ( p->type ) {
      case pt_bool: {
          setBoolPar(pn, 1, value);
          break;
      }

      case pt_int: {
          setIntPar(pn, 1, value);
          break;
      }

      case pt_float: {
        setFloatPar(pn, 1, *((float*)&value));
        break;
      }
    }
  }
  return CR_OK;
}

const char* cliExecute(CliPort* cli, const char* cmd) {
  cli->error = nullptr;
  size_t n = 0;
  int target = 1;
  if (*cmd == '#') { return FST(""); } // Comment line
  if (tolower(cmd[0]) == 'l' && cmd[1] == ' ') { target = 1; cmd += 2; } // Command for local
  if (tolower(cmd[0]) == 'r' && cmd[1] == ' ') { target = 2; cmd += 2; } // Command for remote
  if (tolower(cmd[0]) == 'b' && cmd[1] == ' ') { target = 3; cmd += 2; } // Command for both
  if ((n = tryRead(cli, FST("HELP"), cmd))) { return help(cli); }
  if ((n = tryRead(cli, FST("?"), cmd))) { return help(cli); }
  //if ((n = tryRead(cli, FST("DEBUG"), cmd))) { return controlDebug(cli, cmd+n); }
  if ((n = tryRead(cli, FST("RESTART"), cmd))) { return restart(cli); }
  if ((n = tryRead(cli, FST("SET"), cmd))) { return controlMotors(cli, cmd+n); }

  size_t pn = 0;
  while (pn < MAX_PARAMETER) {
    if ((n = tryRead(cli, parameters[pn].name, cmd))) { return controlParameter(cli, target,  pn, cmd+n); }
    pn++;
  }

  return cliSetError(cli, FST("invalid command"));
}

const char* help(CliPort* cli) {
  if (cli) {
    cliPrint(cli, "\n");
    char buffer[128];
    for (size_t i=0; i < MAX_PARAMETER; i++) {
        size_t n = s2s(buffer, FST("[L|R|B] "));
        n += s2s(buffer+n, parameters[i].name);
        buffer[n++] = ' '; buffer[n++] = '[';
        n += s2s(buffer+n, TYPE_TEXT[parameters[i].type]);
        buffer[n++] = ']';
        while(n<32) { buffer[n++] = ' '; }
        n += s2s(buffer+n, FST("Get/Set "));
        n += s2s(buffer+n, parameters[i].info);
        cliPrintln(cli, buffer);
    }
    cliPrint(cli, "\n");
    cliPrint(cli, help_text);
  }
  return OK_TEXT;  
}

/*
const char* controlDebug(CliPort* cli, const char* cmd) {
  bool state = false;
  if ( cmd[0] == '\0' ) {
    if (debugStream == nullptr) { 
      return FST("off");
    } else if (debugStream == &Serial) {
      return FST("serial");
    } else if (debugStream == stream) {
      return FST("here");
    } else {
      return FST("on");
    }
  }
  size_t n = readBool(cmd, &state);
  if (error) { return error; }
  debugStream = state ? stream : nullptr;
  return OK_TEXT;  
}
*/

const char* changeControlMode(void* ptr) {
  speedPid.integral = 0.0;
  speedPid.last_error = 0.0;
  anglePid.integral = 0.0;
  anglePid.last_error = 0.0;
  wheel_angle = 0;
  return 0;
}

const char* getParameter(CliPort* cli, int target, size_t pn) {
  const PConfig* p = &parameters[pn];
  if (target & 2) {
    proxy_read(pn, target, false);
    return "";
  }
  switch ( p->type ) {
    case pt_bool: {
        if (!p->variable) { return nullptr; }
        i2s(cli->buffer, *((bool*)p->variable), p->fmt);
        return cli->buffer;
    }

    case pt_int: {
        if (!p->variable) { return nullptr; }
        i2s(cli->buffer, *((int*)p->variable), p->fmt);
        return cli->buffer;
    }

    case pt_float: {
        if (!p->variable) { return nullptr; }
        f2s(cli->buffer, *((float*)p->variable), p->fmt);
        return cli->buffer;
    }

    default:
      return cliSetError(cli, FST("unknown parameter type not supported"));
  }
  return OK_TEXT;
}

const char* setBoolPar(size_t pn, int target, bool value) {
    const PConfig* p = &parameters[pn];
    if (target & 2) {
        proxy_value(pn, value, BINARY_WRITE_MAGIC);
    }
    if (target & 1) {
        if (p->variable) { *((bool*)p->variable) = value; }
        if (p->callback) { return (p->callback)(&value); }
    }
    return OK_TEXT;
}

const char* setIntPar(size_t pn, int target, int value) {
    const PConfig* p = &parameters[pn];
    if (target & 2) {
        proxy_value(pn, *((uint32_t*)&value), BINARY_WRITE_MAGIC);
    }
    if (target & 1) {
        if (p->variable) { *((int*)p->variable) = value; }
        if (p->callback) { return (p->callback)(&value); }
    }
    return OK_TEXT;
}

const char* setFloatPar(size_t pn, int target, float value) {
    const PConfig* p = &parameters[pn];
    if (target & 2) {
        proxy_value(pn, *((uint32_t*)&value), BINARY_WRITE_MAGIC);
    }
    if (target & 1) {
        if (p->variable) { *((float*)p->variable) = value; }
        if (p->callback) { return (p->callback)(&value); }
    }
    return OK_TEXT;
}

const char* controlParameter(CliPort* cli, int target, size_t pn, const char* cmd) {
  if ( cmd[0] == '\0' ) { return getParameter(cli, target, pn); }
  const PConfig* p = &parameters[pn];
  switch ( p->type ) {
    case pt_bool: {
      bool value = 0;
      readBool(cli, cmd, &value);
      if (cli->error) { return cli->error; }
      return setBoolPar(pn, target, value);
    }

    case pt_int: {
      int value = 0;
      readInteger(cli, cmd, &value);
      if (cli->error) { return cli->error; }
      return setIntPar(pn, target, value);
    }

    case pt_float: {
      float value = 0.0;
      readFloat(cli, cmd, &value);
      if (cli->error) { return cli->error; }
      return setFloatPar(pn, target, value);
    }

    default:
      return cliSetError(cli, FST("unknown parameter type not supported"));
  }
  return OK_TEXT;
}


const char* restart(CliPort* port) {
  return (char*)((int)port / 0);  
}

const char* controlMotors(CliPort* cli, const char* cmd) {
  size_t n = 0;
  float left = 0.0;
  n += readFloat(cli, cmd+n , &left);
  if (cli->error) { return cli->error; }
  float right = 0.0;
  n += readFloat(cli, cmd+n, &right);
  if (cli->error) { return cli->error; }

  switch (control_type) {
    case CT_PWM:
      target_pwm = right;
      break;

    case CT_SPEED:
      speedPid.set_point = right;
      break;

    case CT_ANGLE:
      anglePid.set_point = right;
      break;
  }
  remote_set_point = left;
  com_enabled = true;
  n = 0;
  char *b = cli->buffer;
  n += f2s(b+n, remote_speed, 2);
  b[n++] = ',';
  n += f2s(b+n, rpm_filtered, 2);
  b[n++] = ',';
  n += i2s(b+n, remote_position, 0);
  b[n++] = ',';
  n += i2s(b+n, wheel_angle, 0);
  b[n++] = '\n';
  b[n] = '\0';
  return b;  
}

/*
const char* steer_cmd(CliPort* port, const char* cmd) {
		

      float value = 0.0;
      readFloat(cli, cmd, &value);
      if (cli->error) { return cli->error; }
  return OK_TEXT;  
}

void steer(float speed, float steerAngle) {
		// Calculate expo rate for less steering with higher speeds
		//float expo = MAP((float)ABS(speed), 0, 1000, 1, 0.5);
    
		float xScale = lookUpTableAngle[(uint16_t)steerAngle];

		// Mix steering and speed value for right and left speed
		if(steerAngle >= 90)
		{
			pwmSlave = CLAMP(scaledSpeed, -1000, 1000);
			pwmMaster = CLAMP(pwmSlave / xScale, -1000, 1000);
		}
		else
		{
			pwmMaster = CLAMP(scaledSpeed, -1000, 1000);
			pwmSlave = CLAMP(xScale * pwmMaster, -1000, 1000);
		}
  return OK_TEXT;  
}
*/


bool isWhiteSpace(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r'; }
bool isWhiteSpaceOrEnd(char c) { return c == '\0' || c == ' ' || c == '\t' || c == '\n' || c == '\r'; }
const char* cliSetError(CliPort* cli, const char* text) { cli->error = text; return text; }

size_t readBool(CliPort* cli, const char* cmd, bool* result) {
  size_t n = 0; 
  if ((n = tryRead(cli, FST("0"), cmd))) { *result = false; return n; }
  if ((n = tryRead(cli, FST("false"), cmd))) { *result = false; return n; }
  if ((n = tryRead(cli, FST("off"), cmd))) { *result = false; return n; }
  if ((n = tryRead(cli, FST("1"), cmd))) { *result = true; return n; }
  if ((n = tryRead(cli, FST("true"), cmd))) { *result = true; return n; }
  if ((n = tryRead(cli, FST("on"), cmd))) { *result = true; return n; }
  cliSetError(cli, FST("invalid boolean"));
  return 0;
}

size_t readInteger(CliPort* cli, const char* cmd, int* result) {
  size_t n = 0;
  int tmp = 0;
  bool is_negative = false;
  if (cmd[n] == '-') {
    is_negative = true;
    n++;
  }
  if (cmd[n] == '0' && cmd[n+1] == 'b') { return readBinary(cli, cmd+n+2, result, is_negative); }
  if (cmd[n] == '0' && cmd[n+1] == 'x') { return readHex(cli, cmd+n+2, result, is_negative); }

  while(!isWhiteSpaceOrEnd(cmd[n])) {
    char c = cmd[n]; 
    if (c < '0' || c > '9') { 
      cliSetError(cli, FST("invalid integer"));
      return 0; 
    }
    tmp = tmp * 10 + (c - '0');
    n++;
  }
  if (is_negative) { tmp = -tmp; }
  while (isWhiteSpace(cmd[n])) { n++; } 
  *result = tmp;
  return n;
}

size_t readFloat(CliPort* cli, const char* cmd, float* result) {
  size_t n = 0;
  float tmp = 0;
  float dec = 0.0;
  bool is_negative = false;
  if (cmd[n] == '-') {
    is_negative = true;
    n++;
  }

  while(!isWhiteSpaceOrEnd(cmd[n])) {
    char c = cmd[n]; 
    if (c == '.') { 
      if (dec) {
        cliSetError(cli, FST("invalid float"));
        return 0;  
      }
      dec = 0.1;
    } else if (c < '0' || c > '9') { 
      cliSetError(cli, FST("invalid float"));
      return 0; 
    } else {
      if (dec == 0) { tmp = tmp * 10.0 + (c - '0'); }
      else { 
        tmp += (float)(c - '0') * dec; 
        dec /= 10.0; 
      }
    }  
    n++;
  }
  if (is_negative) { tmp = -tmp; }
  while (isWhiteSpace(cmd[n])) { n++; } 
  *result = tmp;
  return n;
}

size_t readBinary(CliPort* cli, const char* cmd, int* result, bool is_negative) {
  size_t n = 0;
  int tmp = 0;
  if (!is_negative && cmd[n] == '-') {
    is_negative = true;
    n++;
  }
  while(!isWhiteSpaceOrEnd(cmd[n])) {
    char c = cmd[n]; 
    if (c < '0' || c > '1') { 
      cliSetError(cli, FST("invalid binary"));
      return 0; 
    }
    tmp = (tmp << 1) + (c - '0');
    n++;
  }
  if (is_negative) { tmp = -tmp; }
  while (isWhiteSpace(cmd[n])) { n++; } 
  *result = tmp;
  return n;
}

size_t readHex(CliPort* cli, const char* cmd, int* result, bool is_negative) {
  size_t n = 0;
  int tmp = 0;
  if (!is_negative && cmd[n] == '-') {
    is_negative = true;
    n++;
  }
  while(!isWhiteSpaceOrEnd(cmd[n])) {
    char c = toupper(cmd[n]);
    if (c >= '0' && c <= '9') { 
      tmp = (tmp << 4) + (c - '0');    
    } else if (c >= 'A' && c <= 'F') {
      tmp = (tmp << 4) + (c - 'A' + 10);    
    } else {
      cliSetError(cli, FST("invalid hex"));
      return 0; 
    }
    n++;
  }
  if (is_negative) { tmp = -tmp; }
  while (isWhiteSpace(cmd[n])) { n++; } 
  *result = tmp;
  return n;
}

size_t readWord(CliPort* cli, const char* cmd, char* result, size_t size) {
  size_t n = 0;
  size_t i = 0;
  while (isWhiteSpace(cmd[n])) { n++; }
  while(!isWhiteSpaceOrEnd(cmd[n])) {
    if (i < (size-1)) { result[i++] = cmd[n]; }
    n++;
  }
  result[i] = '\0';
  while (isWhiteSpace(cmd[n])) { n++; } 
  return n;
}


size_t tryRead(CliPort* cli, const char* str, const char* cmd) {
  size_t n = 0;
  while (true) {
    if (str[n] == '\0') {
      if (!isWhiteSpaceOrEnd(cmd[n])) {
        return 0;
      }
      while (isWhiteSpace(cmd[n])) { n++; } 
      return n;
    }
    if (tolower(cmd[n]) != tolower(str[n])) { return 0; }    
    n++;
  }
  return n;
}
