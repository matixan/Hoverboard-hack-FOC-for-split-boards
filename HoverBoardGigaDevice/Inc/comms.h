/*
* This file is part of the hoverboard-firmware-hack-V2 project. The 
* firmware is used to hack the generation 2 board of the hoverboard.
* These new hoverboards have no mainboard anymore. They consist of 
* two Sensorboards which have their own BLDC-Bridge per Motor and an
* ARM Cortex-M3 processor GD32F130C8.
*
* Copyright (C) 2018 Florian Staeblein
* Copyright (C) 2018 Jakob Broemauer
* Copyright (C) 2018 Kai Liebich
* Copyright (C) 2018 Christoph Lehnert
*
* The program is based on the hoverboard project by Niklas Fauth. The 
* structure was tried to be as similar as possible, so that everyone 
* could find a better way through the code.
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
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef COMMS_H
#define COMMS_H

#include <stddef.h>
#include "gd32f1x0.h"
#include "config.h"

#define BINARY_CONTROL_MAGIC 0xB1
#define BINARY_STATUS_MAGIC 0xB2
#define BINARY_REMOTE_CONTROL_MAGIC 0xB3
#define BINARY_REMOTE_STATUS_MAGIC 0xB4

#ifndef REMOTE_UART
#define REMOTE_UART USART1
#endif

typedef struct __attribute__((__packed__)) RemoteControlMessage {
    uint8_t magic;
    uint8_t seq;
    uint8_t control_type;
    float set_point;
    uint8_t error_code;
    uint8_t crc;
} RemoteControlMessage;

typedef struct __attribute__((__packed__)) RemoteStatusMessage {
    uint8_t magic;
    uint8_t seq;
    int32_t pos;
    float speed;
    uint8_t error_code;
    uint8_t crc;
} RemoteStatusMessage;

void remoteRun();
bool handleRemoteData(uint8_t* buffer, size_t size);

//----------------------------------------------------------------------------
// Send buffer via USART
//----------------------------------------------------------------------------
void SendBuffer(uint32_t usart_periph, const char buffer[], uint8_t length);

//----------------------------------------------------------------------------
// Calculate CRC
//----------------------------------------------------------------------------
uint16_t CalcCRC(uint8_t *ptr, int count);

extern bool com_enabled;
extern float remote_set_point;
extern long remote_position;
extern float remote_speed;

#endif
