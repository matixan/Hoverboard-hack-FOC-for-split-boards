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

#include "gd32f1x0.h"
#include "defines.h"
#include "comms.h"
#include "it.h"
#include "utils.h"
#include "bldc.h"
#include "pid.h"
#include "debug.h"

bool com_enabled = false;
uint32_t com_sent_ts = 0;
uint8_t com_seq = 0;
bool com_waiting = false;
uint32_t com_rec_ts = 0;

float remote_set_point = 0.0;
long remote_position = 0.0;
float remote_speed = 0.0;

#define REMOTE_TIME 20

void remoteRun() {
  if (!com_enabled || system_error) { return; }
  uint32_t now = millis();

  #ifdef MASTER
  // Send control messages at 50Hz
  if (!com_waiting && now > com_sent_ts + REMOTE_TIME) { 
    com_sent_ts = now; 
    RemoteControlMessage rcm;
    rcm.magic = BINARY_REMOTE_CONTROL_MAGIC;
    rcm.seq = com_seq;
    rcm.control_type = control_type;
    rcm.set_point = remote_set_point;
    rcm.error_code = system_error;
    rcm.crc = get_crc(&rcm, sizeof(RemoteControlMessage)-1);
    SendBuffer(REMOTE_UART, &rcm, sizeof(RemoteControlMessage));
    com_waiting = true;
  }

  if (com_waiting && now > com_sent_ts + 30) {
    if (system_error != EC_COM_TIMEOUT) {
      DEBUG_printf(FST("Timeout waiting for remote status response %d\n"), com_seq);
      com_seq++;
      //setError(EC_COM_TIMEOUT);
    }
    com_waiting = false;
  }
  #endif // MASTER
  if (com_rec_ts && now > com_rec_ts + REMOTE_TIME * 90) {
    if (system_error != EC_COM_TIMEOUT) {
      DEBUG_println(FST("Comms timeout"));
      setError(EC_COM_TIMEOUT);
      com_enabled = false;
    }
  }
}

void handleRemoteControlMessage(RemoteControlMessage* mp) {
  if (mp->crc != get_crc(mp, sizeof(RemoteControlMessage)-1)) {
    return;
  }
  if (mp->error_code && mp->error_code != remote_system_error) {
    setRemoteError(remote_system_error);
  }
  if (control_type != mp->control_type) {
    control_type = mp->control_type;
    speedPid.integral = 0.0;
    speedPid.last_error = 0.0;
    anglePid.integral = 0.0;
    anglePid.last_error = 0.0;
    wheel_angle = 0;
  }
  switch (control_type) {
    case CT_PWM:
      target_pwm = mp->set_point;
      break;

    case CT_SPEED:
      speedPid.set_point = mp->set_point;
      break;

    case CT_ANGLE:
      anglePid.set_point = mp->set_point;
      break;
  }

  RemoteStatusMessage rsm;
  rsm.magic = BINARY_REMOTE_STATUS_MAGIC;
  rsm.seq = mp->seq;
  rsm.pos = wheel_angle;
  rsm.speed = rpm_filtered;
  rsm.error_code = system_error;
  rsm.crc = get_crc(&rsm, sizeof(RemoteStatusMessage)-1);
  SendBuffer(REMOTE_UART, &rsm, sizeof(RemoteStatusMessage));

  com_enabled = true;
  com_rec_ts = millis();
}

void handleStatusControlMessage(RemoteStatusMessage* mp) {
  //debug_printf("REC %d %d - %d\n", millis(), com_sent_ts, millis()-com_sent_ts);
  com_waiting = false;
  if (mp->crc != get_crc(mp, sizeof(RemoteStatusMessage)-1)) {
    DEBUG_println(FST("Bad Remote Status CRC"));
    return;
  }
  if (mp->error_code && mp->error_code != remote_system_error) {
    DEBUG_printf(FST("Remote Error: %d\n"), mp->error_code);
    setRemoteError(remote_system_error);
    bldc_enable = false;
  }
  if (mp->seq != com_seq) {
    DEBUG_println(FST("Bad Remote Status sequence"));
    return;
  }
  com_seq++;
  remote_position = mp->pos;
  remote_speed = mp->speed;
  com_rec_ts = millis();  
}


bool handleRemoteData(uint8_t* buffer, size_t size) {
  if (buffer[0] == BINARY_REMOTE_CONTROL_MAGIC) {
    if (size != sizeof(RemoteControlMessage)) { return false; }
    handleRemoteControlMessage((RemoteControlMessage*)buffer);
    return true;
  } else if (buffer[0] == BINARY_REMOTE_STATUS_MAGIC) {
    if (size != sizeof(RemoteStatusMessage)) { return false; }
    handleStatusControlMessage((RemoteStatusMessage*)buffer);
    return true;
  } 
  return false;
}


//----------------------------------------------------------------------------
// Send buffer via USART
//----------------------------------------------------------------------------

#if USART_TX_DMA

void SendBuffer(uint32_t usart_periph, const char buffer[], uint8_t length) {
	if(usart_periph == USART0)
	{
		/* Channel disable */
		dma_channel_disable(DMA_CH1);
		
		dma_memory_address_config(DMA_CH1, (uint32_t)buffer);
		dma_transfer_number_config(DMA_CH1, length);
		
		/* enable DMA channel to start send */
		dma_channel_enable(DMA_CH1);
	}
	else if(usart_periph == USART1)
	{
		/* Channel disable */
		dma_channel_disable(DMA_CH3);
		
		dma_memory_address_config(DMA_CH3, (uint32_t)buffer);
		dma_transfer_number_config(DMA_CH3, length);
		
		/* enable DMA channel to start send */
		dma_channel_enable(DMA_CH3);	
	}
}

#else

void SendBuffer(uint32_t usart_periph, const char buffer[], uint8_t length) {
	uint8_t index = 0;	
	for(; index < length; index++)
	{
    usart_data_transmit(usart_periph, buffer[index]);
    while (usart_flag_get(usart_periph, USART_FLAG_TC) == RESET) {}
	}
}

#endif

//----------------------------------------------------------------------------
// Calculate CRC
//----------------------------------------------------------------------------
uint16_t CalcCRC(uint8_t *ptr, int count)
{
  uint16_t  crc;
  uint8_t i;
  crc = 0;
  while (--count >= 0)
  {
    crc = crc ^ (uint16_t) *ptr++ << 8;
    i = 8;
    do
    {
      if (crc & 0x8000)
      {
        crc = crc << 1 ^ 0x1021;
      }
      else
      {
        crc = crc << 1;
      }
    } while(--i);
  }
  return (crc);
}
