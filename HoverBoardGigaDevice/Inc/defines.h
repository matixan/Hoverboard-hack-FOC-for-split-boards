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

#ifndef DEFINES_H
#define DEFINES_H

#include "gd32f1x0.h"
#include "../Inc/config.h"

//#include "boards/Hover1_defines.h"
//#include "boards/YK201A_defines.h"
#include "boards/TTSD22_defines.h"

// Useful math function defines
#define ABS(a) (((a) < 0.0) ? -(a) : (a))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define MAX(x, high) (((x) > (high)) ? (high) : (x))
#define MIN(x, low) (((x) < (low)) ? (low) : (x))
#define MAP(x, xMin, xMax, yMin, yMax) ((x - xMin) * (yMax - yMin) / (xMax - xMin) + yMin)

// ADC buffer struct
typedef struct
{
  uint16_t v_batt;
	uint16_t current_dc;
	uint16_t phase_a;
	uint16_t phase_b;
  uint16_t mcu_temp;
} adc_buf_t;


/*
typedef struct {
  uint16_t dcr; 
  uint16_t dcl; 
  uint16_t rlA;
  uint16_t rlB;
  uint16_t rrB;
  uint16_t rrC;
  uint16_t batt1;
  uint16_t l_tx2;
  uint16_t temp;
  uint16_t l_rx2;
} adc_buf_t;
*/


#endif
