/*
* This file implements FOC motor control.
* This control method offers superior performanace
* compared to previous cummutation method. The new method features:
* ► reduced noise and vibrations
* ► smooth torque output
* ► improved motor efficiency -> lower energy consumption
*
* Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
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

#include "rtwtypes.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
//#include "util.h"

// Matlab includes and defines - from auto-code generation
// ###############################################################################
#include "bldc.h"
#include "BLDC_controller.h"           /* Model's header file */

extern RT_MODEL *const rtM_Left;

extern DW   rtDW_Left;                  /* Observable states */
extern ExtU rtU_Left;                   /* External inputs */
extern ExtY rtY_Left;                   /* External outputs */
extern P    rtP_Left;

// ###############################################################################

static int16_t pwm_margin;              /* This margin allows to have a window in the PWM signal for proper FOC Phase currents measurement */

extern uint8_t ctrlModReq;
static int16_t curDC_max = (I_DC_MAX * A2BIT_CONV);
int16_t curL_phaA = 0, curL_phaB = 0, curL_DC = 0;
// int16_t curR_phaB = 0, curR_phaC = 0, curR_DC = 0;


ErrorCode system_error = EC_OK;
ErrorCode remote_system_error = EC_OK;

#ifdef MASTER
bool reverse_motor = 0;
#else
bool reverse_motor = 1;
#endif

volatile int pwml = 0;
//volatile int pwmr = 0;

extern volatile adc_buf_t adc_buffer;

uint8_t buzzerFreq          = 0;
uint8_t buzzerPattern       = 0;
uint8_t buzzerCount         = 0;
volatile uint32_t buzzerTimer = 0;
static uint8_t  buzzerPrev  = 0;
static uint8_t  buzzerIdx   = 0;

uint8_t        bldc_enable  = 0;        // initially motors are disabled for SAFETY
uint8_t        bldc_enableFin  = 0;

static const uint16_t pwm_res  = 64000000 / 2 / PWM_FREQ; // = 2000

static uint16_t offsetcount = 0;
/*
static int16_t offsetrlA    = 2000;
static int16_t offsetrlB    = 2000;
static int16_t offsetrrB    = 2000;
static int16_t offsetrrC    = 2000;
static int16_t offsetdcl    = 2000;
static int16_t offsetdcr    = 2000;
*/
int16_t offset_current_dc    = 2000;
int16_t offset_v_batt    = 2000;
int16_t offset_phase_a    = 2000;
int16_t offset_phase_b    = 2000;

int16_t        batVoltage       = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
static int32_t batVoltageFixdt  = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE << 16;  // Fixed-point filter output initialized at 400 V*100/cell = 4 V/cell converted to fixed-point

int16_t board_temp_deg_c = 0;        // global variable for calibrated temperature in degrees Celsius
int32_t board_temp_adcFixdt = 0;     // Fixed-point filter output initialized with current ADC converted to fixed-point
int16_t board_temp_adcFilt = 0;
float board_temp_c = 0.0;


int16_t odom_l = 0;

static uint16_t wp_l_vorher = 0;

RT_MODEL rtM_Left_;                     /* Real-time model */
RT_MODEL *const rtM_Left  = &rtM_Left_;

DW       rtDW_Left;                     /* Observable states */
ExtU     rtU_Left;                      /* External inputs */
ExtY     rtY_Left;                      /* External outputs */

uint8_t  ctrlModReq    = CTRL_MOD_REQ;  // Final control mode request 


extern volatile uint32_t ticks_32khz;
uint32_t ticks_32khz_old = 0;

int wheel_pos;
int odom_old = 0;
long wheel_speed_rpm = 0;
long wheel_speed_rpm_filtered = 0;
long wheel_angle = 0;
int _rpm_delta;

  int ul, vl, wl;
  // int ur, vr, wr;


void setError(ErrorCode code) {
  system_error = code;
  if (code) {
    gpio_bit_write(LED_X1_2_PORT, LED_X1_2_PIN, RESET);
    gpio_bit_write(LED_X1_3_PORT, LED_X1_3_PIN, SET);
    bldc_enable = false;

  } else {
    gpio_bit_write(LED_X1_3_PORT, LED_X1_3_PIN, RESET);
    gpio_bit_write(LED_X1_2_PORT, LED_X1_2_PIN, SET);
    bldc_enable = true;
  }
}

void setRemoteError(ErrorCode code) {
  remote_system_error = code;
  bldc_enable = false;
}

void setBldcEnable(FlagStatus state) {
    bldc_enable = state;
}

void setBldcPWM(int16_t power) {
	pwml = ABS(power) >= 50 ? CLAMP(power, -1000, 1000) : 0;
}

void BLDC_Init(void) {
  /* Set BLDC controller parameters */ 
  rtP_Left.b_angleMeasEna       = 0;            // Motor angle input: 0 = estimated angle, 1 = measured angle (e.g. if encoder is available)
  rtP_Left.z_selPhaCurMeasABC   = 0;            // Left motor measured current phases {Green, Blue} = {iA, iB} -> do NOT change
  rtP_Left.z_ctrlTypSel         = CTRL_TYP_SEL;
  rtP_Left.b_diagEna            = DIAG_ENA;
  rtP_Left.i_max                = (I_MOT_MAX * A2BIT_CONV) << 4;        // fixdt(1,16,4)
  rtP_Left.n_max                = N_MOT_MAX << 4;                       // fixdt(1,16,4)
  rtP_Left.b_fieldWeakEna       = FIELD_WEAK_ENA; 
  rtP_Left.id_fieldWeakMax      = (FIELD_WEAK_MAX * A2BIT_CONV) << 4;   // fixdt(1,16,4)
  rtP_Left.a_phaAdvMax          = PHASE_ADV_MAX << 4;                   // fixdt(1,16,4)
  rtP_Left.r_fieldWeakHi        = FIELD_WEAK_HI << 4;                   // fixdt(1,16,4)
  rtP_Left.r_fieldWeakLo        = FIELD_WEAK_LO << 4;                   // fixdt(1,16,4)

  /* Pack LEFT motor data into RTM */
  rtM_Left->defaultParam        = &rtP_Left;
  rtM_Left->dwork               = &rtDW_Left;
  rtM_Left->inputs              = &rtU_Left;
  rtM_Left->outputs             = &rtY_Left;

  /* Initialize BLDC controllers */
  BLDC_controller_initialize(rtM_Left);
}


/* =========================== Filtering Functions =========================== */

  /* Low pass filter fixed-point 32 bits: fixdt(1,32,16)
  * Max:  32767.99998474121
  * Min: -32768
  * Res:  1.52587890625e-05
  * 
  * Inputs:       u     = int16 or int32
  * Outputs:      y     = fixdt(1,32,16)
  * Parameters:   coef  = fixdt(0,16,16) = [0,65535U]
  * 
  * Example: 
  * If coef = 0.8 (in floating point), then coef = 0.8 * 2^16 = 52429 (in fixed-point)
  * filtLowPass16(u, 52429, &y);
  * yint = (int16_t)(y >> 16); // the integer output is the fixed-point ouput shifted by 16 bits
  */
void filtLowPass32(int32_t u, uint16_t coef, int32_t *y) {
  int64_t tmp;  
  tmp = ((int64_t)((u << 4) - (*y >> 12)) * coef) >> 4;
  tmp = CLAMP(tmp, -2147483648LL, 2147483647LL);  // Overflow protection: 2147483647LL = 2^31 - 1
  *y = (int32_t)tmp + (*y);
}


int16_t modulo(int16_t m, int16_t rest_classes){
  return (((m % rest_classes) + rest_classes) %rest_classes);
}

int16_t up_or_down(int16_t vorher, int16_t nachher){
  uint16_t up_down[6] = {0,-1,-2,0,2,1};
  //uint16_t mod_diff =  (((vorher - nachher) % 6) + 6) % 6;
  
  return up_down[modulo(vorher-nachher, 6)];
}


void buzzer_sound() {
  buzzerTimer++;
  // Create square wave for buzzer
  #ifdef BUZZER_PIN
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
    if (buzzerPrev == 0) {
      buzzerPrev = 1;
      if (++buzzerIdx > (buzzerCount + 2)) {    // pause 2 periods
        buzzerIdx = 1;
      }
    }
    if (buzzerTimer % buzzerFreq == 0 && (buzzerIdx <= buzzerCount || buzzerCount == 0)) {
      gpio_bit_write(BUZZER_PORT, BUZZER_PIN, ! gpio_output_bit_get(BUZZER_PORT, BUZZER_PIN));
    }
  } else if (buzzerPrev) {
      gpio_bit_write(BUZZER_PORT, BUZZER_PIN, RESET);
      buzzerPrev = 0;
  }
  #endif
}

void calculateRPM(int delta) {
  uint32_t timer_value = ticks_32khz;
  int32_t dt = (timer_value - ticks_32khz_old);
  if (ticks_32khz_old && dt > 32000) {
    // DEBUG_println(FST("Rotation Stopped"));
    wheel_speed_rpm = 0;
    wheel_speed_rpm_filtered = 0;
    ticks_32khz_old = 0;
    return;
  }

  _rpm_delta += delta;
  if (_rpm_delta >= 18 || _rpm_delta <= -18) { 

    if (ticks_32khz_old == 0) {
      ticks_32khz_old = timer_value;
      _rpm_delta = 0;
      //DEBUG_println(FST("Rotation Started"));
      return;
    }
  
    wheel_speed_rpm = (_rpm_delta * (60 * 32000 << 4) / dt / 360) << 4;
    if (wheel_speed_rpm_filtered == 0) { wheel_speed_rpm_filtered = wheel_speed_rpm; }
    wheel_speed_rpm_filtered = wheel_speed_rpm_filtered - (wheel_speed_rpm_filtered >> 5) + (wheel_speed_rpm >> 5);
    ticks_32khz_old = timer_value;
    _rpm_delta = 0;
  }
}

// =================================
// DMA interrupt frequency =~ 16 kHz
// =================================
//void DMA1_Channel1_IRQHandler(void) {
void CalculateBLDC(void) {

  // DMA1->IFCR = DMA_IFCR_CTCIF1;
  // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  // HAL_GPIO_TogglePin(LED_PORT, LED_PIN);


  if(offsetcount < 2000) {  // calibrate ADC offsets
    offsetcount++;
/*
    offsetrlA = (adc_buffer.rlA + offsetrlA) / 2;
    offsetrlB = (adc_buffer.rlB + offsetrlB) / 2;
    offsetrrB = (adc_buffer.rrB + offsetrrB) / 2;
    offsetrrC = (adc_buffer.rrC + offsetrrC) / 2;
    offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
    offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
*/
    offset_current_dc = (adc_buffer.current_dc + offset_current_dc) / 2;
    offset_v_batt = (adc_buffer.v_batt + offset_v_batt) / 2;
    offset_phase_a = (adc_buffer.phase_a + offset_phase_a) / 2;
    offset_phase_b = (adc_buffer.phase_b + offset_phase_b) / 2;
    return;
  }

  buzzer_sound();
  if (buzzerTimer % 1000 == 0) {  // Filter battery voltage at a slower sampling rate
    filtLowPass32(adc_buffer.v_batt, BAT_FILT_COEF, &batVoltageFixdt);
    batVoltage = (int16_t)(batVoltageFixdt >> 16);  // convert fixed-point to integer
  }

    // ####### CALC BOARD TEMPERATURE #######
    filtLowPass32(adc_buffer.mcu_temp, TEMP_FILT_COEF, &board_temp_adcFixdt);
    board_temp_adcFilt  = (int16_t)(board_temp_adcFixdt >> 16);  // convert fixed-point to integer
    board_temp_deg_c    = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) * (board_temp_adcFilt - TEMP_CAL_LOW_ADC) / (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;


  // Get motor currents
  curL_phaA = (int16_t)(offset_phase_a - adc_buffer.phase_a);
  curL_phaB = (int16_t)(offset_phase_b - adc_buffer.phase_b);
  curL_DC   = (int16_t)(offset_current_dc - adc_buffer.current_dc);
  
  // Disable PWM when current limit is reached (current chopping)
  // This is the Level 2 of current protection. The Level 1 should kick in first given by I_MOT_MAX
  if(ABS(curL_DC) > curDC_max || bldc_enable == 0) {
  	timer_automatic_output_disable(TIMER_BLDC);		
  } else {
  	timer_automatic_output_enable(TIMER_BLDC);
  }


  // Adjust pwm_margin depending on the selected Control Type
  if (rtP_Left.z_ctrlTypSel == FOC_CTRL) {
    pwm_margin = 110;
  } else {
    pwm_margin = 0;
  }

  // ############################### MOTOR CONTROL ###############################

  static boolean_T OverrunFlag = false;

  /* Check for overrun */
  if (OverrunFlag) {
    return;
  }
  OverrunFlag = true;

  /* Make sure to stop BOTH motors in case of an error */
  bldc_enableFin = bldc_enable && !rtY_Left.z_errCode;
  if (rtY_Left.z_errCode) {
    setError(EC_FOC_ERROR);
  }
 
    /* Set motor inputs here */
    rtU_Left.b_motEna     = bldc_enableFin;
    rtU_Left.z_ctrlModReq = ctrlModReq;  
    rtU_Left.r_inpTgt     = reverse_motor ? -pwml : pwml;
    rtU_Left.b_hallA      = !gpio_input_bit_get(HALL_A_PORT, HALL_A_PIN);
    rtU_Left.b_hallB      = !gpio_input_bit_get(HALL_B_PORT, HALL_B_PIN);
    rtU_Left.b_hallC      = !gpio_input_bit_get(HALL_C_PORT, HALL_C_PIN);
    rtU_Left.i_phaAB      = curL_phaA;
    rtU_Left.i_phaBC      = curL_phaB;
    rtU_Left.i_DCLink     = curL_DC;
    // rtU_Left.a_mechAngle   = ...; // Angle input in DEGREES [0,360] in fixdt(1,16,4) data type. If `angle` is float use `= (int16_t)floor(angle * 16.0F)` If `angle` is integer use `= (int16_t)(angle << 4)`
    
    /* Step the controller */
    BLDC_controller_step(rtM_Left);

    /* Get motor outputs here */
    ul            = rtY_Left.DC_phaA;
    vl            = rtY_Left.DC_phaB;
    wl            = rtY_Left.DC_phaC;
  // errCodeLeft  = rtY_Left.z_errCode;
  // motSpeedLeft = rtY_Left.n_mot;
  // motAngleLeft = rtY_Left.a_elecAngle;
    uint8_t encoding = 0;
    if (reverse_motor) {
      encoding = (uint8_t)((rtU_Left.b_hallC<<2) + (rtU_Left.b_hallB<<1) + rtU_Left.b_hallA );
    } else {
      encoding = (uint8_t)((rtU_Left.b_hallA<<2) + (rtU_Left.b_hallB<<1) + rtU_Left.b_hallC );
    }
    wheel_pos = rtConstP.vec_hallToPos_Value[encoding];

    int delta = up_or_down(wp_l_vorher, wheel_pos);
    wp_l_vorher = wheel_pos;
    odom_l = modulo(odom_l + delta, 9000);
    wheel_angle += delta * 6; // 60 steps per rotation => 6 degree per step
    calculateRPM(delta * 6);
  
    /* Apply commands */
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_Y, CLAMP(ul + pwm_res / 2, pwm_margin, pwm_res-pwm_margin));
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_B, CLAMP(vl + pwm_res / 2, pwm_margin, pwm_res-pwm_margin));
  timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_G, CLAMP(wl + pwm_res / 2, pwm_margin, pwm_res-pwm_margin));
	
  // =================================================================
  

  /* Indicate task complete */
  OverrunFlag = false;
 
 // ###############################################################################

}
