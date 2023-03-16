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


#include "stdio.h"
#include "stdlib.h"
//#include "string.h"
//#include <math.h>     
//#define ARM_MATH_CM3
//#include "arm_math.h" 

#include "gd32f1x0.h"


#include "defines.h"
#include "config.h"
#include "setup.h"
#include "debug.h"
#include "it.h"
#include "bldc.h"
#include "pid.h"
#include "cli.h"
#include "utils.h"
#include "comms.h"
#include "buzzer.h"



#ifdef MASTER
int32_t steer = 0; 												// global variable for steering. -1000 to 1000
int32_t speed = 0; 												// global variable for speed.    -1000 to 1000
FlagStatus activateWeakening = RESET;			// global variable for weakening
FlagStatus beepsBackwards = RESET;  			// global variable for beeps backwards
			
extern uint8_t buzzerFreq;    						// global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; 						// global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...
			
extern float batteryVoltage; 							// global variable for battery voltage
extern float currentDC; 									// global variable for current dc
extern float realSpeed; 									// global variable for real Speed
uint8_t slaveError = 0;										// global variable for slave error
	
extern FlagStatus timedOut;								// Timeoutvariable set by timeout timer

uint32_t inactivity_timeout_counter = 0;	// Inactivity counter
uint32_t steerCounter = 0;								// Steer counter for setting update rate

typedef enum {GREEN, ORANGE, RED, NONE} battery_state_t;
void ShowBatteryState(battery_state_t state);
void BeepsBackwards(FlagStatus beepsBackwards);
void ShutOff(void);
#endif

uint32_t last_millis = 0;

float batt_u_calibrated;
float batt_percent;

void watchdogReset() {
    fwdgt_counter_reload();
}

void poweroff(void) {
  bldc_enable = 0;
  DEBUG_println(FST("-- Motors disabled --\n"));
  buzzerCount = 0;  // prevent interraction with beep counter
  buzzerPattern = 0;
  for (int i = 0; i < 8; i++) {
    buzzerFreq = (uint8_t)i;
    delay(100);
  }
  buzzerFreq=0;
  // saveConfig();
  ShowBatteryState(NONE);
  gpio_bit_write(LED_X2_1_PORT, LED_X2_1_PIN, RESET);
  gpio_bit_write(LED_X2_2_PORT, LED_X2_2_PIN, RESET);
  #ifdef SELF_HOLD_PIN
    gpio_bit_write(SELF_HOLD_PORT, SELF_HOLD_PIN, RESET);
  #endif
  while(1) {
	watchdogReset();
	delay(100);
  }
}

//----------------------------------------------------------------------------
// MAIN function
//----------------------------------------------------------------------------
int main (void)
{
	int16_t pwmMaster = 0;
#ifdef MASTER
	FlagStatus enable = RESET;
	FlagStatus enableSlave = RESET;
	FlagStatus chargeStateLowActive = SET;
	int16_t sendSlaveValue = 0;
	uint8_t sendSlaveIdentifier = 0;
	//int8_t index = 8;
  int16_t pwmSlave = 0;
	int16_t scaledSpeed = 0;
	int16_t scaledSteer  = 0;
	float expo = 0;
	float steerAngle = 0;
	float xScale = 0;
#endif
	
	//SystemClock_Config();
  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock / 100);

  // Init watchdog
	if (Watchdog_init() == ERROR)
	{
		// If an error accours with watchdog initialization do not start device
		while(1);
	}
	
    BLDC_Init();

	// Init Interrupts
	Interrupt_init();
	
	// Init timeout timer
	TimeoutTimer_init();

	// Init GPIOs
	GPIO_init();

	// Activate self hold direct after GPIO-init
	#ifdef SELF_HOLD_PIN
	gpio_bit_write(SELF_HOLD_PORT, SELF_HOLD_PIN, SET);
	#endif
	ShowBatteryState(NONE);
	// Init usart master slave
    usart_init(USART0, USART0_BAUD); // Control
    usart_init(REMOTE_UART, USART1_BAUD); // Proxy
	#ifdef MASTER
    DEBUG_println(FST("\nSTART"));
	#endif
	#ifdef SLAVE
	uint8_t rb[2];
	rb[0] = 0xDE;
	rb[1] = 0xDE;
	SendBuffer(REMOTE_UART, &rb, 2);
	#endif

	// Init ADC
	ADC_init();
	
	// Init PWM
	PWM_init();

	// Device has 1,6 seconds to do all the initialization
	// afterwards watchdog will be fired
	fwdgt_counter_reload();
#ifdef MASTER
	//Startup-Sound
	beepShortMany(10, 1); 
	// Wait until button is pressed
	while (gpio_input_bit_get(BUTTON_PORT, BUTTON_PIN))
	{
		// Reload watchdog while button is pressed
		fwdgt_counter_reload();
	}
#endif
		// Enable channel output
		setBldcEnable(SET);
    pwmMaster = CLAMP(100, -1000, 1000);
		setBldcPWM(pwmMaster); //sets motor power before pid


    #if 0
    anglePid.set_point = 360*1;
    // speedPid.set_point = 150.0;
    target_pwm = 500;
    control_type = CT_ANGLE;
    #endif
bool state=0;
while(1) 
{
    uint32_t now = millis(); 
    if (now - last_millis > 1000) { //debug
        last_millis = now;
		watchdogReset();
         gpio_bit_write(LED_X2_2_PORT, LED_X2_2_PIN, state); //big blue 1
		state=!state;
		gpio_bit_write(LED_X2_1_PORT, LED_X2_1_PIN, state); //big blue 2
        /*debug_printf(" | I:%d (%d) [%d %d] U:%d (%d) | Pos: %d %d Ang:%d S:%d (%d)| PWM: %d  (%d %d %d) | PID: %f %f\n\r", 
        bldc_enableFin, curL_DC, offset_current_dc, curL_phaA, curL_phaB, ((int32_t)batVoltage * BAT_CALIB_REAL_VOLTAGE) / BAT_CALIB_ADC, batVoltage, wheel_pos, odom_l, wheel_angle, wheel_speed_rpm_filtered>>8, wheel_speed_rpm>>8, pwml, ul, vl, wl,
        angle_PID_error_old, angle_set_point_old);
        */
        //current_sp, shaft_velocity_sp, shaft_angle_sp, shaft_angle);
        //voltage.d, voltage.q, current.d, current.q);


        /*debug_printf("EN:%d | PWM:%d | Pos: %d %d Ang:%d S:%d (%d) | PID: set: ",
        bldc_enableFin, pwml, wheel_pos, odom_l, wheel_angle, wheel_speed_rpm_filtered>>8, wheel_speed_rpm>>8);
        char buffer[64];
        f2s(buffer, speedPid.set_point, 1); debug_print(buffer); debug_print(" int:");
        f2s(buffer, speedPid.integral, 1); debug_print(buffer); debug_print(" err:");
        f2s(buffer, speedPid.last_error, 3); debug_print(buffer); debug_print("\n"); */
       // debug_printf("Cur:%d  Bat:%d (%d)\n", curL_DC, ((int32_t)batVoltage * BAT_CALIB_REAL_VOLTAGE) / BAT_CALIB_ADC, batVoltage);
		
		//xtern volatile adc_buf_t adc_buffer;
	    //debug_printf("Cur:%d  Bat:%d (%d) %d - %d\n", curL_DC, ((int32_t)batVoltage * BAT_CALIB_REAL_VOLTAGE) / BAT_CALIB_ADC, batVoltage, adc_buffer.v_batt, adc_buffer.mcu_temp);
    }
    pidControllerRun();
    #ifdef CLI
	cliRun();
	
	#endif
	remoteRun();

	batt_u_calibrated = ((float)batVoltage * BAT_CALIB_REAL_VOLTAGE) / BAT_CALIB_ADC / 100.0;
	batt_percent = (batt_u_calibrated - (BAT_CELLS * 3.4)) * 100.0 / (BAT_CELLS * (4.2 - 3.4));
	board_temp_c = board_temp_deg_c * 0.1;

	#ifdef MOTOR_TEMP_PORT
	  if (!gpio_input_bit_get(MOTOR_TEMP_PORT, MOTOR_TEMP_PIN)) { setError(EC_MOTOR_OVER_TEMP); }
    #endif

    if ((TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF)) {  // poweroff before mainboard burns OR low bat 3
		  setError(EC_BOARD_OVER_TEMP);
	      poweroff();
		} else if (batVoltage < BAT_DEAD) {
		  setError(EC_LOW_BATTERY);
		  ShowBatteryState(RED);
	      poweroff();
		  
	    } else if (system_error || remote_system_error) {                                           // 1 beep (low pitch): Motor error, disable motors
	      bldc_enable = false;
	     // beepCount(1, 24, 1);
		 ShowBatteryState(RED);
	    } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) {                             // 5 beeps (low pitch): Mainboard temperature warning
	      beepCount(5, 24, 1);
		  ShowBatteryState(RED);
	    } else if (BAT_LVL1_ENABLE && batVoltage < BAT_LVL1) {                                            // 1 beep fast (medium pitch): Low bat 1
	      beepCount(0, 10, 6);
		  ShowBatteryState(RED);
	    } else if (BAT_LVL2_ENABLE && batVoltage < BAT_LVL2) {                                            // 1 beep slow (medium pitch): Low bat 2
	      beepCount(0, 10, 30);
		  ShowBatteryState(ORANGE);
	    } else {  // do not beep
	      beepCount(0, 0, 0);
		  ShowBatteryState(GREEN);
 	   }
	
		// Shut device off when button is pressed
		if (gpio_input_bit_get(BUTTON_PORT, BUTTON_PIN)) {
      		while (gpio_input_bit_get(BUTTON_PORT, BUTTON_PIN)) {poweroff(); }
    	}
	}
 /* char tbuffer[256];
  uint8_t led_state = 0;

	while(1) {
    DEBUG_println(FST("BLDC start!\n"));
    //gpio_bit_write(LED_X1_2_PORT, LED_X1_2_PIN, SET);
	  // SendBuffer(USART_STEER_COM, "Hello 0123456789!\n\r", 18);
    //SendBuffer(USART_MASTERSLAVE, "HELLO 0123456789!\n\r", 18);

    n = sprintf(tbuffer, "EN:%d | Hall: %d / %d | BLDC: %d (%d %d %d) | Int: %d | U=%d I=%d Speed=%d\n\r", 
    bldc_enable, hall, pos, bldc_outputFilterPwm, hb_y, hb_b, hb_g, bldc_int_cnt, (int)(batteryVoltage*1000.0), (int)(currentDC*1000.0), (int)(realSpeed*1000.0));
    SendBuffer(USART_MASTERSLAVE, tbuffer, n);

    // TIMER_CCHP(TIMER_BLDC) |= 0x8000;
    
    n = sprintf(tbuffer, "EN:%d | I:%d (%d) [%d %d] U:%d (%d) | Pos: %d %d Ang:%d S:%d (%d)| PWM: %d  (%d %d %d) | PWM: %X\n\r", 
      bldc_enableFin, curL_DC, offset_current_dc, curL_phaA, curL_phaB, ((int32_t)batVoltage * BAT_CALIB_REAL_VOLTAGE) / BAT_CALIB_ADC, batVoltage, wheel_pos, odom_l, wheel_angle, wheel_speed_rpm_filtered>>8, wheel_speed_rpm>>8, pwml, ul, vl, wl,
      TIMER_CCHP(TIMER_BLDC)); // Bad: 583C  Good: D83C (POEN bit 15 should be high)
    SendBuffer(USART_MASTERSLAVE, tbuffer, n);
    
    if (wheel_angle > 360*10) {
     		setBldcPWM(0); 
    }
    fwdgt_counter_reload();
    // gpio_bit_write(LED_X2_GREEN_PORT, LED_X2_GREEN_PIN, led_state & 1);
    led_state++;
		//Delay(DELAY_IN_MAIN_LOOP);
    delay(100);*/
	watchdogReset();
}

//----------------------------------------------------------------------------
// Shows the battery state on the LEDs
//----------------------------------------------------------------------------
void ShowBatteryState(battery_state_t state)
{
  switch (state) {
    case GREEN:
	    gpio_bit_write(LED_X1_1_PORT, LED_X1_1_PIN, RESET);
	    gpio_bit_write(LED_X1_2_PORT, LED_X1_2_PIN, SET);
		gpio_bit_write(LED_X1_3_PORT, LED_X1_3_PIN, RESET);
      break; 

    case ORANGE:
	    gpio_bit_write(LED_X1_1_PORT, LED_X1_1_PIN, SET);
	    gpio_bit_write(LED_X1_2_PORT, LED_X1_2_PIN, SET);
		gpio_bit_write(LED_X1_3_PORT, LED_X1_3_PIN, SET);
	  
      break; 

    case RED:
	    gpio_bit_write(LED_X1_1_PORT, LED_X1_1_PIN, RESET);
	    gpio_bit_write(LED_X1_2_PORT, LED_X1_2_PIN, RESET);
		gpio_bit_write(LED_X1_3_PORT, LED_X1_3_PIN, SET);
      break; 

	case NONE:
	    gpio_bit_write(LED_X1_1_PORT, LED_X1_1_PIN, RESET);
	    gpio_bit_write(LED_X1_2_PORT, LED_X1_2_PIN, RESET);
		gpio_bit_write(LED_X1_3_PORT, LED_X1_3_PIN, RESET);
	  break;
  }
}
/*
#ifdef MASTER
		steerCounter++;	
		if ((steerCounter % 2) == 0)
		{	
			// Request steering data
			SendSteerDevice();
		}
		
		// Calculate expo rate for less steering with higher speeds
		expo = MAP((float)ABS(speed), 0, 1000, 1, 0.5);
		
	  // Each speedvalue or steervalue between 50 and -50 means absolutely no pwm
		// -> to get the device calm 'around zero speed'
		scaledSpeed = speed < 50 && speed > -50 ? 0 : CLAMP(speed, -1000, 1000) * SPEED_COEFFICIENT;
		scaledSteer = steer < 50 && steer > -50 ? 0 : CLAMP(steer, -1000, 1000) * STEER_COEFFICIENT * expo;
		
		// Map to an angle of 180 degress to 0 degrees for array access (means angle -90 to 90 degrees)
		steerAngle = MAP((float)scaledSteer, -1000, 1000, 180, 0);
		// xScale = lookUpTableAngle[(uint16_t)steerAngle];

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
		
		// Read charge state
		// chargeStateLowActive = gpio_input_bit_get(CHARGE_STATE_PORT, CHARGE_STATE_PIN);
		
		// Enable is depending on charger is connected or not
		enable = chargeStateLowActive;
		
		// Enable channel output
		SetEnable(enable);

		// Decide if slave will be enabled
		enableSlave = (enable == SET && timedOut == RESET) ? SET : RESET;
		
		// Decide which process value has to be sent
		switch(sendSlaveIdentifier)
		{
			case 0:
				sendSlaveValue = currentDC * 100;
				break;
			case 1:
				sendSlaveValue = batteryVoltage * 100;
				break;
			case 2:
				sendSlaveValue = realSpeed * 100;
				break;
				default:
					break;
		}
		
    // Set output
		setBldcPWM(pwmMaster);
		SendSlave(-pwmSlave, enableSlave, RESET, chargeStateLowActive, sendSlaveIdentifier, sendSlaveValue);
		
		// Increment identifier
		sendSlaveIdentifier++;
		if (sendSlaveIdentifier > 2)
		{
			sendSlaveIdentifier = 0;
		}
		
		// Show green battery symbol when battery level BAT_LOW_LVL1 is reached
    if (batteryVoltage > BAT_LOW_LVL1)
		{
			// Show green battery light
			ShowBatteryState(GREEN);
			
			// Beeps backwards
			BeepsBackwards(beepsBackwards);
		}
		// Make silent sound and show orange battery symbol when battery level BAT_LOW_LVL2 is reached
    else if (batteryVoltage > BAT_LOW_LVL2 && batteryVoltage < BAT_LOW_LVL1)
		{
			// Show orange battery light
			ShowBatteryState(ORANGE);
			
      buzzerFreq = 5;
      buzzerPattern = 8;
    }
		// Make even more sound and show red battery symbol when battery level BAT_LOW_DEAD is reached
		else if  (batteryVoltage > BAT_LOW_DEAD && batteryVoltage < BAT_LOW_LVL2)
		{
			// Show red battery light
			ShowBatteryState(RED);
			
      buzzerFreq = 5;
      buzzerPattern = 1;
    }
		// Shut device off, when battery is dead
		else if (batteryVoltage < BAT_LOW_DEAD)
		{
      ShutOff();
    }
		else
		{
			ShutOff();
    }

		// Shut device off when button is pressed
		if (gpio_input_bit_get(BUTTON_PORT, BUTTON_PIN))
		{
      while (gpio_input_bit_get(BUTTON_PORT, BUTTON_PIN)) {}
			ShutOff();
    }
		
		// Calculate inactivity timeout (Except, when charger is active -> keep device running)
    if (ABS(pwmMaster) > 50 || ABS(pwmSlave) > 50 || !chargeStateLowActive)
		{
      inactivity_timeout_counter = 0;
    }
		else
		{
      inactivity_timeout_counter++;
    }
		
		// Shut off device after INACTIVITY_TIMEOUT in minutes
    if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1))
		{ 
      ShutOff();
    }
#endif	


#ifdef MASTER
//----------------------------------------------------------------------------
// Turns the device off
//----------------------------------------------------------------------------
void ShutOff(void)
{
	int index = 0;

	buzzerPattern = 0;
	for (; index < 8; index++)
	{
		buzzerFreq = index;
		delay(100);
	}
	buzzerFreq = 0;
	
	// Send shut off command to slave
	SendSlave(0, RESET, SET, RESET, RESET, RESET);
	
	// Disable usart
	usart_deinit(USART_MASTERSLAVE);
	
	// Set pwm and enable to off
	SetEnable(RESET);
	setBldcPWM(0);
	
	#ifdef SELF_HOLD_PIN
	gpio_bit_write(SELF_HOLD_PORT, SELF_HOLD_PIN, RESET);
	#endif
	while(1)
	{
		// Reload watchdog until device is off
		fwdgt_counter_reload();
	}
}



//----------------------------------------------------------------------------
// Beeps while driving backwards
//----------------------------------------------------------------------------
void BeepsBackwards(FlagStatus beepsBackwards)
{
	// If the speed is less than -50, beep while driving backwards
	if (beepsBackwards == SET && speed < -50)
	{
		buzzerFreq = 5;
    buzzerPattern = 4;
	}
	else
	{
		buzzerFreq = 0;
		buzzerPattern = 0;
	}
}
#endif
*/