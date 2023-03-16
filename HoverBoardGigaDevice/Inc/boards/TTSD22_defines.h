/* ============================================== *\
 * Hardware
\* ============================================== */

#define MCU_TYPE "GD32F130C8T6"
#define BOARD_TYPE "TTSD2.2"

// LED X1 defines comments for 6-LED PCB, 4-LED PCB lacks white wire - ORANGE LED
#define LED_X1_1_PIN GPIO_PIN_12 //white wire ORANGE LED
#define LED_X1_1_PORT GPIOA
#define LED_X1_2_PIN GPIO_PIN_15 //blue wire GREEN LED
#define LED_X1_2_PORT GPIOA
#define LED_X1_3_PIN GPIO_PIN_3 //black wire RED LED
#define LED_X1_3_PORT GPIOB

// LED X2 defines
#define LED_X2_1_PIN GPIO_PIN_1
#define LED_X2_1_PORT GPIOA
#define LED_X2_2_PIN GPIO_PIN_0
#define LED_X2_2_PORT GPIOA

// Mosfet output
#define MOSFET_OUT_PIN GPIO_PIN_13
#define MOSFET_OUT_PORT GPIOC

// Brushless Control DC (BLDC) defines
// Channel G
#define RCU_TIMER_BLDC RCU_TIMER0
#define TIMER_BLDC TIMER0
#define TIMER_BLDC_CHANNEL_G TIMER_CH_2
#define TIMER_BLDC_GH_PIN GPIO_PIN_10
#define TIMER_BLDC_GH_PORT GPIOA
#define TIMER_BLDC_GL_PIN GPIO_PIN_15
#define TIMER_BLDC_GL_PORT GPIOB
// Channel B
#define TIMER_BLDC_CHANNEL_B TIMER_CH_1
#define TIMER_BLDC_BH_PIN GPIO_PIN_9
#define TIMER_BLDC_BH_PORT GPIOA
#define TIMER_BLDC_BL_PIN GPIO_PIN_14
#define TIMER_BLDC_BL_PORT GPIOB
// Channel Y
#define TIMER_BLDC_CHANNEL_Y TIMER_CH_0
#define TIMER_BLDC_YH_PIN GPIO_PIN_8
#define TIMER_BLDC_YH_PORT GPIOA
#define TIMER_BLDC_YL_PIN GPIO_PIN_13
#define TIMER_BLDC_YL_PORT GPIOB

// Timer BLDC short circuit emergency shutoff define
#define TIMER_BLDC_EMERGENCY_SHUTDOWN_PIN GPIO_PIN_12
#define TIMER_BLDC_EMERGENCY_SHUTDOWN_PORT GPIOB
#define TIMER_BLDC_EMERGENCY_SHUTDOWN_POLARITY TIMER_BREAK_POLARITY_LOW


// Hall sensor defines
#define HALL_A_PIN GPIO_PIN_11
#define HALL_A_PORT GPIOB
#define HALL_B_PIN GPIO_PIN_1
#define HALL_B_PORT GPIOF
#define HALL_C_PIN GPIO_PIN_14
#define HALL_C_PORT GPIOC

//#define MOTOR_TEMP_PIN	GPIO_PIN_0
//#define MOTOR_TEMP_PORT GPIOA

// Usart master slave defines
#define USART_MASTERSLAVE USART1
#define USART_MASTERSLAVE_TX_PIN GPIO_PIN_2
#define USART_MASTERSLAVE_TX_PORT GPIOA
#define USART_MASTERSLAVE_RX_PIN GPIO_PIN_3
#define USART_MASTERSLAVE_RX_PORT GPIOA

// ADC defines
#define VBATT_PIN	GPIO_PIN_4
#define VBATT_PORT GPIOA
#define VBATT_CHANNEL ADC_CHANNEL_4
#define CURRENT_DC_PIN	GPIO_PIN_6
#define CURRENT_DC_PORT GPIOA
#define CURRENT_DC_CHANNEL ADC_CHANNEL_6

#define PHASE_A_PIN	GPIO_PIN_1
#define PHASE_A_PORT GPIOB
#define PHASE_A_CHANNEL ADC_CHANNEL_9
#define PHASE_B_PIN	GPIO_PIN_0
#define PHASE_B_PORT GPIOB
#define PHASE_B_CHANNEL ADC_CHANNEL_8

// Usart steer defines
#define USART_STEER_COM USART0
#define USART_STEER_COM_TX_PIN GPIO_PIN_6
#define USART_STEER_COM_TX_PORT GPIOB
#define USART_STEER_COM_RX_PIN GPIO_PIN_7
#define USART_STEER_COM_RX_PORT GPIOB

#ifdef MASTER
// Self hold defines
#define SELF_HOLD_PIN GPIO_PIN_2
#define SELF_HOLD_PORT GPIOB

// Button defines
#define BUTTON_PIN GPIO_PIN_15
#define BUTTON_PORT GPIOC

// Buzzer defines
#define BUZZER_PIN GPIO_PIN_10
#define BUZZER_PORT GPIOB

// Charge state defines
#define CHARGE_STATE_PIN GPIO_PIN_0
#define CHARGE_STATE_PORT GPIOF
#endif

// Debug pin defines
#define DEBUG_PIN GPIO_PIN_4
#define DEBUG_PORT GPIOB

// ADC value conversion defines
#define MOTOR_AMP_CONV_DC_AMP 0.201465201465  // 3,3V * 1/3 - 0,004Ohm * IL(ampere) = (ADC-Data/4095) *3,3V
#define ADC_BATTERY_VOLT      0.024169921875 	// V_Batt to V_BattMeasure = factor 30: ( (ADC-Data/4095) *3,3V *30 )

#define BAT_CALIB_REAL_VOLTAGE  2400      // input voltage measured by multimeter (multiplied by 100). In this case 43.00 V * 100 = 4300
#define BAT_CALIB_ADC           954      // adc-value measured by mainboard (value nr 5 on UART debug output)
#define BAT_CELLS               10        // battery number of cells. Normal Hoverboard battery: 10s