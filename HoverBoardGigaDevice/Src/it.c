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
#include "it.h"
#include "defines.h"
#include "config.h"
#include "bldc.h"
#include "led.h"
#include "cli.h"

uint32_t ticksMs;
uint32_t ticksMs10;
uint32_t timeoutCounter_ms = 0;
FlagStatus timedOut = SET;
volatile uint32_t ticks_32khz = 0;

//----------------------------------------------------------------------------
// SysTick_Handler 100Hz
//----------------------------------------------------------------------------
void SysTick_Handler(void) {
  	ticksMs10++;
}

//----------------------------------------------------------------------------
// Resets the timeout to zero
//----------------------------------------------------------------------------
void ResetTimeout(void)
{
  	timeoutCounter_ms = 0;
}

//----------------------------------------------------------------------------
// Timer13_Update_Handler
// Is called when upcouting of timer13 is finished and the UPDATE-flag is set
// -> period of timer13 running with 1kHz -> interrupt every 1ms
//----------------------------------------------------------------------------
void TIMER13_IRQHandler(void)
{	
	//TIMER13_int_cnt++;  // Debug counter
	ticksMs++;
	if (timeoutCounter_ms > TIMEOUT_MS)
	{
		// First timeout reset all process values
		if (timedOut == RESET)
		{
			setBldcPWM(0);
		}
		timedOut = SET;
	}
	else
	{
		timedOut = RESET;
		timeoutCounter_ms++;
	}
	
	// Update LED program
	//CalculateLEDProgram();
	
	// Clear timer update interrupt flag
	timer_interrupt_flag_clear(TIMER13, TIMER_INT_UP);
}

//----------------------------------------------------------------------------
// Timer0_Update_Handler
// Is called when upcouting of timer0 is finished and the UPDATE-flag is set
// AND when downcouting of timer0 is finished and the UPDATE-flag is set
// -> pwm of timer0 running with 16kHz -> interrupt every 31,25us
//----------------------------------------------------------------------------
void TIMER0_BRK_UP_TRG_COM_IRQHandler(void)
{
	//TIMER0_int_cnt++;  // Debug counter
	// Start ADC conversion
	adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
	
	// Clear timer update interrupt flag
	timer_interrupt_flag_clear(TIMER_BLDC, TIMER_INT_UP);

	ticks_32khz++;
}

//----------------------------------------------------------------------------
// This function handles DMA_Channel0_IRQHandler interrupt
// Is called, when the ADC scan sequence is finished
// -> ADC is triggered from timer0-update-interrupt -> every 31,25us
//----------------------------------------------------------------------------
void DMA_Channel0_IRQHandler(void)
{
    // gpio_bit_write(LED_X1_1_PORT, LED_X1_1_PIN, SET);

	//DMA0_int_cnt++;  // Debug counter
	// Calculate motor PWMs
	CalculateBLDC();
	
	#ifdef SLAVE
	// Calculates RGB LED
	CalculateLEDPWM();
	#endif
	
	if (dma_interrupt_flag_get(DMA_CH0, DMA_INT_FLAG_FTF))
	{
		dma_interrupt_flag_clear(DMA_CH0, DMA_INT_FLAG_FTF);        
	}
    // gpio_bit_write(LED_X1_1_PORT, LED_X1_1_PIN, RESET);
}


//----------------------------------------------------------------------------
// This function handles DMA_Channel1_2_IRQHandler interrupt
// Is asynchronously called when USART0 RX finished
//----------------------------------------------------------------------------
void DMA_Channel1_2_IRQHandler(void)
{
	// USART steer/bluetooth RX
	if (dma_interrupt_flag_get(DMA_CH2, DMA_INT_FLAG_FTF))
	{
		dma_interrupt_flag_clear(DMA_CH2, DMA_INT_FLAG_FTF);        
	}
}


//----------------------------------------------------------------------------
// This function handles DMA_Channel3_4_IRQHandler interrupt
// Is asynchronously called when USART_SLAVE RX finished
//----------------------------------------------------------------------------
void DMA_Channel3_4_IRQHandler(void)
{
	// USART master slave RX
	if (dma_interrupt_flag_get(DMA_CH4, DMA_INT_FLAG_FTF))
	{
		dma_interrupt_flag_clear(DMA_CH4, DMA_INT_FLAG_FTF);        
	}
}

volatile char foo =0;

void USART0_IRQHandler(void)
{
#if USART_RX_DMA
	if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_IDLE))
	{
		/* clear USART_INT_FLAG_IDLE */
		usart_interrupt_flag_clear(USART0, USART_INT_FLAG_IDLE);
		uint16_t bytes = usart_data_receive(USART0);
		/* disable USART0_RX DMA_Channel */
		dma_channel_disable(DMA_CH2);    
		/* reset DMA_Channel CNT */
		dma_transfer_number_config(DMA_CH2, USART0_RX_SIZE);
		/* enable USART0_RX DMA_Channel */
		dma_channel_enable(DMA_CH2);
	}
#else	
	/*!< read data buffer not empty interrupt and flag */
	if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE))
	{
		usart_interrupt_flag_clear(USART0, USART_INT_FLAG_RBNE);
		// char c = usart_data_receive(USART0);
 		cliReceive(&control_cli, usart_data_receive(USART0));
	}
#endif

   	/*!< read data buffer not empty interrupt and overrun error flag */
   	if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE_ORERR)) { 
	   usart_interrupt_flag_clear(USART0, USART_INT_FLAG_RBNE_ORERR);
	} 
 
    #if 0
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_EB)) { usart_interrupt_flag_clear(USART0, USART_INT_FLAG_EB); cliReceive(&control_cli, '0');} /*!< end of block interrupt and flag */
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RT)) { usart_interrupt_flag_clear(USART0, USART_INT_FLAG_RT); cliReceive(&control_cli, '1');}       /*!< receiver timeout interrupt and flag */
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_AM)) { usart_interrupt_flag_clear(USART0, USART_INT_FLAG_AM); cliReceive(&control_cli, '2');}       /*!< address match interrupt and flag */
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_PERR)) { usart_interrupt_flag_clear(USART0, USART_INT_FLAG_PERR); cliReceive(&control_cli, '3');}       /*!< parity error interrupt and flag */
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_TBE)) { usart_interrupt_flag_clear(USART0, USART_INT_FLAG_TBE); cliReceive(&control_cli, '4');}        /*!< transmitter buffer empty interrupt and flag */
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_TC)) { usart_interrupt_flag_clear(USART0, USART_INT_FLAG_TC); cliReceive(&control_cli, '5');}         /*!< transmission complete interrupt and flag */
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_IDLE)) { usart_interrupt_flag_clear(USART0, USART_INT_FLAG_IDLE); cliReceive(&control_cli, '7');}       /*!< IDLE line detected interrupt and flag */
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_LBD)) { usart_interrupt_flag_clear(USART0, USART_INT_FLAG_LBD); cliReceive(&control_cli, '8');}        /*!< LIN break detected interrupt and flag */
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_WU)) { usart_interrupt_flag_clear(USART0, USART_INT_FLAG_WU); cliReceive(&control_cli, '9');}       /*!< wakeup from deep-sleep mode interrupt and flag */
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_CTS)) { usart_interrupt_flag_clear(USART0, USART_INT_FLAG_CTS); cliReceive(&control_cli, 'A');}      /*!< CTS interrupt and flag */
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_ERR_NERR)) { usart_interrupt_flag_clear(USART0, USART_INT_FLAG_ERR_NERR); cliReceive(&control_cli, 'B');}   /*!< error interrupt and noise error flag */
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_ERR_ORERR)) { usart_interrupt_flag_clear(USART0, USART_INT_FLAG_ERR_ORERR); cliReceive(&control_cli, 'C');}  /*!< error interrupt and overrun error */
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_ERR_FERR )) { usart_interrupt_flag_clear(USART0, USART_INT_FLAG_ERR_FERR); cliReceive(&control_cli, 'D');} /*!< error interrupt and frame error flag */
	#endif

}


void USART1_IRQHandler(void)
{
#if USART_RX_DMA	
	if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_IDLE))
	{
		/* clear USART_INT_FLAG_IDLE */
		usart_interrupt_flag_clear(USART1, USART_INT_FLAG_IDLE);
		uint16_t bytes = 1;// usart_data_receive(USART1);
		uint16_t n = 0;
		while(n < bytes) {
			cliReceive(&master_slave_cli, USART1_RX_Buffer[n++]);
		}
		/* disable USART1_RX DMA_Channel */
		dma_channel_disable(DMA_CH4);    
		/* reset DMA_Channel CNT */
		dma_transfer_number_config(DMA_CH4, USART1_RX_SIZE);
		/* enable USART1_RX DMA_Channel */
		dma_channel_enable(DMA_CH4);
	}
#else	
	/*!< read data buffer not empty interrupt and flag */
	if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE))
	{
		usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE);
		// USART1_RX_Buffer[0] = usart_data_receive(USART1);
		cliReceive(&proxy_cli, usart_data_receive(USART1));
	}
#endif	
   	/*!< read data buffer not empty interrupt and overrun error flag */
   	if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE_ORERR)) { 
	   usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE_ORERR);
	} 

}



//----------------------------------------------------------------------------
// Returns number of milliseconds since system start
//----------------------------------------------------------------------------
uint32_t millis()
{
	return ticksMs;
}


//----------------------------------------------------------------------------
// Returns number of microseconds since system start (32kHz actual resolution)
//----------------------------------------------------------------------------
uint32_t micros() {
  	return (ticks_32khz * 125LL) >> 4;
}


//----------------------------------------------------------------------------
// Delays number of milliseconds
//----------------------------------------------------------------------------
void delay(uint32_t delayMs) {
  	uint32_t curTicks= ticksMs;
  	while ((ticksMs - curTicks) < delayMs) {
		__NOP();
	}
}

//----------------------------------------------------------------------------
// This function handles Non maskable interrupt.
//----------------------------------------------------------------------------
void NMI_Handler(void)
{
}

//----------------------------------------------------------------------------
// This function handles Hard fault interrupt.
//----------------------------------------------------------------------------
void HardFault_Handler(void)
{
  while(1) {}
}

//----------------------------------------------------------------------------
// This function handles Memory management fault.
//----------------------------------------------------------------------------
void MemManage_Handler(void)
{
  while(1) {}
}

//----------------------------------------------------------------------------
// This function handles Prefetch fault, memory access fault.
//----------------------------------------------------------------------------
void BusFault_Handler(void)
{
  while(1) {}
}

//----------------------------------------------------------------------------
// This function handles Undefined instruction or illegal state.
//----------------------------------------------------------------------------
void UsageFault_Handler(void)
{
  while(1) {}
}

//----------------------------------------------------------------------------
// This function handles System service call via SWI instruction.
//----------------------------------------------------------------------------
void SVC_Handler(void)
{
}

//----------------------------------------------------------------------------
// This function handles Debug monitor.
//----------------------------------------------------------------------------
void DebugMon_Handler(void)
{
}

//----------------------------------------------------------------------------
// This function handles Pendable request for system service.
//----------------------------------------------------------------------------
void PendSV_Handler(void)
{
}
