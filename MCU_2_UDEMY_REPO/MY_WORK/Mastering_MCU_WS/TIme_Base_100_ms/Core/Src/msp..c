/*
 * msp..c
 *
 *  Created on: Dec 26, 2023
 *      Author: wardawg
 */
#include <main.h>
void HAL_MspInit(void) {
	//Here,we do the Low Level Processor Initializations.

	//1. Set up the priority grouping of Processor
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	//2. Enable the required exceptions (system) of the Arm Cortex Mx Processor
	SCB->SHCSR |= 0x7 << 16; //For setting the usage fault, bus fault and memory fault system exceptions

	//3. Configure the priority for the system initializations.
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
}


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htimer6){

	//1. Enable teh Clock for ttimer 6 peripheral
	__HAL_RCC_TIM6_CLK_ENABLE();

	//2. Enable teh IRQ Of the timer 6
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

	//3. Setting the prioruty of the Irq
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 15, 0);
}



