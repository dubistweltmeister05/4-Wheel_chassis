/*
 * it.c
 *
 *  Created on: Dec 26, 2023
 *      Author: wardawg
 */

#include <main.h>

extern TIM_HandleTypeDef htimer6;
void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void TIM6_DAC_IRQHandler(){
	HAL_TIM_IRQHandler(&htimer6);
}
