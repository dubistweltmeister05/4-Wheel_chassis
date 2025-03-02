/*
 * it.c
 *
 *  Created on: Dec 26, 2023
 *      Author: wardawg
 */

#include <main.h>

extern TIM_HandleTypeDef htimer4;
void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void TIM_4_IRQHandler(){
	HAL_TIM_IRQHandler(&htimer4);
}
