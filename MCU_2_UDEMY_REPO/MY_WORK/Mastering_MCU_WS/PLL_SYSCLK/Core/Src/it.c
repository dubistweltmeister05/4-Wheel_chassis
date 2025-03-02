/*
 * it.c
 *
 *  Created on: Dec 26, 2023
 *      Author: wardawg
 */


#include"main.h"

void SysTick_Handler(){
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
