/*
 * msp..c
 *
 *  Created on: Dec 26, 2023
 *      Author: wardawg
 */
#include "stm32f4xx_hal.h"

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
void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
	GPIO_InitTypeDef gpio_uart;
	//Here,we do the Low Level Peripheral Initializations.

	//1. Enable the clock for the USART2 peripheral
	__HAL_RCC_USART2_CLK_ENABLE();

	//2. Do the pin muxing configurations
	gpio_uart.Pin = GPIO_PIN_2;
	gpio_uart.Mode = GPIO_MODE_AF_PP;
	gpio_uart.Pull = GPIO_PULLUP;
	gpio_uart.Speed = GPIO_SPEED_FAST;
	gpio_uart.Alternate = GPIO_AF7_USART2;

	(HAL_GPIO_Init(GPIOA, &gpio_uart));

	gpio_uart.Pin = GPIO_PIN_3;
	(HAL_GPIO_Init(GPIOA, &gpio_uart));

	//3. Enabler the IRQ and set the priorities.
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);

}
