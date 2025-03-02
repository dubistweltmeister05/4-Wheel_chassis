/*
 * main.c
 *
 *  Created on: Dec 26, 2023
 *      Author: wardawg
 */

#include "stm32f4xx_hal.h"
#include <main.h>

/*
 * Handle Variables for the Peripherals
 */
UART_HandleTypeDef huart2;

/*
 * Headers for the Functions
 */
void SystemClockConfig();
void UART2_Inits();
void Error_Handler();

int main() {

	HAL_Init();
	SystemClockConfig();
	UART2_Inits();

	return 0;
}

void SystemClockConfig() {

}
void UART2_Inits() {
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;

	if ((HAL_UART_Init(&huart2)) != HAL_OK) {
		Error_Handler();
	}
}

void Error_Handler() {
	while (1)
		;
}
