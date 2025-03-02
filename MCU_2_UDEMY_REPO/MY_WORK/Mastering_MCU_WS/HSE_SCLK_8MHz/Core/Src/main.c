/*
 * main.c
 *
 *  Created on: Dec 26, 2023
 *      Author: wardawg
 */

#include "stm32f4xx_hal.h"
#include <main.h>
#include <string.h>

/*
 * Handle Variables for the Peripherals
 */
UART_HandleTypeDef huart2;

/*
 * Headers for the Functions
 */

void UART2_Inits();
void Error_Handler();
extern void initialise_monitor_handles(void);

int main() {
	initialise_monitor_handles();
	printf("Bravo-6 to Gold Eagle actual. going dark\n");
	RCC_OscInitTypeDef osc_init;
	RCC_ClkInitTypeDef clk_init;
	HAL_Init();

	UART2_Inits();
	printf("The values before the custom config are :- \n");
	printf("The SYSCLK is --> %ld\n", HAL_RCC_GetSysClockFreq());
	printf("The HCLK is --> %ld\n", HAL_RCC_GetHCLKFreq());
	printf("The APB1CLK is --> %ld\n", HAL_RCC_GetPCLK1Freq());
	printf("The APB2CLK is --> %ld\n", HAL_RCC_GetPCLK2Freq());
	memset(&osc_init, 0, sizeof(osc_init));
	osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	osc_init.HSEState = RCC_HSE_BYPASS;
	if (HAL_RCC_OscConfig(&osc_init) != HAL_OK) {
		printf("WE fucked up man\n");
		Error_Handler();
	}
	printf(
			"Bravo-6 to Gold Eagle actual. Initialization of clock is complete.\n");
	printf("Proceeding to complete the switch for SYSCLK\n");

	clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK| \
 RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	clk_init.AHBCLKDivider = RCC_SYSCLK_DIV2;
	clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
	clk_init.APB2CLKDivider = RCC_HCLK_DIV2;
	if (HAL_RCC_ClockConfig(&clk_init, FLASH_ACR_LATENCY_0WS) != HAL_OK) {
		printf("WE fucked up man\n");
		Error_Handler();
	}
	printf("The SYSCLK is --> %ld\n", HAL_RCC_GetSysClockFreq());
	printf("The HCLK is --> %ld\n", HAL_RCC_GetHCLKFreq());
	printf("The APB1CLK is --> %ld\n", HAL_RCC_GetPCLK1Freq());
	printf("The APB2CLK is --> %ld\n", HAL_RCC_GetPCLK2Freq());

	while (1)
		;
	return 0;
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
