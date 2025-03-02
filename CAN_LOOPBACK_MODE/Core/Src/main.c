/*
 * main.c
 *
 *  Created on: Jan 25, 2024
 *      Author: wardawg
 */

#include "stm32f4xx_hal.h"

#include "main.h"

void SystemClock_Config(uint8_t CLOCK_FREQ);

void CAN1_Init(void);

extern void initialise_monitor_handles(void);

CAN_HandleTypeDef hcan1;

int main() {

	initialise_monitor_handles();

	HAL_Init();
	SystemClock_Config(SYS_CLK_FREQ_168);

	while(1);

	return 0;
}

void Error_handler() {
	while (1)
		;
}
void CAN1_Init(void)
{
	hcan1.Instance = CAN1;
	hcan1.Init.Mode = CAN_MODE_LOOPBACK;
	hcan1.Init.AutoBusOff = ENABLE;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;

	//Settings related to CAN bit timings
	hcan1.Init.Prescaler = 5;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;

	if ( HAL_CAN_Init (&hcan1) != HAL_OK)
	{
		Error_handler();
	}

}
void SystemClock_Config(uint8_t CLOCK_FREQ) {
	RCC_OscInitTypeDef osc_init;
	RCC_ClkInitTypeDef clk_init;
	uint32_t FLatency = 0;

	osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	osc_init.HSEState = RCC_HSE_BYPASS;
	osc_init.PLL.PLLState = RCC_PLL_ON;
	osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	switch (CLOCK_FREQ) {
	case SYS_CLK_FREQ_50: {
		osc_init.PLL.PLLM = 8;
		osc_init.PLL.PLLN = 100;
		osc_init.PLL.PLLP = 2;
		osc_init.PLL.PLLQ = 2;

		clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
		RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

		FLatency = FLASH_ACR_LATENCY_1WS;

		break;
	}
	case SYS_CLK_FREQ_84: {
		osc_init.PLL.PLLM = 8;
		osc_init.PLL.PLLN = 168;
		osc_init.PLL.PLLP = 2;
		osc_init.PLL.PLLQ = 2;
		clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
		RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

		FLatency = FLASH_ACR_LATENCY_2WS;
		break;
	}
	case SYS_CLK_FREQ_120: {
		osc_init.PLL.PLLM = 8;
		osc_init.PLL.PLLN = 240;
		osc_init.PLL.PLLP = 2;
		osc_init.PLL.PLLQ = 2;

		clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
		RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV4;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

		FLatency = FLASH_ACR_LATENCY_3WS;
		break;
	}
	case SYS_CLK_FREQ_168: {

		//Enable the Clock for the Power Controller
		__HAL_RCC_PWR_CLK_ENABLE();

		//Set Voltage Scale As 1
		__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

		osc_init.PLL.PLLM = 8;
		osc_init.PLL.PLLN = 336;
		osc_init.PLL.PLLP = 2;
		osc_init.PLL.PLLQ = 2;

		clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
		RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV4;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

		FLatency = FLASH_ACR_LATENCY_5WS;
		break;
	}
	default:
		return;

	}

	if (HAL_RCC_OscConfig(&osc_init) != HAL_OK) {
		printf(
				"Gold Eagle Actual to Bravo 6-> We have a problem in the PLL OSC init\n");
		Error_handler();
	}
	printf(
			"Gold Eagle Actual to Bravo 6-> The PLL OSC is confirmed. We are GO for clock Init\n");
	if (HAL_RCC_ClockConfig(&clk_init, FLatency) != HAL_OK) {
		printf(
				"Gold Eagle Actual to Bravo 6-> We have a problem in the PLL CLOCK init\n");
		Error_handler();
	}
	printf(
			"Gold Eagle Actual to Bravo 6-> The PLL ClOCK is confirmed. Standby for further tasking\n");
	printf("Bravo 6 to Gold Eagle Actual-> Acknowledged. Standing By\n");

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
//	printf("The SYSCLK is --> %ld\n", HAL_RCC_GetSysClockFreq());
	printf("The HCLK is --> %ld\n", HAL_RCC_GetHCLKFreq());
}
