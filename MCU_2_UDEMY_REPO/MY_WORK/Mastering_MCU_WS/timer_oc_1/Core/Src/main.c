/*
 * main.c
 *
 *  Created on: Dec 26, 2023
 *      Author: wardawg
 */
#include <stdio.h>
#include <main.h>

/*
 * Handle Variables for the Peripherals
 */
GPIO_InitTypeDef led;
TIM_HandleTypeDef htimer4;
volatile uint32_t ccr_content;

/*
 * Headers for the Functions
 */
extern void initialise_monitor_handles(void);

void SystemClock_Config(uint8_t CLOCK_FREQ);

void SystemClockConfig();

void Error_Handler();

void timer4_init();

void GPIO_Init();

uint32_t pulse1_val = 25000;

uint32_t pulse2_val = 12500;

uint32_t pulse3_val = 6250;

uint32_t pulse4_val = 3125;

int main() {
	initialise_monitor_handles();

	printf("Bravo-6 to Gold Eagle Actual -> Going Dark\n");
	HAL_Init();
	SystemClock_Config(SYS_CLK_FREQ_50);
	GPIO_Init();
	timer4_init();
	if (HAL_TIM_OC_Start_IT(&htimer4, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_OC_Start_IT(&htimer4, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_OC_Start_IT(&htimer4, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_OC_Start_IT(&htimer4, TIM_CHANNEL_4) != HAL_OK) {
		Error_Handler();
	}

	while (1)
		;
	return 0;
}

void timer4_init() {

	TIM_OC_InitTypeDef tim4oc_init;
	htimer4.Instance = TIM4; //Standard macro, defined to the Base address of the timer 6.
	htimer4.Init.Prescaler = 1;
	htimer4.Init.Period = 0xFFFFFFFF;
	if (HAL_TIM_OC_Init(&htimer4) != HAL_OK) {
		Error_Handler();
	}
	tim4oc_init.OCMode = TIM_OCMODE_TOGGLE;
	tim4oc_init.OCPolarity = TIM_OCPOLARITY_HIGH;
	tim4oc_init.Pulse = pulse1_val;
	if (HAL_TIM_OC_ConfigChannel(&htimer4, &tim4oc_init, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}

	tim4oc_init.Pulse = pulse2_val;
	if (HAL_TIM_OC_ConfigChannel(&htimer4, &tim4oc_init, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}

	tim4oc_init.Pulse = pulse3_val;
	if (HAL_TIM_OC_ConfigChannel(&htimer4, &tim4oc_init, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}

	tim4oc_init.Pulse = pulse4_val;
	if (HAL_TIM_OC_ConfigChannel(&htimer4, &tim4oc_init, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
}

void GPIO_Init() {
	__HAL_RCC_GPIOD_CLK_ENABLE();
	led.Pin = GPIO_PIN_15;
	led.Mode = GPIO_MODE_OUTPUT_PP;
	led.Pull = GPIO_NOPULL;
	led.Speed = GPIO_SPEED_FAST;

	HAL_GPIO_Init(GPIOD, &led);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
}

void Error_Handler() {
	while (1)
		;
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
		Error_Handler();
	}
	printf(
			"Gold Eagle Actual to Bravo 6-> The PLL OSC is confirmed. We are GO for clock Init\n");
	if (HAL_RCC_ClockConfig(&clk_init, FLatency) != HAL_OK) {
		printf(
				"Gold Eagle Actual to Bravo 6-> We have a problem in the PLL CLOCK init\n");
		Error_Handler();
	}
	printf(
			"Gold Eagle Actual to Bravo 6-> The PLL ClOCK is confirmed. Standby for further tasking\n");
	printf("Bravo 6 to Gold Eagle Actual-> Acknowledged. Standing By\n");

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	/* TIM3_CH1 toggling with frequency = 500 Hz */
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		ccr_content = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, ccr_content + pulse1_val);
	}

	/* TIM3_CH2 toggling with frequency = 1000 Hz */
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		ccr_content = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, ccr_content + pulse2_val);

	}

	/* TIM3_CH3 toggling with frequency = 2000 Hz */
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		ccr_content = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, ccr_content + pulse3_val);

	}

	/* TIM3_CH4 toggling with frequency = 4000 Hz */
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
		ccr_content = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, ccr_content + pulse4_val);

	}
}

