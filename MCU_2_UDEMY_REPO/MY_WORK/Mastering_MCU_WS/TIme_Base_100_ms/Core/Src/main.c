/*
 * main.c
 *
 *  Created on: Dec 26, 2023
 *      Author: wardawg
 */

#include <main.h>

/*
 * Handle Variables for the Peripherals
 */
GPIO_InitTypeDef led;
TIM_HandleTypeDef htimer6;

/*
 * Headers for the Functions
 */
void SystemClockConfig();

void Error_Handler();

void timer6_init();

void GPIO_Init();

int main() {

	HAL_Init();
	SystemClockConfig();
	GPIO_Init();
	timer6_init();

	//Let's start the timer
	HAL_TIM_Base_Start_IT(&htimer6);


	return 0;
}

void SystemClockConfig() {

}

void timer6_init() {
	htimer6.Instance = TIM6; //Standard macro, defined to the Base address of the timer 6.
	htimer6.Init.Prescaler = 24;
	htimer6.Init.Period = 6400 - 1;
	if (HAL_TIM_Base_Init(&htimer6) != HAL_OK) {
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
}

void Error_Handler() {
	while (1)
		;
}
