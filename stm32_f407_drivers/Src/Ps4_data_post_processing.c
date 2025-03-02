/*
 * Ps4_data_post_processing.c
 *
 *  Created on: Dec 19, 2023
 *      Author: wardawg
 */

#include "stm32F407xx.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
extern void initialise_monitor_handles(void);
void led_init(){
	GPIO_Handle_t led;
	led.pGPIOx = GPIOD;
	led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	led.GPIO_PinConfig.GPIO_PinOPType = GPIO_NO_PUPD;
	led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&led);

	led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&led);

	led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&led);

	led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&led);

}
int main(){
	//	get the god printf all set up and ready to go

	initialise_monitor_handles();
	//	get the LED all set up

	led_init();

	uint8_t arr[6] = {69, 20, -124, 250, 32};
	uint8_t temp_1 = arr[3];
//	uint8_t temp_2 = arr[4];

	printf("The number is %d\n", temp_1);
	uint8_t status = 0;
	for (int i = 0; i < 8; i++)
	{
		status = temp_1 & (0x01);
		printf("The bit at %d position is %d\n", i, status);
		temp_1 = (temp_1 >> 1);
		// printf("The number is %d\n", temp);
	}
	return 0;

}

