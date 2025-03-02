/*
 * led_toggle.c
 *
 *  Created on: Dec 10, 2023
 *      Author: wardawg
 */

#include <stdio.h>
#include "stm32F407xx.h"
#include "stm32F407_gpio_driver.h"
extern void initialise_monitor_handles(void);
void delay(){
	for (int i=0;i<500000;i++);
}

void gpio_inits(){
	GPIO_Handle_t gpio;
	gpio.pGPIOx = GPIOD;
	gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;

	GPIO_Init(&gpio);
	gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;

	GPIO_Init(&gpio);
	gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;

	GPIO_Init(&gpio);
	gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;

	GPIO_Init(&gpio);

}

void button_init(){
	GPIO_Handle_t Button;
	Button.pGPIOx = GPIOA;
	Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&Button);

}

int main(){
	initialise_monitor_handles();

	gpio_inits();
	button_init();


	while(1){

		if(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)){
			printf("Bravo -6 send traffic \n");
			delay();
			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, ENABLE);
			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_13, ENABLE);
			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_14, ENABLE);
			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_15, ENABLE);
			delay();
//			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, DISABLE);
//			delay();
			printf("Executed successfully Alpha -3. Good to Go Dark.\n");



		}
	}

	return 0;

}



