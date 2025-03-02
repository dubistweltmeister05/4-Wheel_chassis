/*
 * button_Interrrupt_led.c
 *
 *  Created on: Dec 10, 2023
 *      Author: wardawg
 */
#include "stm32F407xx.h"
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
	Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&Button);

}
int main(){
	gpio_inits();
	button_init();

	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);
	while(1);
	return 0;
}
void EXTI0_IRQHandler(){
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	for(int i=0;i<10;i++){
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, ENABLE);
	delay();
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, DISABLE);
	delay();
	i++;
	}
}


