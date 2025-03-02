/*
 * Ps4_Interfacing_Arduino.c
 *
 *  Created on: Dec 21, 2023
 *      Author: wardawg
 */
/*
 * STM SIDE --->
 * PB6 --> SCL
 * PB9 --> SDA
 *
 *
 * ARDUINO SIDE --->
 * A5 --> SCL
 * A4 --> SDA
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"
#include <stdbool.h>

extern void initialise_monitor_handles();
/*
 * Helper Function headers
 */
int map_for_ps(uint8_t val, int in_min, int in_max, int out_min, int out_max);
bool getClick(uint8_t i, uint8_t temp_1);

#define MY_ADDR 0x61;

#define SLAVE_ADDR  0x68

void delay(void) {
	for (uint32_t i = 0; i < 500000 / 2; i++)
		;
}

I2C_Handle_t I2C1Handle;

//rcv buffer
uint8_t rcv_buf[7];
/*
 * PB6-> SCL
 * PB7 -> SDA
 */

void I2C1_GPIOInits(void) {
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);

}

void I2C1_Inits(void) {
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR
	;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);

}

void GPIO_ButtonInit(void) {
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

}
void led_gpio_inits() {
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

int main(void) {

	uint8_t commandcode;

	uint8_t len = 5;

	initialise_monitor_handles();

	printf("Application is running\n");

	//Led Configurations
	led_gpio_inits();

	GPIO_ButtonInit();

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	//ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while (1) {
		//wait till button is pressed
		//		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		//		delay();

		commandcode = 0x05;
		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR,
				I2C_ENABLE_SR);

		I2C_MasterReceiveData(&I2C1Handle, rcv_buf, len, SLAVE_ADDR,
				I2C_DISABLE_SR);

		printf(" x: %d  y: %d  w: %d b1: %d b2: %d\n", rcv_buf[0],rcv_buf[1],rcv_buf[2],rcv_buf[3],rcv_buf[4]);

	}
	return 0;
}

/*
 *getClick,a function used to define custom actions for a particular button.
 */

bool getClick(uint8_t i, uint8_t temp_1) {
	i = 7 - i;
	uint8_t prev_Status[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	temp_1 = temp_1 >> i;
	uint8_t status = temp_1 & 0x01;
	if (prev_Status[i] ^ status) {
		if (status == 1) {
			prev_Status[i] = 1;
			return 1;
		}
		if (status == 0) {
			prev_Status[i] = 0;
			return 0;
		}
	}
}

int map_for_ps(uint8_t val, int in_min, int in_max, int out_min, int out_max) {
	int ret_val = ((val - in_min) * (out_max - (out_min)) / (in_max - in_min)
			+ (out_min));
	return (ret_val);

}

