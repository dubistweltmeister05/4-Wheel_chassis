/*
 * I2C_Mstr_Send_Data.c
 *
 *  Created on: Dec 16, 2023
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




#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stm32F407xx.h>

#define SlaveAddr  0x68

#define MY_ADDR 0x61;
I2C_Handle_t I2C1Handle;
void delay(){
	for (int i=0;i<500000/2;i++);
}



void I2C1_GPIO_Inits(){
	GPIO_Handle_t i2c_gpio;
	i2c_gpio.pGPIOx = GPIOB;
	i2c_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2c_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	i2c_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCL
	i2c_gpio.GPIO_PinConfig.GPIO_PinNumber = 6;
	GPIO_Init(&i2c_gpio);

	//SDA
	i2c_gpio.GPIO_PinConfig.GPIO_PinNumber = 9;
	GPIO_Init(&i2c_gpio);


}

void I2C1_Inits(){

	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;

	I2C_Init(&I2C1Handle);



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
	uint8_t some_data[] = "We are testing the master Tx \n";

	//initialize the button
	button_init();

	//for the GPIO Pins initialization
	I2C1_GPIO_Inits();

	//for the I2C peripheral
	I2C1_Inits();

	//Enable the I2C Peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	while(1){
		while(!(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0)));

		delay();

		//Send Some data bruv
		I2C_MasterSendData(&I2C1Handle,some_data, strlen((char*)some_data), SlaveAddr,0);
	}
	return 0;
}
