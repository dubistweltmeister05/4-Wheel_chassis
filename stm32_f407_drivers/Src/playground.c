/*
 * playground.c
 *
 *  Created on: Dec 12, 2023
 *      Author: wardawg
 */

#include "stm32F407xx.h"
#include "stm32F407_spi_driver.h"
#include <string.h>

/*
 * PB15 -> MOSI
 * PB14 -> MISO
 * PB13 -> SCLK
 * PB12 -> NSS
 *
 * ALTFN - Mode 5
 *
 */
void delay(){
	for (int i=0;i<500000;i++);
}

void SPI_GPIO_Inits(){
	GPIO_Handle_t spi_gpio;
	spi_gpio.pGPIOx = GPIOB;
	spi_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	spi_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	spi_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	spi_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	spi_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	spi_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&spi_gpio);

	spi_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&spi_gpio);

	spi_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&spi_gpio);

	spi_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&spi_gpio);


}

void SPI_Inits(){
	SPI_Handle_t spi2;

	spi2.pSPIx = SPI2;
	spi2.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	spi2.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	spi2.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_HD;
	spi2.SPIConfig.SPI_CPHA =SPI_CPHA_LOW;
	spi2.SPIConfig.SPI_CPOL =SPI_CPOL_LOW;
	spi2.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	spi2.SPIConfig.SPI_SSM = SPI_SSM_DI; //Hardware Slave MGMT

	SPI_Init(&spi2);
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

	//Create a user buffer to hold the data
	char *user_buffer = "I'm tryna be what I'm destined to be";

	//THis initialized the GPIO pins in the SPI Mode
	SPI_GPIO_Inits();

	//This is to initialize the button code
	button_init();

	//This initializes the SPI Configuration as per the need
	SPI_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);
	while(1){
		//Check the button press and wait
		while(!(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0)));

		//to avoid the de-bounce
		delay();

		//We need to explicitly ENABLE the SPE Bit in the CR1
		SPI_PeripheralControl(SPI2, ENABLE);

		//first, send the length information
		uint8_t dataLen = strlen(user_buffer);
		SPI_SendData(SPI2, &dataLen,1);

		//Send Traffic
		SPI_SendData(SPI2,(uint8_t*)user_buffer, (uint8_t)strlen(user_buffer));

		//Confirm that SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		//Disable the Peripheral, save yourself some battery
		SPI_PeripheralControl(SPI2, DISABLE);
	}

}
