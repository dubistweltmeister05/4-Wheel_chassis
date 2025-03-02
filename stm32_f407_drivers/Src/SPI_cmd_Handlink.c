/*
 * SPI_cmd_Handlink.c
 *
 *  Created on: Dec 13, 2023
 *      Author: wardawg
 */

/*
 * Header Includes
 */

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stm32F407xx.h>


/*
 * Start of the code
 */


//Command Macros
#define COMMAND_LED_CTRL  	0x50
#define COMMANDSENSOR_READ 	0x51
#define COMMAND_LED_READ 	0x52
#define COMMAND_PRINT 		0x53
#define COMMAND_ID_READ 	0x54

#define LED_ON   			1
#define LED_OFF				0

//Arduino Analog Pins
#define ARDUINO_PIN0		0
#define ARDUINO_PIN1		1
#define ARDUINO_PIN2		2
#define ARDUINO_PIN3		3
#define ARDUINO_PIN4		4

//arduino led pin
#define ARDUINO_LED			13

/*
 * Helper Functions
 */
void delay(){
	for (int i=0;i<5000;i++);
}
extern void initialise_monitor_handles(void);



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

uint8_t SPI_VerifyResponse(uint8_t ackbyte){
	if(ackbyte == 0x05){
		//ACK
		return 1;
	}
	return 0;
}


int main(){

	initialise_monitor_handles();
	printf("Okay, the universe shouldn't have given me this power, cause now imma go crazyyyyy with it\n");
	printf("Nahh, Imma have toooo muchhh funnnn woth this shit bruhhhh\n");

	uint8_t dummy = 0xff;
	uint8_t dummy_read;
	//Create a user buffer to hold the data
	//char *user_buffer = "I'm tryna be what I'm destined to be";


	//This is to initialize the button code
	button_init();


	//THis initialized the GPIO pins in the SPI Mode
	SPI_GPIO_Inits();

	//This initializes the SPI Configuration as per the need
	SPI_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);
	while(1){
		//Check the button press and wait
		while(!(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0)));

		printf("HERE WE GO\n");

		//to avoid the de-bounce
		delay();

		//We need to explicitly ENABLE the SPE Bit in the CR1
		SPI_PeripheralControl(SPI2, ENABLE);

		/*
		 * Command 1 is the LED_CONTROL <pin_no>  <value>
		 */
		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];
		SPI_SendData(SPI2, &commandcode, 1);

		//Do a DUMMY READ to clear the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Upon successful reception, we have the slave send an ACK bit.
		//To read that, we need to send a dummy byte first.
		SPI_SendData(SPI2, &dummy, 1);

		//Now, we receive the ack byte at the master
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		if(SPI_VerifyResponse(ackbyte)){
			//Set Arguments
			args[0]=ARDUINO_LED;
			args[1] = LED_ON;


			//Send Arguments
			SPI_SendData(SPI2,args,2);
		}
		//Confirm that SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		//Disable the Peripheral, save yourself some battery
		SPI_PeripheralControl(SPI2, DISABLE);
	}

}



