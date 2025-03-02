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

/*
 * First demo of Post processing
 *
for (int i =0;i<5;i++){
	if (rcv_buf[i]==69){
		printf("you naughty bastard!!\n");
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
	}
	if(rcv_buf[i]==255){
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
	}
	if(rcv_buf[i]==1){
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_14);
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_15);
	}

}
 */
/*  ***************************************************************************************************************************************
 * 19/12/2023.
 * 10:45 AM->
 *
 * Alright, GGs for completing the execution of this so far.
 * Now, we bout to take the last 2 bytes that represent the button pressing on the bloody controller
 * and post process that into something useful basically.
 *
 * What we need to do is identify which button is pressed. We get that from the bit values of the byte
 * that we have received from the last 2 indexes of the rcv_buf. If a bit is 1, the that MF is set. If
 * 0, then it aint't pressed.
 *
 * So, what I am thinking, is that a byte is 8 bits (genius, ain't I?). So we copy it into a temp
 * variable first and rotate temp 8 times, and for each rotation, we mask out all the other values but
 * rightmost (LSB) bit.
 *
 * Put that value into a status variable and based on the button, we get an led to blink. I'mma try that
 * on a c compiler first, as this is no more an embedded task, but rather a coding task.
 *
 ***************************************************************************************************************************************
 * 5:11 PM->
 *
 * Okay, we passed the review. But, as always, we have a functional modification that we need to make.
 * Soooo, we gotta identify if a button is clicked or PRESSED.
 *
 * Here's a scenario.
 *
 * Suppose we have a button that we need to press in order to increase the speed of the motor, and another
 * one in order to decrease the speed of the same mf. If it is clicked once, and released immediately, then
 * we gotta do it once. If it is pressed, then keep doin it.
 * Like all things, it is HELLA simple of the virgin board(ARDUINO).SO let's think this through shall we?
 *
 * # we get 2 samples of the 5 byte array of the Ps4 Data, and put it in temp_1, and temp_2.
 * # essentially. temp_2 is the current sample and temp_1 is the previous sample.
 * # Now, we run a loop 8 times. Each time, we check corresponding bits of the samples.
 * # If they don't match, then we can say that we have a click. If they do, then we do not have a click.
 *
 * But hold up, we need to get a status_1 and status_2 situation to go on in order to check the bits
 * and a counter (==i) to show the bit being looked at.
 *
 * status_x = temp_x & 0x01;
 * temp_1 = temp_1 >> 1
 *
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "stm32f407xx.h"

extern void initialise_monitor_handles();


#define MY_ADDR 0x61;

#define SLAVE_ADDR  0x68

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;

//rcv buffer
uint8_t rcv_buf[6];

/*
 * PB6-> SCL
 * PB7 -> SDA
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);


	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);


}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

}
void led_gpio_inits(){
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



int main(void)
{

	uint8_t commandcode;

	uint8_t len=5;

	initialise_monitor_handles();

	printf("Application is running\n");

	//Led Configurations
	led_gpio_inits();

	GPIO_ButtonInit();
	printf("Button set Alpha-3\n");

	//i2c pin inits
	I2C1_GPIOInits();
	printf("Peripheral Pins Initialized Gold Eagle\n");

	//i2c peripheral configuration
	I2C1_Inits();
	printf("Peripheral configured Kate \n");

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);
	printf("And enabled too lads\n");

	//ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);
	printf("Ack good to go boys\n");

	while(1)
	{
		//wait till button is pressed
//		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
//		delay();
		//
		//		commandcode = 0x51;
		//
		//		I2C_MasterSendData(&I2C1Handle,&commandcode,1,SLAVE_ADDR,I2C_ENABLE_SR);
		//
		//		I2C_MasterReceiveData(&I2C1Handle,&len,1,SLAVE_ADDR,I2C_ENABLE_SR);

		commandcode = 0x05;
		I2C_MasterSendData(&I2C1Handle,&commandcode,1,SLAVE_ADDR,I2C_ENABLE_SR);


		I2C_MasterReceiveData(&I2C1Handle,rcv_buf,len,SLAVE_ADDR,I2C_DISABLE_SR);

		rcv_buf[len+1] = '\0';
		for (int i=0;i<len;i++){
			printf("%d : %d\n",i,rcv_buf[i]);
		}


//		uint8_t temp_1 = rcv_buf[3];
//		printf("The number is %d\n", temp_1);
//		uint8_t status = 0;
//		for (int i = 0; i < 8; i++)
//		{
//			status = temp_1 & (0x01);
//			printf("The bit at %d position is %d\n", i, status);
//			temp_1 = (temp_1 >> 1);
//			// printf("The number is %d\n", temp);
//		}

	}
	return 0;
}


/*
 *getClick,a function used to define custom actions for a particular button.
 */


bool getClick(uint8_t i, uint8_t temp_1)
{
	i=7-i;
    uint8_t prev_Status[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    temp_1 = temp_1 >> i;
    uint8_t status = temp_1 & 0x01;
    if (prev_Status[i] ^ status)
    {
        if (status == 1)
        {
            prev_Status[i] = 1;
            return 1;
        }
        if (status == 0)
        {
            prev_Status[i] = 0;
            return 0;
        }
    }
}
