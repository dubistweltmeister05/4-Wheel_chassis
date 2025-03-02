/*
 * main.c
 *
 *  Created on: Dec 26, 2023
 *      Author: wardawg
 */
#include <string.h>
#include <stdio.h>
#include <main.h>
#include <stm32f407xx.h>

/*
 * Macro Definitions
 */

#define MY_ADDR 0x61;

#define SLAVE_ADDR  0x68

/*
 * Handle Variables for the Peripherals
 */
GPIO_InitTypeDef led;

TIM_HandleTypeDef htimer4;

volatile uint32_t ccr_content;

I2C_Handle_t I2C1Handle;

uint8_t rcv_buf[7];

/*
 * Headers for the Functions
 */
extern void initialise_monitor_handles(void);

void System_Clock_Config(uint8_t CLOCK_FREQ);

void SystemClock_Config();

void Error_Handler();

void timer4_init();

void gpio_timer_Init();

void I2C1_GPIOInits(void);

void I2C1_Inits(void);

int main() {
	//Main Scope Variables and the Welcome Text!
	uint8_t commandcode;

	uint8_t len = 5;

//	void SystemClock_Config();

//	uint32_t brightness;
	initialise_monitor_handles();

	printf("Bravo-6 to Gold Eagle Actual--> Going Dark\n");

	//Calling All Initialization Functions

	//1. The Main HAL_INIT
	HAL_Init();

	// We gotta run at 168, so here's the one for that
	SystemClock_Config(SYS_CLK_FREQ_168);

	//The GPIO-Acts-As-Timer Init
	gpio_timer_Init();

	//The actual Timer Init
	timer4_init();

	//The GPIO-Acts-As-I2C1 Init
	I2C1_GPIOInits();

	//The Actual I2C1 Init
	I2C1_Inits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	//ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	/*******************************************************************************************************************
	 *Logic starts Here
	 *******************************************************************************************************************/

	//Crank that Timer 4 on all the channels
	if (HAL_TIM_PWM_Start(&htimer4, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Start(&htimer4, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Start(&htimer4, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Start(&htimer4, TIM_CHANNEL_4) != HAL_OK) {
		Error_Handler();
	}

	while (1) {

		commandcode = 0x05;
		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR,
		I2C_ENABLE_SR);

		I2C_MasterReceiveData(&I2C1Handle, rcv_buf, len, SLAVE_ADDR,
		I2C_DISABLE_SR);

		printf(" x: %d  y: %d  w: %d b1: %d b2: %d\n", rcv_buf[0], rcv_buf[1],
				rcv_buf[2], rcv_buf[3], rcv_buf[4]);
		__HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_1, (rcv_buf[0] ));
		__HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_2, (rcv_buf[1] ));
		__HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_3, (rcv_buf[0]));
		__HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_4, (rcv_buf[1] ));
		HAL_Delay(1);

	}
	return 0;
}

void SystemClock_Config() {

}

void timer4_init() {
	TIM_OC_InitTypeDef tim4_PWM_Config;
	htimer4.Instance = TIM4; //Standard macro, defined to the Base address of the timer 6.
	htimer4.Init.Prescaler = 4999;
	htimer4.Init.Period = 256 - 1;
	if (HAL_TIM_PWM_Init(&htimer4) != HAL_OK) {
		Error_Handler();
	}
	memset(&tim4_PWM_Config, 0, sizeof(tim4_PWM_Config));

	tim4_PWM_Config.OCMode = TIM_OCMODE_PWM1;
	tim4_PWM_Config.OCPolarity = TIM_OCPOLARITY_HIGH;

	tim4_PWM_Config.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htimer4, &tim4_PWM_Config, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}

	tim4_PWM_Config.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htimer4, &tim4_PWM_Config, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();

	}

	tim4_PWM_Config.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htimer4, &tim4_PWM_Config, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}

	tim4_PWM_Config.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htimer4, &tim4_PWM_Config, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}

	printf("Timers are running\n");

}

void gpio_timer_Init() {
	GPIO_InitTypeDef timer4_gpio;
	timer4_gpio.Mode = GPIO_MODE_AF_PP;
	timer4_gpio.Alternate = GPIO_AF2_TIM4;
	timer4_gpio.Pull = GPIO_NOPULL;
	timer4_gpio.Speed = GPIO_SPEED_FAST;
	timer4_gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;

	HAL_GPIO_Init(GPIOD, &timer4_gpio);

	printf("Timer GPIOs Are Running\n");
}

void Error_Handler() {
	while (1)
		;
}

void System_Clock_Config(uint8_t CLOCK_FREQ) {
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
