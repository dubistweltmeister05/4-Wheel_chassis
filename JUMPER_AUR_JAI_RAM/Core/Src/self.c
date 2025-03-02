/*
 * Are your dreams really worth trying if they don't let you sleep?
 */

/*
 * Good morning lads and lasses. Welcome aboard the RCS-168M.
 * Wish you all a speedy win!
 */

/*
 * Instead of the big header files, we have a pair of files (.h and .c)
 * for each of the peripherals that we have initialized via the cube.
 */

/*
 * Okay, so here's an idea. The variable names for each component is going to be the
 * initials and the shirt numbers of a Manchester united player, in accordance with
 * the role that they played in the club
 *
 * WHY?
 *
 * Because it's MY system. And we are the greatest team in the competition, just like Manchester United
 *
 * That's why.
 *
 * And, because I find it funny.
 *
 * Here's a table, so that anyone who's NOT me has a chance of understanding this.
 * (I know I am diabolical, but this is a bit too funny to me)
 *
 * PS4 data buffer --> PS18 (Paul Scholes 18, for the absolute controller that he was in the mid-field)
 * BNO data object --> RK16 (Roy Keane 16, slotting in at CDM, man was the balance that set us up for success)
 * BTS 1 velocity  --> CR7  (Cristiano Ronaldo 7, for the wheel on left wing. May you run as fast as his free-kick)
 * BTS 2 velocity  --> DB7  (David Bechkam 7, for the wheel on the right wing. May you run as smooth as his cross)
 * BTS 3 velocity  --> PE3  (Patrice Evra 3, for the wheel at left back. May you run as reliably as his overlaps)
 * BTS 4 velocity  --> GN2  (Gary Neville 2, for the wheel at right back. May you run as long as his career)
 *
 * And, some additional clearance as needed to understand this.
 * TIMER3 ch 1 & 2 --> BTS 1 RPWM & LPWM (CR7)
 * TIMER3 ch 3 & 4 --> BTS 2 RPWM & LPWM (DB7)
 * TIMER4 ch 1 & 2 --> BTS 3 RPWM & LPWM (PE3)
 * TIMER4 ch 3 & 4 --> BTS 4 RPWM & LPWM (GN2)
 *
 */

#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include "stdio.h"
#include "bno055_stm32.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <main.h>
#include <stm32f407xx.h>

/*
 * Macro Definitions
 */

#define MY_ADDR 0x61;

#define SLAVE_ADDR  0x68

#define SYS_CLK_FREQ_50  50

#define SYS_CLK_FREQ_84  84

#define SYS_CLK_FREQ_120 120

#define SYS_CLK_FREQ_168 168

/*
 * Headers for the Functions
 */

extern void initialise_monitor_handles(void);

void System_Clock_Config(uint8_t CLOCK_FREQ);

void SystemClock_Config();

void Error_Handler();

void timer4_init();

void timer3_init();

void gpio_timer4_Init();

void gpio_timer3_Init();

void I2C2_GPIOInits(void);

void I2C2_Inits(void);

long map(long x, long in_min, long in_max, long out_min, long out_max);

void movement(float v, float v_w, float angle, float KP, float KD);

/*
 * Handle Variables for the Peripherals
 */
GPIO_InitTypeDef led;

TIM_HandleTypeDef htimer4;

TIM_HandleTypeDef htimer3;

volatile uint32_t ccr_content;

I2C_Handle_t I2C2Handle;

uint8_t PS_18[7];

bno055_vector_t RK16;

int CR7, DB7, GN2, PE3; //v1,v2,v3,v4

float targated_angle = 0, curr_angle = 0, error = 0, prev_error = 0,
		correction = 0, diff = 0;

float angle;

float KP = 2.0, KD = 0.9;

float x, y, w, v, v_x, v_y, v_w;

/***********************************************************************************************************************
 * 														MAIN CODE
 ***********************************************************************************************************************/

int main() {
	//Main Scope Variables and the Welcome Text!
	uint8_t commandcode;

	uint8_t len = 5;

//	uint32_t brightness;
	initialise_monitor_handles();

	printf("Bravo-6 to Gold Eagle Actual--> Going Dark\n");
	/***********************************************************************************************************************
	 * 												INITIALIZATION AND DAT
	 ***********************************************************************************************************************/
	//Calling All Initialization Functions
	//1. The Main HAL_INIT
	HAL_Init();

	// We gotta run at 168, so here's the one for that
	System_Clock_Config(SYS_CLK_FREQ_168);

	//BNO inits
	MX_GPIO_Init();
	MX_I2C1_Init();

	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();

	//The GPIO-Acts-As-Timer Init
	gpio_timer4_Init();
	gpio_timer3_Init();

	//The actual Timer Init
	timer4_init();
	timer3_init();

	//The GPIO-Acts-As-I2C1 Init
	I2C2_GPIOInits();

	//The Actual I2C1 Init
	I2C2_Inits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C2, ENABLE);

	//ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C2, I2C_ACK_ENABLE);

	/*******************************************************************************************************************
	 *												Logic starts Here
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
		/***********************************************************************************************************************
		 * 												CONTROL FROM THE PS_18
		 ***********************************************************************************************************************/

		commandcode = 0x05;
		I2C_MasterSendData(&I2C2Handle, &commandcode, 1, SLAVE_ADDR,
		I2C_ENABLE_SR);

		I2C_MasterReceiveData(&I2C2Handle, PS_18, len, SLAVE_ADDR,
		I2C_DISABLE_SR);

		//Little printf to know what the hell is being sent
		printf(" x: %d  y: %d  w: %d b1: %d b2: %d\n", PS_18[0], PS_18[1],
				PS_18[2], PS_18[3], PS_18[4]);

		/***********************************************************************************************************************
		 * 											PROCESSING OF DATA RECEIVED FROM PS_18
		 ***********************************************************************************************************************/

		//Converting the LeftHatX value to a suitable range
		if ((PS_18[0] < 138) && (PS_18[0] > 116))
			x = 0;
		else
			x = map(PS_18[0], 0, 255, -127, 127);
		//Converting the LeftHatY value to a suitable range
		if ((PS_18[1] < 138) && (PS_18[1] > 116))
			y = 0;
		else
			y = map(PS_18[1], 0, 255, -127, 127);

		v = sqrt((pow(x, 2) + pow(y, 2)));

		v = map(v, 0, 150, 0, 100);

		v_w = map(w, 0, 255, 0, 43);

		angle = atan2(y, x);

		/***********************************************************************************************************************
		 * 												BALANCE FROM RK16
		 ***********************************************************************************************************************/

		RK16 = bno055_getVectorEuler();
		//We care only about the heading [;-)]
		printf("Heading: %.2f\r\n", RK16.x);
		curr_angle = (RK16.x);
		if (curr_angle > 180)
			curr_angle = curr_angle - 360;

		/***********************************************************************************************************************
		 * 												COMMANDS TO THE WIDEOUTS
		 ***********************************************************************************************************************/

		movement(v, v_w, angle, KP, KD);
	}
	return 0;
}

void movement(float v, float v_w, float angle, float KP, float KD) {
	if (v > 95)
		v = 95;

	v_x = v * cos(angle);

	v_y = v * sin(angle);

	//PID

	error = curr_angle - targated_angle;
	if (error < -180) {
		error = error + 360;
	}

	else if (error > 180) {
		error = error - 360;
	}

	diff = error - prev_error;

	correction = (error * KP) + (diff * KD);

	prev_error = error;

	CR7 = ((0.70711 * (-v_x)) + (0.707 * (v_y)) - (0.707 * v_w)) + correction;
	DB7 = ((0.70711 * (-v_x)) + (0.707 * (-v_y)) - (0.707 * v_w)) + correction;
	GN2 = ((0.70711 * (v_x)) + (0.70711 * (-v_y)) - (0.707 * v_w)) + correction;
	PE3 = ((0.70711 * (v_x)) + (0.707 * (v_y)) - (0.707 * v_w)) + correction;

	if (CR7 > 0) {
		__HAL_TIM_SET_COMPARE(&htimer3, TIM_CHANNEL_1, CR7);
		__HAL_TIM_SET_COMPARE(&htimer3, TIM_CHANNEL_2, 0);
	} else {
		__HAL_TIM_SET_COMPARE(&htimer3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htimer3, TIM_CHANNEL_2, (CR7 * (-1)));
	}

	if (DB7 > 0) {
		__HAL_TIM_SET_COMPARE(&htimer3, TIM_CHANNEL_3, DB7);
		__HAL_TIM_SET_COMPARE(&htimer3, TIM_CHANNEL_4, 0);
	} else {
		__HAL_TIM_SET_COMPARE(&htimer3, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htimer3, TIM_CHANNEL_4, (DB7 * (-1)));

	}

	if (GN2 > 0) {
		__HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_1, GN2);
		__HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_2, 0);
	} else {
		__HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_2, (GN2 * (-1)));

	}

	if (PE3 > 0) {
		__HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_3, PE3);
		__HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_4, 0);

	} else {
		__HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_4, (PE3* (-1)));

	}

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

	printf("Timer 4 is running\n");

}

void timer3_init() {
	TIM_OC_InitTypeDef tim3_PWM_Config;
	htimer3.Instance = TIM3; //Standard macro, defined to the Base address of the timer 6.
	htimer3.Init.Prescaler = 4999;
	htimer3.Init.Period = 256 - 1;
	if (HAL_TIM_PWM_Init(&htimer3) != HAL_OK) {
		Error_Handler();
	}
	memset(&tim3_PWM_Config, 0, sizeof(tim3_PWM_Config));

	tim3_PWM_Config.OCMode = TIM_OCMODE_PWM1;
	tim3_PWM_Config.OCPolarity = TIM_OCPOLARITY_HIGH;

	tim3_PWM_Config.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htimer3, &tim3_PWM_Config, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}

	tim3_PWM_Config.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htimer3, &tim3_PWM_Config, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();

	}

	tim3_PWM_Config.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htimer3, &tim3_PWM_Config, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}

	tim3_PWM_Config.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htimer3, &tim3_PWM_Config, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}

	printf("Timer 3 is running\n");

}

void gpio_timer4_Init() {
	GPIO_InitTypeDef timer4_gpio;
	timer4_gpio.Mode = GPIO_MODE_AF_PP;
	timer4_gpio.Alternate = GPIO_AF2_TIM4;
	timer4_gpio.Pull = GPIO_NOPULL;
	timer4_gpio.Speed = GPIO_SPEED_FAST;
	timer4_gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;

	HAL_GPIO_Init(GPIOD, &timer4_gpio);

	printf("Timer GPIOs Are Running\n");
}
void gpio_timer3_Init() {
	GPIO_InitTypeDef timer3_gpio;
	timer3_gpio.Mode = GPIO_MODE_AF_PP;
	timer3_gpio.Alternate = GPIO_AF2_TIM3;
	timer3_gpio.Pull = GPIO_NOPULL;
	timer3_gpio.Speed = GPIO_SPEED_FAST;
	timer3_gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;

	HAL_GPIO_Init(GPIOD, &timer3_gpio);

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

void I2C2_GPIOInits(void) {
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&I2CPins);

	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	GPIO_Init(&I2CPins);

}

void I2C2_Inits(void) {
	I2C2Handle.pI2Cx = I2C2;
	I2C2Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C2Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR
	;
	I2C2Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C2Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C2Handle);

}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
