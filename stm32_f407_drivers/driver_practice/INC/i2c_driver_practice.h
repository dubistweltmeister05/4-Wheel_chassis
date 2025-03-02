/*
 * i2c_driver_practice.h
 *
 *  Created on: Dec 16, 2023
 *      Author: wardawg
 */

#ifndef INC_I2C_DRIVER_PRACTICE_H_
#define INC_I2C_DRIVER_PRACTICE_H_

/*
 * The Configuration Structure
 */

typedef struct{
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_AckControl;
	uint16_t I2C_FMDutyCycle;
	uint32_t I2C_SCLSpeed;
}I2C_Config_t;

/*
 * The Handle Structure
 */

typedef struct {
	I2C_RegDef_t *pI2Cx; //To access the registers of the I2C Peripheral
	I2C_Config_t pI2CHandle; //TO access the configurable elements of the I2C Peripheral
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM    100000
#define I2C_SCL_SPEED_FM4k  400000
#define I2C_SCL_SPEED_FM2k  200000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE    	1
#define I2C_ACK_DISABLE    	0

/*
 * @I2C_FMDutyCycle
 */
#define i2C_FMDuty_2		0
#define i2C_FMDuty_16_9		1



/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Send and receive data APIs
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint8_t len, uint8_t SlaveAddr,uint8_t Sr);

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 * Other peripheral Control APIs
 */
void I2C_PreipheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_SSIConfig(I2C_RegDef_t *pI2Cx, Uint8_t EnOrDi);
void I2C_SSOEConfig(I2C_RegDef_t *pI2Cx, Uint8_t EnOrDi);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName);





























































#endif /* INC_I2C_DRIVER_PRACTICE_H_ */
