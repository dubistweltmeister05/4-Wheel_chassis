/*
 * i2c_driver_practice.c
 *
 *  Created on: Dec 16, 2023
 *      Author: wardawg
 */
#include "stm32F407xx.h"

/*
 * Peripheral Control API, used to (en/dis)able the peripheral
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= 0x01;
	}
	else {
		pI2Cx->CR1 &= ~0x01;
	}
}

/*
 * PeriClockControl, used to enable or disable the clock, and only the clock(PCLK) for the Peripheral
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		if (pI2Cx == I2C1)
			I2C1_PCLK_EN();
		if (pI2Cx == I2C2)
			I2C2_PCLK_EN();
		if (pI2Cx == I2C3)
			I2C3_PCLK_EN();
	}
	else{
		if (pI2Cx == I2C1)
			I2C1_PCLK_DI();
		if (pI2Cx == I2C2)
			I2C2_PCLK_DI();
		if (pI2Cx == I2C3)
			I2C3_PCLK_DI();
	}
}


/*
 * The initialization and deinit API
 */
void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t tempreg;

	//For the ACK control
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;

	//for the freq field
	tempreg|= 0;
	tempreg | RCC_GetPCKL1Value()/1000000;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//For the Slave address
	tempreg | pI2CHandle->I2C_Config->I2C_DeviceAddress << 1;
	tempreg |= 1<<14;
	pI2CHandle->pI2Cx->OAR1 |= (tempreg);

	// for the CCR
	uint16_t ccr_val=0;
	tempreg = 0;
	if (pI2CHandle->I2C_Config->I2C_SCLSpeed<= I2C_SCL_SPEED_SM){
		//standard mode
		ccr_val = RCC_GetPCKL1Value()/(2*pI2CHandle->I2C_Config->I2C_SCLSpeed);
		tempreg |= (ccr_val&0xFFF);
	}
	else {
		//Fast mode

		//enable the bit no 15
		tempreg |= 1<<15;

		//Feed the mode value in the bit no  14
		tempreg |= pI2CHandle->I2C_Config->I2C_FMDutyCycle;

		if (pI2CHandle->I2C_Config->I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_val = RCC_GetPCKL1Value()/(3*pI2CHandle->I2C_Config->I2C_SCLSpeed);

		}
		else if (pI2CHandle->I2C_Config->I2C_FMDutyCycle == I2C_FM_DUTY_16_9){
			ccr_val = RCC_GetPCKL1Value()/(25*pI2CHandle->I2C_Config->I2C_SCLSpeed);
		}
		tempreg |= (ccr_val&0xFFF);
	}
	pI2CHandle->pI2Cx->CCR |= (tempreg);


}
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
		I2C1_REG_RESET();
	if(pI2Cx == I2C2)
		I2C2_REG_RESET();
	if(pI2Cx == I2C3)
		I2C3_REG_RESET();

}

/*
 * The sender and receiver
 */


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)   );

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR)   );

	//5. clear the ADDR flag according to its software sequence
	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6. send the data until len becomes 0

	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) ); //Wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) );

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF) );


	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	if(Sr == I2C_DISABLE_SR )
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr){
	//1. Generate the Start Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm the success by checking the SB FLag
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)   );

	//3.Send the Slave Address with the r/nw bit as 1(R)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	//4.wait till the ADDR Phase is done by checking the ADDR Flag in the SR1
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));


	//for reading the only 1 byte
	if (Len == 1){
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

		//Clear the ADDR
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//Wait till the RxNE becomes 1
		while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));

		//generate the stop condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//read the Data in the buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

		return;
	}

	//for reading data from slave when Len > 1
	if (Len >1){
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//read the data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--){
			//wait until RXNE becomes 1
			while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));

			if(i == 2){ //if last 2 bytes are remaining

				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				//generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			}
			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			*pRxBuffer ++;


		}
	}
	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
		{
			I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
		}

}


/*
 * Helpers
 */
uint32_t RCC_GetPCKL1Value(void){
	uint8_t APB1PreVal[]={2,4,8,16};
	uint8_t AHBPreVal[]={2,4,8,16,32,64,128,256,512};

	uint32_t PCLK1,SystemClk,apb1;
	uint8_t clksrc,temp,ahbp;
	clksrc = ((RCC->CFGR >> 2) & 0x3);
	if (clksrc == 0)
		SystemClk = 16000000;
	else if (clksrc == 1)
		SystemClk = 8000000;

	temp=((RCC->CFGR >> 4) & 0xF);

	if(temp<8){
		ahbp = 1;
	}
	else{
		ahbp= AHBPreVal[temp-8];
	}

	temp= ((RCC->CFGR >> 10) & 0x7);
	if (temp<4)
		apb1=1;
	else{
		apb1=APB1PreVal[temp-4];
	}
	PCLK1 = (SystemClk/ahbp)/apb1;

	return PCLK1;
}


static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= (1); //SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->DR = SlaveAddr;
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if (EnOrDi == ENABLE){
		pI2Cx->CR1 |= (1<<I2C_CR1_ACK);
	}
	else{
		pI2Cx->CR1 &= ~(1<<I2C_CR1_ACK);
	}
}

void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle )
{
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize  == 1)
			{
				//first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}

		}
		else
		{
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}

	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}


}






















