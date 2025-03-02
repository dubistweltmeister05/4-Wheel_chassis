/*
 * spi_drivers_practice.c
 *
 *  Created on: Dec 12, 2023
 *      Author: wardawg
 */
#include<stm32F407xx.h>
#include <spi_driver_practice.h>


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(*pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		if(*pSPIx == SPI2){
			SPI2_PCLK_EN();
		}
		if(*pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
	}
	else{
		if(*pSPIx == SPI1){
			SPI1_PCLK_DI();
		}
		if(*pSPIx == SPI2){
			SPI2_PCLK_DI();
		}
		if(*pSPIx == SPI3){
			SPI3_PCLK_DI();
		}
	}
}
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	//peripheral clock enable

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//first lets configure the SPI_CR1 register

	uint32_t tempreg = 0;

	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR ;

	//2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= ( 1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= ( 1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4.  Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6 . configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;

}

void SPI_DeInit(SPI_RegDef_t *pSPIx){

}


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flag){
	if (pSPIx->SR & (Flag)){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

//This is a blocking call
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t len){
	while(len >0){
		//1. Wait till the TxE is set.
		while((SPI_GetFlagStatus(&pSPIx, SPI_TXE_FLAG)) == FLAG_RESET);

		//2. Check the DFF Bits
		if(pSPIx->CR1 &(1<<SPI_CR1_DFF)){
			//We have 16 bit data (DFF)
			pSPIx->DR = *(uint16_t*)pTxBuffer;
			len--;
			len--;
			(uint16_t*)pTxBuffer++;

		}
		else{
			//we have 8 bits of data (DFF)
			pSPIx->DR = *(pTxBuffer);
			len--;
			pTxBuffer++;
		}

	}

}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t len){
	while(len >0){
		//1. Wait till the TxE is set.
		while((SPI_GetFlagStatus(&pSPIx, SPI_RXNE_FLAG)) == FLAG_RESET);

		//2. Check the DFF Bits
		if(pSPIx->CR1 &(1<<SPI_CR1_DFF)){
			//We have 16 bit data (DFF)
			*(uint16_t*)pRxBuffer = pSPIx->DR;
			len--;
			len--;
			(uint16_t*)pRxBuffer++;

		}
		else{
			//we have 8 bits of data (DFF)
			*(pTxBuffer)=pSPIx->DR;
			len--;
			pRxBuffer++;
		}

	}

}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){

}


void SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t len){
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX){
		//1. Save the TxBuffer address and length in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;

		//2. Mark SPI as BusyInTx so that nothing takes over till transmission is complete
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get an interrupt whenever the TXE Flag is set in the SR
		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);

		//4. Data transmission is handled by the ISR Code

	}
	return state;
}

void SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t len){
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_RX){
		//1. Save the TxBuffer address and length in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;

		//2. Mark SPI as BusyInTx so that nothing takes over till transmission is complete
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the TXEIE control bit to get an interrupt whenever the TXE Flag is set in the SR
		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_RXNEIE);

		//4. Data transmission is handled by the ISR Code

	}
	return state;
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){

	uint8_t temp1, temp2;
	//checking for the TXE flag error first
	temp1 = pSPIHandle->pSPIx->SR & (1<<SPI_TXE_FLAG);
	temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE);

	if(temp1&&temp2)
	{
		spi_handle_txe(pSPIHandle);
	}

	//checking for the RXNE flag error first
	temp1 = pSPIHandle->pSPIx->SR & (1<<SPI_RXNE_FLAG);
	temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_RXNEIE);

	if(temp1&&temp2)
	{
		spi_handle_rxne(pSPIHandle);
	}

	//checking for the ovr flag error first
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
		//handle ovr error
		spi_handle_ovr_err(pSPIHandle);
	}
}
/*
 * Helper APIs
 */


vois SPI_PreipheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	}
	else{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, Uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1<<SPI_CR1_SSI);
	}
	else{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, Uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1<<SPI_CR1_SSOE);
	}
	else{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSOE);
	}
}

void spi_handle_txe(SPI_Handle_t *pSPIHandle){
	//Check the DFF
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){
		//16 bit DFF

		//Load data into the DR
		pSPIHandle->pSPIx->DR = *(uint16_t*)pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else {
		//8 bit DFF
		pSPIHandle->pSPIx->DR = *(uint8_t*)pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		(uint8_t*)pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen){
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}
void spi_handle_rxne(SPI_Handle_t *pSPIHandle){
	//Check the DFF
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){
		//16 bit DFF

		//Read Data from register
		*(uint16_t*)pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}
	else {
		//8 bit DFF

		//Read Data from the DR
		*(uint8_t*)pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		(uint8_t*)pSPIHandle->pRxBuffer++;
	}
	if(! pSPIHandle->RxLen)
	{
		//reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}

void spi_handle_ovr_err(SPI_Handle_t *pSPIHandle){
	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}



void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}
