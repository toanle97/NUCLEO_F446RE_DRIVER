/*
 * stm32f446xx_spi_drivers.c
 *
 *  Created on: 26 thg 6, 2020
 *      Author: toan
 */

#include "stm32f446xx_spi_drivers.h"
#include "stddef.h"

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t state){
	if(state == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}else if(pSPIx == SPI4){
			SPI4_PCLK_EN();
		}
	}else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DIS();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_DIS();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_DIS();
		}else if(pSPIx == SPI4){
			SPI4_PCLK_DIS();
		}
	}
}

void SPI_Init(SPI_Handle_t *pSPIHandle){
	//1. Configure SPI Mode
	pSPIHandle -> pSPIx-> CR1 &= ~(1 << 2);
	pSPIHandle -> pSPIx-> CR1 |= (pSPIHandle -> SPIConfig.SPI_Mode << 2);

	//2. Configure SPI Bus
	if(pSPIHandle -> SPIConfig.SPI_BusConfig == SPI_BusConfig_FD){
		pSPIHandle -> pSPIx-> CR1 &= ~(1 << 15);
	}else if(pSPIHandle -> SPIConfig.SPI_BusConfig == SPI_BusConfig_HD){
		pSPIHandle -> pSPIx-> CR1 |= (1 << 15);
	}else if(pSPIHandle -> SPIConfig.SPI_BusConfig == SPI_BusConfig_RXOnly){
		pSPIHandle ->pSPIx-> CR1 &= ~(1 << 15);
		pSPIHandle ->pSPIx-> CR1 |= (1 << 10);
	}

	//3. Configure SPI CPOL
	pSPIHandle -> pSPIx-> CR1 &= ~(1 << 1);
	pSPIHandle -> pSPIx-> CR1 |= (pSPIHandle ->SPIConfig.SPI_CPOL << 1);

	//4. Configure SPI PHASE
	pSPIHandle -> pSPIx-> CR1 &= ~(1 << 0);
	pSPIHandle -> pSPIx-> CR1 |= (pSPIHandle ->SPIConfig.SPI_CPHA);

	//5.Configure SPI Data Frame Format
	pSPIHandle -> pSPIx -> CR1 &= ~(1 << 11);
	pSPIHandle -> pSPIx -> CR1 |= (pSPIHandle -> SPIConfig.SPI_DFF << 11);

	//6. Configure SPI Speed
	pSPIHandle -> pSPIx -> CR1 &= ~(7 << 3);
	pSPIHandle -> pSPIx -> CR1 |= (pSPIHandle -> SPIConfig.SPI_Speed << 3);

	//7. Configure SPI Software Select Management
	pSPIHandle -> pSPIx -> CR1 &= ~(1 << 9);
	pSPIHandle -> pSPIx -> CR1 |= (pSPIHandle -> SPIConfig.SPI_SSM << 9);

	//8. Configure SPI Frame Format
	pSPIHandle -> pSPIx -> CR2 &= ~(1 << 4);
	pSPIHandle -> pSPIx -> CR2 |= (pSPIHandle -> SPIConfig.SPI_FF << 4);
}

void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1){
		RCC -> APB2RSTR &= ~(1 << 12);
	}else if(pSPIx == SPI2){
		RCC -> APB1RSTR &= ~(1 << 14);
	}else if(pSPIx == SPI3){
		RCC -> APB1RSTR &= ~(1 << 15);
	}else if(pSPIx == SPI4){
		RCC -> APB2RSTR &= ~(1 << 13);
	}
}

void SPI_SendData(SPI_Handle_t *pSPIHandle, uint8_t *txBuff, uint8_t len){
	//1. Check if there is any data to transmit
	while(len > 0){
		//2. Check TxBuffer is empty
		while(!(pSPIHandle -> pSPIx-> SR & (1 << 1)));
		//3. Check data frame
		if(((pSPIHandle -> pSPIx -> CR1) & (1 << 11)) == 0){
			pSPIHandle -> pSPIx-> DR = *txBuff;
			txBuff++;
			len--;
		}else if(((pSPIHandle -> pSPIx -> CR1) & (1 << 11)) == 1){
			pSPIHandle -> pSPIx-> DR = *((uint16_t *)txBuff);
			(uint16_t*)txBuff++;
			len -=2;
		}
	}
}
void SPI_ReceiveData(SPI_Handle_t *pSPIHandle, uint8_t *rxBuff, uint8_t len){
	while(len > 0){
		//2. Check RxBuffer is not empty
		while(!(pSPIHandle -> pSPIx-> SR & (1 << 0)));
		//3. Check data frame
		if(((pSPIHandle -> pSPIx -> CR1) & (1 << 11)) == 0){
			*rxBuff = pSPIHandle -> pSPIx-> DR;
			rxBuff++;
			len--;
		}else if(((pSPIHandle -> pSPIx -> CR1) & (1 << 11)) == 1){
			*((uint16_t *)rxBuff) = pSPIHandle -> pSPIx-> DR;
			(uint16_t *)rxBuff++;
			len -= 2;
		}
	}
}
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t state){
	uint32_t temp1, temp2;
	temp1 = IRQNumber / 4;
	temp2 = IRQNumber % 4;
	if(state == ENABLE){
		if(IRQNumber < 32){
			NVIC_ISER -> ISER[0] |= (1 << IRQNumber);
		}else if(32 <= IRQNumber && IRQNumber < 64){
			NVIC_ISER -> ISER[1] |= (1 << ((IRQNumber % 32)));
		}else if(64 <= IRQNumber && IRQNumber < 96){
			NVIC_ISER -> ISER[2] |= (1 << ((IRQNumber % 64)));
		}
		NVIC_IPR -> IPR[temp1] |= IRQPriority << ((temp2 * 8) + 4);
	}else if(state == DISABLE){
		if(IRQNumber < 32){
			NVIC_ICER -> ICER[0] |= (1 << IRQNumber);
		}else if(32 <= IRQNumber && IRQNumber < 64){
			NVIC_ICER -> ICER[1] |= (1 << ((IRQNumber % 32)));
		}else if(64 <= IRQNumber && IRQNumber < 96){
			NVIC_ICER -> ICER[2] |= (1 << ((IRQNumber % 64)));
		}
	}
}
void SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuff, uint32_t Len){
	//1. Check if SPI is ready
	if(pSPIHandle -> TxState == SPI_STATE_READY){
		//2. Save TxBuff Address and Len into global variable in order to use later
		pSPIHandle -> pTxBuff = pTxBuff;
		pSPIHandle -> TxLen = Len;
		//3. Change state of SPI
		pSPIHandle -> TxState = SPI_STATE_BUSY_IN_TX;
		//4. Enable TXEIE Interrupt
		pSPIHandle -> pSPIx -> CR2 |= (1 << 7);
	}
}
void SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuff, uint32_t Len){
	if(pSPIHandle -> RxState == SPI_STATE_READY){
		pSPIHandle -> pRxBuff = pRxBuff;
		pSPIHandle -> RxLen = Len;

		pSPIHandle -> RxState = SPI_STATE_BUSY_IN_RX;
		pSPIHandle -> pSPIx -> CR2 |= (1 << 6);
		}
}
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
	//1. Check which source has caused the interrupt
	uint8_t temp1, temp2;
	temp1 = pSPIHandle -> pSPIx -> CR2 & (1 << 7);
	temp2 = pSPIHandle -> pSPIx -> SR & (1 << 1);
	if(temp1 && temp2){
		if(pSPIHandle -> SPIConfig.SPI_DFF == SPI_DFF_16Bits){
			while(pSPIHandle -> TxLen > 0){
				pSPIHandle -> pSPIx -> DR = *(uint16_t *)pSPIHandle->pTxBuff;
				(uint16_t *)pSPIHandle -> pTxBuff ++;
				pSPIHandle -> TxLen -=2;
			}
		}else if(pSPIHandle -> SPIConfig.SPI_DFF == SPI_DFF_8Bits){
			while(pSPIHandle -> TxLen > 0){
				pSPIHandle -> pSPIx -> DR = *(pSPIHandle->pTxBuff);
				pSPIHandle -> pTxBuff ++;
				pSPIHandle -> TxLen --;
			}
		}
		//2. When Len == 0, Close transmission and inform the app that Tx is over
		pSPIHandle -> pSPIx -> CR2 &= ~(1 << 7);
		pSPIHandle -> pTxBuff = NULL;
		pSPIHandle -> TxLen = 0;
		pSPIHandle -> TxState = SPI_STATE_READY;
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLETE);
	}

	temp1 = pSPIHandle -> pSPIx -> CR2 & (1 << 6);
	temp2 = pSPIHandle -> pSPIx -> SR & (1 << 0);
	if(temp1 && temp2){
		if(pSPIHandle -> SPIConfig.SPI_DFF == SPI_DFF_16Bits){
			*(uint16_t *)pSPIHandle->pRxBuff = (uint16_t)pSPIHandle -> pSPIx -> DR;
			(uint16_t *)pSPIHandle -> pRxBuff ++;
			pSPIHandle -> RxLen -= 2;
		}else if(pSPIHandle -> SPIConfig.SPI_DFF == SPI_DFF_8Bits){
			*pSPIHandle -> pRxBuff = pSPIHandle -> pSPIx -> DR;
			pSPIHandle -> pRxBuff ++;
			pSPIHandle -> RxLen --;
		}
		if(pSPIHandle -> RxLen == 0){
			pSPIHandle -> pSPIx -> CR2 &= ~(1 << 6);
			pSPIHandle -> pRxBuff = NULL;
			pSPIHandle -> RxLen = 0;
			pSPIHandle -> RxState = SPI_STATE_READY;
			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE);
		}
	}

	temp1 = pSPIHandle -> pSPIx -> CR2 & (1 << 5);
	temp2 = pSPIHandle -> pSPIx -> SR & (1 << 6);
	if(temp1 && temp2){
		uint8_t temp_data;
		if(pSPIHandle -> TxState != SPI_STATE_BUSY_IN_TX){
			temp_data = pSPIHandle -> pSPIx -> DR;
			temp_data = pSPIHandle -> pSPIx -> SR;
		}
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_COMPLETE);
	}
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t EventStatus){

}
