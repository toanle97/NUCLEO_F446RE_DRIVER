/*
 * stm32f446xx_usart_driver.c
 *
 *  Created on: 14 thg 8, 2020
 *      Author: toan
 */

#include "stm32f446xx_usart_driver.h"
#include "stdbool.h"

void USART_PeriClockControl(USART_Regdef_t *pUSARTx, uint8_t state){
	if(state == ENABLE){
		if(pUSARTx == USART1){
			USART1_PCLK_EN();
		}else if(pUSARTx == USART2){
			USART2_PCLK_EN();
		}else if(pUSARTx == USART3){
			USART3_PCLK_EN();
		}else if(pUSARTx == UART4){
			UART4_PCLK_EN();
		}else if(pUSARTx == UART5){
			UART5_PCLK_EN();
		}else if(pUSARTx == USART6){
			USART6_PCLK_EN();
		}
	}else if(state == DISABLE){
		if(pUSARTx == USART1){
			USART1_PCLK_DIS();
		}else if(pUSARTx == USART2){
			USART2_PCLK_DIS();
		}else if(pUSARTx == USART3){
			USART3_PCLK_DIS();
		}else if(pUSARTx == UART4){
			UART4_PCLK_DIS();
		}else if(pUSARTx == UART5){
			UART5_PCLK_DIS();
		}else if(pUSARTx == USART6){
			USART6_PCLK_DIS();
		}
	}
}
void USART_PeriControl(USART_Regdef_t *pUSARTx, uint8_t state){
	if(state == ENABLE){
		pUSARTx -> CR1 |= (1 << 13);
	}else{
		pUSARTx -> CR1 &= ~(1 << 13);
	}
}

void USART_Init(USART_Handle_t *pUSART_Handle){
	//1. Configure USART MODE
	if(pUSART_Handle -> USART_Config.USART_MODE == USART_TX_ONLY){
		pUSART_Handle -> USARTx -> CR1 |= (1 << 3);
	}else if(pUSART_Handle -> USART_Config.USART_MODE == USART_RX_ONLY){
		pUSART_Handle -> USARTx -> CR1 |= (1 << 2);
	}else if(pUSART_Handle -> USART_Config.USART_MODE == USART_TX_RX){
		pUSART_Handle -> USARTx -> CR1 |= (3 << 2 );
	}

	//2. Configure Number of bits
	if(pUSART_Handle -> USART_Config.USART_WordLen == USART_WordLen_8){
		pUSART_Handle -> USARTx -> CR1 &= ~(1 << 12);
	}else if(pUSART_Handle -> USART_Config.USART_WordLen == USART_WordLen_9){
		pUSART_Handle -> USARTx -> CR1 |= (1 << 12);
	}

	//3. Configure Parity bit
	if(pUSART_Handle -> USART_Config.USART_Paritybits == USART_PARITY_DISABLE){
		pUSART_Handle -> USARTx -> CR1 &= ~(1 << 10);
	}else if(pUSART_Handle -> USART_Config.USART_Paritybits == USART_PARITY_ENABLE){
		if(pUSART_Handle -> USART_Config.USART_Paritybits == USART_PARITY_EN_ODD){
			pUSART_Handle -> USARTx -> CR1 |= (1 << 9);
		}else if(pUSART_Handle -> USART_Config.USART_Paritybits == USART_PARITY_EN_EVEN){
			pUSART_Handle -> USARTx -> CR1 &= ~(1 << 9);
		}
	}

	//4. Configure Stop bits
	pUSART_Handle -> USARTx -> CR2 &= ~(3 << 12);
	pUSART_Handle -> USARTx -> CR2 |= (pUSART_Handle -> USART_Config.USART_Stopbits);

	//5. Configure Hardware Flow
	pUSART_Handle -> USARTx -> CR3 &= ~(3 << 8);
	pUSART_Handle -> USARTx -> CR3 |= pUSART_Handle -> USART_Config.USART_HWcontrol;

	//6. Configure BaudRate
	uint32_t udiv = 0;
	uint32_t reg_temp = 0;
	uint32_t M_part, F_part;
	uint8_t B_flag = false;
	if(pUSART_Handle -> USART_Config.USART_Sampling == USART_OVER8_0){
		udiv = 16000000/(pUSART_Handle -> USART_Config.USART_BRR * 16) * 100;
		B_flag = false;
	}else if(pUSART_Handle -> USART_Config.USART_Sampling == USART_OVER8_1){
		udiv = 16000000/(pUSART_Handle -> USART_Config.USART_BRR * 8) *100;
		B_flag = true;
	}
	M_part = udiv/100;
	reg_temp |= M_part << 4;
	if(B_flag){
		F_part = ((udiv - M_part*100) * 8 + 50) / 100;
	}else{
		F_part = ((udiv - M_part*100) * 16 + 50) / 100;
	}
	reg_temp |= F_part;
	pUSART_Handle -> USARTx -> BRR = reg_temp;
}
void USART_DeInit(USART_Regdef_t *pUSARTx){
	if(pUSARTx == USART1){
		RCC -> APB2RSTR &= ~(1 << 4);
	}else if(pUSARTx == USART6){
		RCC -> APB2RSTR &= ~(1 << 5);
	}else if(pUSARTx == USART2){
		RCC -> APB1RSTR &= ~(1 << 17);
	}else if(pUSARTx == USART3){
		RCC -> APB1RSTR &= ~(1 << 18);
	}else if(pUSARTx == UART4){
		RCC -> APB1RSTR &= ~(1 << 19);
	}else if(pUSARTx == UART5){
		RCC -> APB1RSTR &= ~(1 << 20);
	}
}
void USART_SendData(USART_Handle_t * pUSART_Handle, uint8_t *TxBuff, uint32_t Len){
	pUSART_Handle -> USARTx -> CR1 |= (1 << 13);
	uint16_t *pdata;
	while(Len > 0){
		//1. Check if TXE is set?
		while(!(pUSART_Handle -> USARTx -> SR & (1 << 7)));
		//2. Check Word Length
		if(pUSART_Handle -> USART_Config.USART_WordLen == USART_WordLen_9){
			pdata = (uint16_t *)TxBuff;
			pUSART_Handle -> USARTx -> DR = (*pdata & (uint16_t)0x1FF);
			//3. Check if Parity bit is used?
			if(pUSART_Handle -> USART_Config.USART_Paritybits == USART_PARITY_DISABLE){
				TxBuff += 2;
				Len -= 2;
			}else{
				TxBuff++;
				Len--;
			}
		}else if(pUSART_Handle -> USART_Config.USART_WordLen == USART_WordLen_8){
			pUSART_Handle -> USARTx -> DR = (*TxBuff & (uint8_t)0xFF);
			TxBuff++;
			Len--;
		}
	}
	while(!(pUSART_Handle -> USARTx -> SR & (1 << 6)));
}

void USART_ReceiveData(USART_Handle_t *pUSART_Handle, uint8_t *pRxBuff, uint32_t Len){
	pUSART_Handle -> USARTx -> CR1 |= (1 << 13);
	while(Len > 0){
		//1.Check if RXNE is set?
		while(!(pUSART_Handle -> USARTx -> SR & (1 << 5)));
		//2.Check Word Length
		if(pUSART_Handle -> USART_Config.USART_WordLen == USART_WordLen_9){
			if(pUSART_Handle -> USART_Config.USART_Paritybits == USART_PARITY_DISABLE){
				*(uint16_t *)pRxBuff = pUSART_Handle -> USARTx -> DR & (uint16_t)0x1FF;
				pRxBuff += 2;
				Len -= 2;
			}else{
				*pRxBuff = pUSART_Handle -> USARTx -> DR & (uint8_t)0xFF;
				pRxBuff++;
				Len--;
			}
		}else if(pUSART_Handle -> USART_Config.USART_WordLen == USART_WordLen_8){
			if(pUSART_Handle -> USART_Config.USART_Paritybits == USART_PARITY_DISABLE){
				*pRxBuff = pUSART_Handle -> USARTx -> DR & (uint8_t)0xFF;
			}else{
				*pRxBuff = pUSART_Handle -> USARTx -> DR & (uint8_t)0x7F;
			}
			pRxBuff++;
			Len--;
		}
	}
}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTx, uint8_t *TxBuff, uint32_t Len){
	uint8_t txState = pUSARTx -> TxState;
	if(txState != USART_STATE_BUSY_IN_TX){
		pUSARTx -> TxLen = Len;
		pUSARTx -> pTxBuff = TxBuff;
		pUSARTx -> TxState = USART_STATE_BUSY_IN_TX;

		pUSARTx -> USARTx -> CR1 |= (1 << 7);
		pUSARTx -> USARTx -> CR1 |= (1 << 6);
	}
	return txState;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTx, uint8_t *RxBuff, uint32_t Len){
	uint8_t rxState = pUSARTx -> RxState;
	if(rxState == USART_STATE_READY){
		pUSARTx -> RxLen = Len;
		pUSARTx -> pRxBuff = RxBuff;
		pUSARTx -> RxState = USART_STATE_BUSY_IN_RX;

		pUSARTx -> USARTx -> CR1 |= (1 << 5);
	}
	return rxState;
}
void USART_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t state){
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
void USART_IRQHandling(USART_Handle_t *usartHandle){
	uint32_t temp1, temp2;
	//Check if Interrupt source is TC
	 temp1 = usartHandle -> USARTx-> SR & (1 << 6);
	 temp2 = usartHandle -> USARTx -> CR1 & (1 << 7);
	 if(temp1 && temp2){
		 if(usartHandle -> TxState == USART_STATE_BUSY_IN_TX){
			 if(!usartHandle -> TxLen){
				 usartHandle -> USARTx -> SR &= ~(1 << 6);
				 usartHandle -> USARTx -> CR1 &= ~(1 << 7);
				 usartHandle -> TxState = USART_STATE_READY;
				 *(usartHandle -> pTxBuff) = 0;
				 usartHandle -> TxLen = 0;
				 USART_ApplicationEventCallback(usartHandle, USART_EVENT_TX_COMPLETE);
			 }
		 }
	 }
	 // Check if interrupt source is RXNE
	 temp1 = usartHandle -> USARTx -> SR & (1 << 5);
	 temp2 = usartHandle -> USARTx -> CR1 & (1 << 5);
	 if(temp1 && temp2){
		 if(usartHandle -> RxState == USART_STATE_BUSY_IN_RX){
			 if(usartHandle -> RxLen > 0){
				 if(usartHandle -> USART_Config.USART_WordLen == USART_WordLen_9){
					 if(usartHandle -> USART_Config.USART_Paritybits == USART_PARITY_DISABLE){
						 *((uint16_t *)usartHandle -> pRxBuff) = (usartHandle -> USARTx -> DR & 0x01FF);
						 usartHandle -> pRxBuff += 2;
						 usartHandle -> RxLen -= 2;
					 }else{
						 *((uint8_t *)usartHandle -> pRxBuff) = (usartHandle -> USARTx -> DR & 0xFF);
						 usartHandle -> pRxBuff ++;
						 usartHandle -> RxLen --;
					 }
				 }else{
					 if(usartHandle -> USART_Config.USART_Paritybits == USART_PARITY_DISABLE){
						 *((uint8_t *)usartHandle -> pRxBuff) = (usartHandle -> USARTx -> DR & 0xFF);
						 usartHandle -> pRxBuff ++;
						 usartHandle -> RxLen --;
					 }else{
						 *((uint8_t *)usartHandle -> pRxBuff) = (usartHandle -> USARTx -> DR & 0x7F);
						 usartHandle -> pRxBuff ++;
						 usartHandle -> RxLen --;
					 }
				 }
			 }if(!usartHandle -> RxLen){
				 usartHandle -> USARTx -> CR1 &= ~(1 << 5);
				 usartHandle -> RxState = USART_STATE_READY;
				 USART_ApplicationEventCallback(usartHandle, USART_EVENT_RX_COMPLETE);
			 }
		 }
	 }
}

__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t event){

}
