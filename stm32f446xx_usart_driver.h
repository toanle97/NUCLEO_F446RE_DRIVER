/*
 * stm32f446xx_usart_driver.h
 *
 *  Created on: 14 thg 8, 2020
 *      Author: toan
 */

#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_

#include "stm32f446xx.h"

/*
 * USART MODE
 */
#define USART_TX_ONLY		1
#define USART_RX_ONLY		2
#define USART_TX_RX			3

/*
 * USART Number of stopbits
 */
#define USART_Stopbits_1	0
#define USART_Stopbits_0_5	1
#define USART_Stopbits_2	2
#define USART_Stopbits_1_5	3

/*
 * USART Word Length
 */
#define USART_WordLen_8		0
#define USART_WordLen_9 	1

/*
 * USART Parity control
 */
#define USART_PARITY_EN_ODD   		2
#define USART_PARITY_EN_EVEN  		1
#define USART_PARITY_DISABLE   		0
#define USART_PARITY_ENABLE			3

/*
 * USART Hardware Flow Control
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	2
#define USART_HW_FLOW_CTRL_RTS    	1
#define USART_HW_FLOW_CTRL_CTS_RTS	3
/*
 * USART BaudRate
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					2400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define USART_STD_BAUD_3M 					3000000

/*
 * USART Sampling
 */
#define USART_OVER8_0		0
#define USART_OVER8_1		1

/*
 * USART IRQ Number
 */
#define USART1_IRQ_NO		37
#define USART2_IRQ_NO		38
#define USART3_IRQ_NO		39
#define UART4_IRQ_NO		52
#define UART5_IRQ_NO		53
#define USART6_IRQ_NO		71

typedef enum{
	USART_STATE_READY,
	USART_STATE_BUSY_IN_TX,
	USART_STATE_BUSY_IN_RX
}USART_Status_t;

typedef enum{
	USART_EVENT_TX_COMPLETE,
	USART_EVENT_RX_COMPLETE,
	USART_EVENT_OVR_COMPLETE
}USART_Event_Status_t;

typedef struct{
	uint8_t USART_MODE;
	uint32_t USART_BRR;
	uint8_t USART_Stopbits;
	uint8_t USART_WordLen;
	uint8_t USART_Paritybits;
	uint8_t USART_HWcontrol;
	uint8_t USART_Sampling;
}USART_Config_t;

typedef struct{
	USART_Regdef_t *USARTx;
	USART_Config_t USART_Config;
	uint8_t 		*pTxBuff;
	uint8_t 		*pRxBuff;
	uint32_t		TxLen;
	uint32_t		RxLen;
	uint8_t			TxState;
	uint8_t			RxState;
}USART_Handle_t;

void USART_PeriClockControl(USART_Regdef_t *pUSARTx, uint8_t state);
void USART_PeriControl(USART_Regdef_t *pUSARTx, uint8_t state);

void USART_Init(USART_Handle_t *pUSART_Handle);
void USART_DeInit(USART_Regdef_t *pUSARTx);

void USART_SendData(USART_Handle_t * pUSARTx, uint8_t *TxBuff, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSART_Handle, uint8_t *pRxBuff, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTx, uint8_t *TxBuff, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTx, uint8_t *TxBuff, uint32_t Len);

uint8_t USART_GetFlagStatus(USART_Regdef_t *pUSARTx, uint8_t StatusFlagName);
void USART_ClearFlag(USART_Regdef_t *pUSARTx, uint8_t StatusFlagName);

void USART_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t state);
void USART_IRQHandling(USART_Handle_t *usartHandle);

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t event);

#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
