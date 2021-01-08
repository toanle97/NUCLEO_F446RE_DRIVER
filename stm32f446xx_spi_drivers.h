/*
 * stm32f446xx_spi_drivers.h
 *
 *  Created on: 26 thg 6, 2020
 *      Author: toan
 */

#ifndef INC_STM32F446XX_SPI_DRIVERS_H_
#define INC_STM32F446XX_SPI_DRIVERS_H_

#include "stm32f446xx.h"
#include "stdio.h"

typedef struct{
	uint8_t SPI_Mode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_DFF;
	uint8_t SPI_SSM;
	uint8_t SPI_Speed;
	uint8_t SPI_FF;
}SPI_Config_t;

typedef struct{
	SPI_RegDef_t 	*pSPIx;
	SPI_Config_t	SPIConfig;
	uint8_t 		*pTxBuff;
	uint8_t 		*pRxBuff;
	uint32_t		TxLen;
	uint32_t		RxLen;
	uint8_t			TxState;
	uint8_t			RxState;
}SPI_Handle_t;

typedef enum{
	SPI_STATE_READY,
	SPI_STATE_BUSY_IN_TX,
	SPI_STATE_BUSY_IN_RX
}SPI_Status_t;

typedef enum{
	SPI_EVENT_TX_COMPLETE,
	SPI_EVENT_RX_COMPLETE,
	SPI_EVENT_OVR_COMPLETE
}SPI_Event_Status_t;

/*
 * @SPI_MODE
 */
#define SPI_MODE_SLAVE			0
#define SPI_MODE_MASTER			1

/*
 * @SPI_BUSCofig
 */
#define SPI_BusConfig_FD		0
#define SPI_BusConfig_HD		1
#define SPI_BusConfig_RXOnly	2

/*
 * SPI_CPOL
 */
#define SPI_CPOL_LOW			0
#define SPI_CPOL_HIGH			1

/*
 * SPI_CPHA
 */
#define SPI_CPHA_LOW			0
#define SPI_CPHA_HIGH			1

/*
 * SPI_DFF
 */
#define SPI_DFF_8Bits			0
#define SPI_DFF_16Bits			1

/*
 * SPI_SSM
 */
#define SPI_SSM_DIS				0
#define SPI_SSM_EN				1

/*
 * SPI_Speed
 */
#define SPI_Speed_DIV2			0
#define SPI_Speed_DIV4			1
#define SPI_Speed_DIV8			2
#define SPI_Speed_DIV16			3
#define SPI_Speed_DIV32			4
#define SPI_Speed_DIV64			5
#define SPI_Speed_DIV128		6
#define SPI_Speed_DIV256		7

/*
 * SPI_FF
 */
#define SPI_FF_Motorola			0
#define SPI_FF_TI				1

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t state);
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

void SPI_SendData(SPI_Handle_t *pSPIHandle, uint8_t *txBuff, uint8_t len);
void SPI_ReceiveData(SPI_Handle_t *pSPIHandle, uint8_t *rxBuff, uint8_t len);

void SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuff, uint32_t Len);
void SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuff, uint32_t Len);
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t EventStatus);

void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t state);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

#endif /* INC_STM32F446XX_SPI_DRIVERS_H_ */
