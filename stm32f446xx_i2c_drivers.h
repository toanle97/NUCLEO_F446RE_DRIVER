/*
 * stm32f446xx_i2c_drivers.h
 *
 *  Created on: 29 thg 8, 2020
 *      Author: toan
 */

#ifndef INC_STM32F446XX_I2C_DRIVERS_H_
#define INC_STM32F446XX_I2C_DRIVERS_H_

#include "stm32f446xx.h"

typedef struct{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_AddressMode;
	uint16_t I2C_SlaveAddress;
	uint8_t I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
	uint8_t I2C_MasterRW;
}I2C_Config_t;

typedef struct{
	I2C_Regdef_t* pI2Cx;
	I2C_Config_t  I2C_Config;
	uint8_t *pTxBuff;
	uint8_t *pRxBuff;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState;
	uint32_t Rxsize;
	uint8_t DevAddress;
	uint8_t SRepeated;
}I2C_Handle_t;

/*
 * Read/Write
 */
#define I2C_READ           	1
#define I2C_WRITE			0

/*
 * SPEED
 */
#define I2C_SCLSpeed_SM		100000
#define I2C_SCLSpeed_FM4K	400000
#define I2C_SCLSpeed_FM2K	200000

/*
 * Address Mode
 */
#define I2C_AddressMode_7bits	0
#define I2C_AddressMode_10bits  1

/*
 * ACK Control
 */
#define I2C_ACKControl_DIS		0
#define I2C_ACKControl_EN		1

/*
 * FMDutyCycle
 */
#define I2C_FM_2				0
#define I2C_FM_16_9				1

/*
 * I2C Application State
 */
#define I2C_READY				0
#define I2C_BUSY_IN_RX			1
#define I2C_BUSY_IN_TX			2
#define I2C_EV_TX_CMPT			3
#define I2C_EV_STOP				4
#define I2C_EV_RX_CMPT			5
#define I2C_ER_BERR 			6
#define I2C_ER_ARLO  			7
#define I2C_ER_AF    			8
#define I2C_ER_OVR   			9
#define I2C_ER_TIMEOUT			10



void I2C_PeriClockControl(I2C_Regdef_t *, uint8_t state);
void I2C_PeriEnable(I2C_Regdef_t *, uint8_t state);
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_Regdef_t *pI2Cx);

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *TxBuff, uint32_t len, uint8_t SlaveAdress);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *RxBuff, uint32_t len, uint8_t SlaveAddress);

void I2C_SlaveSendData(I2C_Regdef_t *, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_Regdef_t*);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *TxBuff, uint32_t len, uint8_t SlaveAdress, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *RxBuff, uint32_t len, uint8_t SlaveAddress, uint8_t Sr);

void I2C_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t state);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *);
void I2C_ApplicationEventCallback(I2C_Handle_t *, uint8_t state);


#endif /* INC_STM32F446XX_I2C_DRIVERS_H_ */
