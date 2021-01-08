/*
 * stm32f446xx_i2c_drivers.c
 *
 *  Created on: 29 thg 8, 2020
 *      Author: toan
 */

#include "stm32f446xx_i2c_drivers.h"
#include "stddef.h"

void I2C_PeriClockControl(I2C_Regdef_t *pI2Cx , uint8_t state){
	if(state == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	}else if(state == DISABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_DIS();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_DIS();
		}else if(pI2Cx == I2C3){
			I2C3_PCLK_DIS();
		}
	}
}

void I2C_Init(I2C_Handle_t *pI2CHandle){
	//1. Configure SCL Speed
	if(pI2CHandle -> I2C_Config.I2C_SCLSpeed == I2C_SCLSpeed_SM){
		pI2CHandle -> pI2Cx -> CCR &= ~(1 << 15);
		pI2CHandle -> pI2Cx -> CR2 |= (0x10);
		pI2CHandle -> pI2Cx -> CCR |= (0x50);
	}else if(pI2CHandle -> I2C_Config.I2C_SCLSpeed == I2C_SCLSpeed_FM2K){
		pI2CHandle -> pI2Cx -> CCR |= (1 << 15);
		if(pI2CHandle -> I2C_Config.I2C_FMDutyCycle == I2C_FM_2){
			pI2CHandle -> pI2Cx -> CCR &= ~(1 << 14);
			pI2CHandle -> pI2Cx -> CR2 |= (0x10);
			pI2CHandle -> pI2Cx -> CCR |= (0x1B);
		}else if(pI2CHandle -> I2C_Config.I2C_FMDutyCycle == I2C_FM_16_9){
			pI2CHandle -> pI2Cx -> CCR |= (1 << 14);
			pI2CHandle -> pI2Cx -> CR2 |= 0x10;
			pI2CHandle -> pI2Cx -> CCR |= 0x03;
		}
	}else if(pI2CHandle -> I2C_Config.I2C_SCLSpeed == I2C_SCLSpeed_FM4K){
		if(pI2CHandle -> I2C_Config.I2C_FMDutyCycle == I2C_FM_2){
			pI2CHandle -> pI2Cx -> CCR &= ~(1 << 14);
			pI2CHandle -> pI2Cx -> CR2 |= (0x10);
			pI2CHandle -> pI2Cx -> CCR |= (0x1D);
		}else if(pI2CHandle -> I2C_Config.I2C_FMDutyCycle == I2C_FM_16_9){
			pI2CHandle -> pI2Cx -> CCR |= (1 << 14);
			pI2CHandle -> pI2Cx -> CR2 |= 0x10;
			pI2CHandle -> pI2Cx -> CCR |= 0x02;
		}
	}
	//2. Configure ACK Control
	if(pI2CHandle -> I2C_Config.I2C_ACKControl == I2C_ACKControl_DIS){
		pI2CHandle -> pI2Cx -> CR1 &= ~(1 << 10);
	}else if(pI2CHandle -> I2C_Config.I2C_ACKControl == I2C_ACKControl_EN){
		pI2CHandle -> pI2Cx -> CR1 |= (1 << 10);
	}

	//3. Assign slave address
	pI2CHandle -> pI2Cx -> OAR1 |= (1 << 14);
	if(pI2CHandle -> I2C_Config.I2C_AddressMode == I2C_AddressMode_7bits){
		pI2CHandle -> pI2Cx -> OAR1 |= (pI2CHandle -> I2C_Config.I2C_SlaveAddress & 0xFE);
	}else if(pI2CHandle -> I2C_Config.I2C_AddressMode == I2C_AddressMode_10bits){
		pI2CHandle -> pI2Cx -> OAR1 |= (pI2CHandle -> I2C_Config.I2C_SlaveAddress & 0x3FF);
	}

	//4. COnfigure Trise
	if(pI2CHandle -> I2C_Config.I2C_SCLSpeed == I2C_SCLSpeed_SM){
		pI2CHandle -> pI2Cx -> TRISE |= 0x11;
	}else if(pI2CHandle -> I2C_Config.I2C_SCLSpeed == I2C_SCLSpeed_FM4K){
		pI2CHandle -> pI2Cx -> TRISE |= 0x06;
	}
}
void I2C_DeInit(I2C_Regdef_t *pI2Cx){
	if(pI2Cx == I2C1){
		RCC -> APB1RSTR |= (1 << 21);
	}if(pI2Cx == I2C2){
		RCC -> APB1RSTR |= (1 << 22);
	}if(pI2Cx == I2C3){
		RCC -> APB1RSTR |= (1 << 23);
	}
}

void I2C_PeriEnable(I2C_Regdef_t *pI2Cx, uint8_t state){
	if(state == ENABLE){
		pI2Cx -> CR1 |= (1 << 0);
	}else if(state == DISABLE){
		pI2Cx -> CR1 &= ~(1 << 0);
	}
}
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *TxBuff, uint32_t len, uint8_t SlaveAddress){
	uint32_t sr1_temp, sr2_temp;
	//0. Generate Start Condition
	pI2CHandle -> pI2Cx -> CR1 |= (1 << 8);
	//1. Wait until start successfully
	while(!(pI2CHandle -> pI2Cx -> SR1 & 0x01));
	//2.Clear SB bit by read SR1 and write Slave address to DR
	sr1_temp = pI2CHandle -> pI2Cx -> SR1;
	pI2CHandle -> pI2Cx -> DR = SlaveAddress << 1;
	//3. Check Transmit
	if(pI2CHandle -> I2C_Config.I2C_MasterRW == I2C_WRITE){
		pI2CHandle -> pI2Cx -> DR &= ~(1 << 0);
		//4. Wait until ADDR = 1 (match slave address)
		while(!(pI2CHandle -> pI2Cx -> SR1 & 0x02));
		sr1_temp = pI2CHandle -> pI2Cx -> SR1;
		sr2_temp = pI2CHandle -> pI2Cx -> SR2;
		//5. Wait until TxE = 1
		while(len > 0){
			while(!(pI2CHandle -> pI2Cx -> SR1 & (1 << 7)));
			pI2CHandle -> pI2Cx -> DR = *TxBuff;
			TxBuff++;
			len--;
		}
		//6. Wait until TxE = 1 && BTF = 1 before generating Stop Condition
		while(!((pI2CHandle -> pI2Cx ->SR1 & (1 <<7)) && (pI2CHandle -> pI2Cx -> CR1 & (1 << 2))));
		//7. Generate Stop Condition
		pI2CHandle -> pI2Cx -> CR1 |= (1 << 9);
	}
}
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *RxBuff, uint32_t len, uint8_t SlaveAddress){
	uint16_t sr1_temp, sr2_temp;

	//1. Generate Start Condition
	pI2CHandle -> pI2Cx -> CR1 |= (1 << 8);
	//2. Wait until Start Successfully
	while(!(pI2CHandle -> pI2Cx -> SR1 & 0x01));
	//3. Clear SB bit
	sr1_temp = pI2CHandle -> pI2Cx -> SR1;
	SlaveAddress = (SlaveAddress << 1) ;
	SlaveAddress |= 1;
	pI2CHandle -> pI2Cx -> DR |= SlaveAddress;
	//4. Wait until ADDR = 1
	while(!(pI2CHandle -> pI2Cx -> SR1 & (1 << 1)));
	if(len == 1){
		//5. Clear ACK
		pI2CHandle -> pI2Cx -> CR1 &= ~(1 << 10);

		//6. Clear ADDR bit
		sr1_temp = pI2CHandle -> pI2Cx -> SR1;
		sr2_temp = pI2CHandle -> pI2Cx -> SR2;
		//7. Wait Until RxNE = 1
		while(!(pI2CHandle -> pI2Cx -> SR1 & (1 << 6)));
		//8. Set Stop Condition
		pI2CHandle -> pI2Cx -> CR1 |= (1 << 9);
		//9. Read Data from DR
		*RxBuff = pI2CHandle -> pI2Cx -> DR & 0xFF;
	}else if(len > 1){
		for(uint32_t i = len; i > 0; i--){
			if(i == 2){
				// Clear ACK
				pI2CHandle -> pI2Cx -> CR1 &= ~(1 << 10);
				// Clear ADDR Flag
				sr1_temp = pI2CHandle -> pI2Cx -> SR1;
				sr2_temp = pI2CHandle -> pI2Cx -> SR2;
				// Wait Until BTF is set
				while(!(pI2CHandle -> pI2Cx -> SR1 & (1 << 2)));
				// Set Stop Condition
				pI2CHandle -> pI2Cx -> CR1 |= (1 << 9);
				// Read Data 1
				*RxBuff = pI2CHandle -> pI2Cx -> DR & 0xFF;
				RxBuff++;
				// Read Data 2
				while(!(pI2CHandle -> pI2Cx -> SR1 & (1 << 6)));
				*RxBuff = pI2CHandle -> pI2Cx -> DR & 0xFF;
			}else if(i > 2){
				// Clear ADDR Flag
				sr1_temp = pI2CHandle -> pI2Cx -> SR1;
				sr2_temp = pI2CHandle -> pI2Cx -> SR2;
				// Wait Until RxNE is set
				while(!(pI2CHandle -> pI2Cx -> SR1 & (1 << 6)));
				*RxBuff = pI2CHandle -> pI2Cx -> DR & 0xFF;
				RxBuff++;
			}
		}
	}

	if(pI2CHandle -> I2C_Config.I2C_ACKControl == I2C_ACKControl_EN){
		pI2CHandle -> pI2Cx -> CR1 |= 1 << 10;
	}
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *TxBuff, uint32_t len, uint8_t SlaveAddress, uint8_t Sr){
	uint8_t busystate = pI2CHandle -> TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle -> pTxBuff = TxBuff;
		pI2CHandle -> TxLen = len;
		pI2CHandle -> TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle -> DevAddress = SlaveAddress;
		pI2CHandle -> SRepeated = Sr;

		//Implement code to Generate START Condition
		pI2CHandle -> pI2Cx -> CR1 |= (1 << 8);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle -> pI2Cx -> CR2 |= ( 1 << 10);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle -> pI2Cx -> CR2 |= (1 << 9);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle -> pI2Cx -> CR2 |= (1 << 8);

	}

	return pI2CHandle -> TxRxState;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *RxBuff, uint32_t len, uint8_t SlaveAddress, uint8_t Sr){
	uint8_t busystate = pI2CHandle -> TxRxState;
	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle -> pRxBuff = RxBuff;
		pI2CHandle -> RxLen = len;
		pI2CHandle -> TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle -> Rxsize = len;
		pI2CHandle -> DevAddress = SlaveAddress;
		pI2CHandle -> SRepeated = Sr;

		//Implement code to Generate START Condition
		pI2CHandle -> pI2Cx -> CR1 |= (1 << 8);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle -> pI2Cx -> CR2 |= ( 1 << 10);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle -> pI2Cx -> CR2 |= (1 << 9);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle -> pI2Cx -> CR2 |= (1 << 8);
	}

	return pI2CHandle -> TxRxState;
}

void I2C_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t state){
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

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
	uint8_t temp1, temp2, temp3;
	uint32_t sr1_temp, sr2_temp, dr_temp;
	uint8_t DevAdd;
	//ITBUFEN, ITEVTEN
	temp1 = pI2CHandle -> pI2Cx -> CR2 & (1 << 10);
	temp2 = pI2CHandle -> pI2Cx -> CR2 & (1 << 9);

	//1. SB Interrupt
	temp3 = pI2CHandle -> pI2Cx -> SR1 & 0x01;
	if(temp2 && temp3){
		sr1_temp = pI2CHandle -> pI2Cx -> SR1;
		if(pI2CHandle -> TxRxState == I2C_BUSY_IN_TX){
			pI2CHandle -> pI2Cx -> DR = pI2CHandle -> DevAddress << 1;
		}else if(pI2CHandle -> TxRxState == I2C_BUSY_IN_RX){
			DevAdd = pI2CHandle -> DevAddress << 1;
			DevAdd |= 1;
			pI2CHandle -> pI2Cx -> DR = DevAdd;
		}
	}
	//2. ADDR Interrupt
	temp3 = pI2CHandle -> pI2Cx -> SR1 & (1 << 1);
	if(temp2 && temp3){
		if(pI2CHandle -> pI2Cx -> SR2 & (1 << 0)){
			if(pI2CHandle -> TxRxState == I2C_BUSY_IN_RX){
				if(pI2CHandle -> Rxsize == 1){
					//Disable the ACK
					pI2CHandle -> pI2Cx -> CR1 &= ~(1 << 10);
				}

			}//Clear ADDR Flag
			sr1_temp = pI2CHandle -> pI2Cx -> SR1;
			sr2_temp = pI2CHandle -> pI2Cx -> SR2;
		}else{
			sr1_temp = pI2CHandle -> pI2Cx -> SR1;
			sr2_temp = pI2CHandle -> pI2Cx -> SR2;
		}
	}
	//3. BTF Interrupt
	temp3 = pI2CHandle -> pI2Cx -> SR1 & (1 << 2);
	if(temp2 && temp3){
		if(pI2CHandle -> TxRxState == I2C_BUSY_IN_TX){
			if(pI2CHandle -> TxLen == 0){
				if(pI2CHandle -> pI2Cx -> SR1 & (1 << 7)){
					if(pI2CHandle -> SRepeated == DISABLE){
						pI2CHandle -> pI2Cx -> CR1 |= (1 << 9);
					}
					//Reset all of the elements of Handle Structure
					I2C_CloseSendData(pI2CHandle);
					//Notify Application about Transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPT);
				}
			}
		}else if(pI2CHandle -> TxRxState == I2C_BUSY_IN_RX){
			;
		}
	}
	//4. STOPF Event
	temp3 = pI2CHandle -> pI2Cx -> SR1 & (1 << 4);
	if(temp2 && temp3){
		//Clear STOPF by reading SR1 and Writing to CR1
		pI2CHandle -> pI2Cx -> CR1 |= 0;
		//Notify the Application that Stop Detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}
	//5. TxE Event
	temp3 = pI2CHandle -> pI2Cx -> SR1 & (1 << 7);
	if(temp1 && temp2 && temp3){
		//Check for device mode
		//Master Mode
		if((pI2CHandle -> pI2Cx -> SR2 & (1 << 0))){
			if(pI2CHandle -> TxLen > 0){
				pI2CHandle -> pI2Cx -> DR = *(pI2CHandle -> pTxBuff);
				pI2CHandle -> pTxBuff++;
				pI2CHandle -> TxLen--;
			}
		}else if(!(pI2CHandle -> pI2Cx -> SR2 & (1 << 0))){
			//Slave Mode

		}
	}

	//6. RxNE Event
	temp3 = pI2CHandle -> pI2Cx -> SR1 & (1 << 6);
	if(temp1 && temp2 && temp3){
		if(pI2CHandle -> TxRxState == I2C_BUSY_IN_RX){
			if(pI2CHandle -> Rxsize == 1){
				(*pI2CHandle -> pRxBuff) = pI2CHandle -> pI2Cx -> DR & 0xFF;
				pI2CHandle -> pRxBuff++;
				pI2CHandle -> RxLen--;
			}else if(pI2CHandle -> Rxsize > 1){
				if(pI2CHandle -> Rxsize == 2){
					pI2CHandle -> pI2Cx -> CR1 &= ~(1 << 10);
				}
				*pI2CHandle -> pRxBuff = pI2CHandle -> pI2Cx -> DR & 0xFF;
				pI2CHandle -> pRxBuff++;
				pI2CHandle -> RxLen--;
			}else if(pI2CHandle -> Rxsize == 0){

				//1. Generate Stop Condition
				pI2CHandle -> pI2Cx -> CR1 |= (1 << 9);
				//2. Close I2C Reception
				I2C_CloseReceiveData(pI2CHandle);
				//3.Notify the Application
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPT);
			}
		}
	}
}
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle){
	pI2CHandle -> pI2Cx -> CR2 &= ~(1 << 10);
	pI2CHandle -> pI2Cx -> CR2 &= ~(1 << 9);

	pI2CHandle -> TxLen = 0;
	pI2CHandle -> pTxBuff = NULL;
	pI2CHandle -> TxRxState = I2C_READY;
}
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle){
	pI2CHandle -> pI2Cx -> CR2 &= ~(1 << 10);
	pI2CHandle -> pI2Cx -> CR2 &= ~(1 << 9);

	pI2CHandle -> RxLen = 0;
	pI2CHandle -> Rxsize = 0;
	pI2CHandle -> pRxBuff = NULL;
	pI2CHandle -> TxRxState = I2C_READY;
	pI2CHandle -> pI2Cx -> CR1 |= (1 << 10);
}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){
	uint8_t temp1 = pI2CHandle -> pI2Cx -> CR2 & (1 << 8);
	uint8_t temp2;
	//1.BERR
	temp2 = pI2CHandle -> pI2Cx -> SR1 & (1 << 8);
	if(temp1 && temp2){
		//Clear BERR by reset BERR
		pI2CHandle -> pI2Cx -> SR1 &= ~(1 << 8);
		//notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_BERR);
	}
	//2.ARLO
	temp2 = pI2CHandle -> pI2Cx -> SR1 & (1 << 9);
	if(temp1 && temp2){
		pI2CHandle -> pI2Cx -> SR1 &=  ~(1 << 9);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_ARLO);
	}
	//3.AF
	temp2 = pI2CHandle -> pI2Cx -> SR1 & (1 << 10);
	if(temp1 && temp2){
		pI2CHandle -> pI2Cx -> SR1 &=  ~(1 << 10);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_AF);
	}
	//4.OVR (Overrun/Underrun)
	temp2 = pI2CHandle -> pI2Cx -> SR1 & (1 << 11);
	if(temp1 && temp2){
		pI2CHandle -> pI2Cx -> SR1 &=  ~(1 << 11);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_OVR);
	}
	//5.Timeout
	temp2 = pI2CHandle -> pI2Cx -> SR1 & (1 << 14);
	if(pI2CHandle -> pI2Cx -> CR1 & (1 << 1)){
		if(temp1 && temp2){
			pI2CHandle -> pI2Cx -> SR1 &=  ~(1 << 14);
			I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_TIMEOUT);
		}
	}
}
void I2C_SlaveSendData(I2C_Regdef_t *pI2Cx, uint8_t data){
	pI2Cx -> DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_Regdef_t *pI2Cx){
	return (uint8_t)pI2Cx -> DR;
}

