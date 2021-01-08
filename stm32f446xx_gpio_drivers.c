/*
 * stm32f446xx_gpio_drivers.c
 *
 *  Created on: June 20th, 2020
 *      Author: VIET TOAN
 */

#include "stm32f446xx_gpio_drivers.h"

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t state){
	if(state == ENABLE){
		if(pGPIOx == GPIOA)			{GPIOA_PCLK_EN();}
		else if(pGPIOx == GPIOB)	{GPIOB_PCLK_EN();}
		else if(pGPIOx == GPIOC)	{GPIOC_PCLK_EN();}
		else if(pGPIOx == GPIOD)	{GPIOD_PCLK_EN();}
		else if(pGPIOx == GPIOE)	{GPIOE_PCLK_EN();}
		else if(pGPIOx == GPIOF)	{GPIOF_PCLK_EN();}
		else if(pGPIOx == GPIOG)	{GPIOG_PCLK_EN();}
		else if(pGPIOx == GPIOH)	{GPIOH_PCLK_EN();}
	}else{
		if(pGPIOx == GPIOA)			{GPIOA_PCLK_DIS();}
		else if(pGPIOx == GPIOB)	{GPIOB_PCLK_DIS();}
		else if(pGPIOx == GPIOC)	{GPIOC_PCLK_DIS();}
		else if(pGPIOx == GPIOD)	{GPIOD_PCLK_DIS();}
		else if(pGPIOx == GPIOE)	{GPIOE_PCLK_DIS();}
		else if(pGPIOx == GPIOF)	{GPIOF_PCLK_DIS();}
		else if(pGPIOx == GPIOG)	{GPIOG_PCLK_DIS();}
		else if(pGPIOx == GPIOH)	{GPIOH_PCLK_DIS();}
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0;
	//1. Configure Pin Mode
	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}else{
		//1. Configure the mask bits of the 23 interrupt lines (EXTI_IMR)
		EXTI -> EXTI_IMR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		//2. Configure Interrupt mode
		if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_IT_FT){
			EXTI -> EXTI_FTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			EXTI -> EXTI_RTSR &= ~(1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_IT_RT){
			EXTI -> EXTI_RTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			EXTI -> EXTI_FTSR &= ~(1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_IT_RF){
			EXTI -> EXTI_FTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			EXTI -> EXTI_RTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}
		//3. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1, temp2, portValue;
		GPIO_RegDef_t *pGPIOx = pGPIOHandle -> pGPIOx;
		temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber / 4;
		temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber % 4;
		SYSCFG_PCLK_EN();
		if(pGPIOx == GPIOA){
			portValue = 0;
		}else if(pGPIOx == GPIOB){
			portValue = 1;
		}else if(pGPIOx == GPIOC){
			portValue = 2;
		}else if(pGPIOx == GPIOD){
			portValue = 3;
		}else if(pGPIOx == GPIOE){
			portValue = 4;
		}else if(pGPIOx == GPIOF){
			portValue = 5;
		}else if(pGPIOx == GPIOG){
			portValue = 6;
		}else if(pGPIOx == GPIOH){
			portValue = 7;
		}
		SYSCFG -> EXTICR[temp1] |= portValue << (temp2 * 4);

	}
	temp = 0;

	//2. Configure Pin Speed
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;
	temp = 0;

	//3. Configure PUPD
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PUPD << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//4. Configure Output Type
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_OPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//5. Configure Alternate Function
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ATF){
		uint32_t temp1 = 0;
		uint32_t temp2 = 0;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp] &= ~(0x0F << 4 * temp2);
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_AF << (4 * temp2);
		temp1 = temp2 = 0;
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_RESET();
	}else if(pGPIOx == GPIOB){
		GPIOB_RESET();
	}else if(pGPIOx == GPIOC){
		GPIOC_RESET();
	}else if(pGPIOx == GPIOD){
		GPIOD_RESET();
	}else if(pGPIOx == GPIOE){
		GPIOE_RESET();
	}else if(pGPIOx == GPIOF){
		GPIOF_RESET();
	}else if(pGPIOx == GPIOG){
		GPIOG_RESET();
	}else if(pGPIOx == GPIOH){
		GPIOH_RESET();
	}
}

uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t pinState = 0;
	pinState = (uint8_t)(((pGPIOx -> IDR) >> PinNumber) & 0x00000001);
	return pinState;
}
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t state){
	if(state == GPIO_PIN_SET){
		(pGPIOx -> ODR) |= (1 << PinNumber);
	}else if(state == GPIO_PIN_RESET){
		(pGPIOx -> ODR) &= ~(1 << PinNumber);
	}
}
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	/* Solution 1
	uint8_t pinState;
	pinState = (uint8_t)(((pGPIOx -> IDR) >> PinNumber) & 0x00000001);
	if(pinState == GPIO_PIN_SET){
		(pGPIOx -> ODR) |= (1 << PinNumber);
	}else if(pinState == GPIO_PIN_RESET){
		(pGPIOx -> ODR) &= ~(1 << PinNumber);
	}*/
	/*Solution 2*/
	pGPIOx -> ODR ^= (1 << PinNumber); // Using XOR

}
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t state){
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
void GPIO_IRQHandling(uint32_t GPIO_Number){
	if(EXTI -> EXTI_PR &= (1 << GPIO_Number)){
		//Clear Pending Register by writing 1 to the corresponding GPIO Number
		EXTI -> EXTI_PR |= 1 << GPIO_Number;
	}
}

