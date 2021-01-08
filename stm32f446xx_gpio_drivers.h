/*
 * stm32f446xx_gpio_drivers.h
 *
 *  Created on: 20 thg 6, 2020
 *      Author: toan
 */

#ifndef INC_STM32F446XX_GPIO_DRIVERS_H_
#define INC_STM32F446XX_GPIO_DRIVERS_H_

#include "stm32f446xx.h"
#include "stdbool.h"

/*
 * GPIO PIN NUMBER
 */
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15
/*
 * PIN MODE
 */
#define GPIO_MODE_INPUT		0
#define GPIO_MODE_OUTPUT	1
#define GPIO_MODE_ATF		2
#define GPIO_MODE_ANALOG	3
#define GPIO_IT_RT			4
#define GPIO_IT_FT			5
#define GPIO_IT_RF			6

/*
 * PIN SPEED
 */
#define GPIO_SP_LOW			0
#define GPIO_SP_MEDIUM		1
#define GPIO_SP_FAST		2
#define GPIO_SP_HIGH		3

/*
 * PULL UP AND PULL DOWN
 */
#define GPIO_PUPD_NO		0
#define GPIO_PU				1
#define GPIO_PD				2

/*
 * GPIO OUTPUT TYPE
 */
#define GPIO_PP				0
#define GPIO_OD				1

typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t	GPIO_PinMode;
	uint8_t	GPIO_PinSpeed;
	uint8_t	GPIO_PUPD;
	uint8_t	GPIO_OPType;
	uint8_t GPIO_AF;
}GPIO_Config_t;

typedef struct{
	GPIO_RegDef_t *pGPIOx;
	GPIO_Config_t GPIO_PinConfig;
}GPIO_Handle_t;


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t state);
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t state);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t state);
void GPIO_IRQHandling(uint32_t GPIO_Number);


#endif /* INC_STM32F446XX_GPIO_DRIVERS_H_ */
