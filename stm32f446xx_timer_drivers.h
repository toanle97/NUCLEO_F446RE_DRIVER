/*
 * stm32f446xx_timer_drivers.h
 *
 *  Created on: 5 thg 9, 2020
 *      Author: toan
 */

#ifndef INC_STM32F446XX_TIMER_DRIVERS_H_
#define INC_STM32F446XX_TIMER_DRIVERS_H_

#include "stm32f446xx.h"
#include "stddef.h"

typedef struct{
	uint16_t PSC;
	uint16_t Period;
	uint8_t TriggerEV;
}TIM_Basic_Config_t;

typedef struct{
	TIM_Basic_Config_t TIM_Basic_Config;
	TIM_Basic_Regdef_t *pTIM_Bx;
}TIM_Basic_Handle_t;

void TIM_PeriClockControl(TIM_Basic_Regdef_t *pTIMx, uint8_t state);
void TIM_BASIC_INIT(TIM_Basic_Handle_t *TIM_Basic_Handle);
void TIM_BASIC_DEINIT(TIM_Basic_Handle_t *TIM_Basic_Handle);


#endif /* INC_STM32F446XX_TIMER_DRIVERS_H_ */
