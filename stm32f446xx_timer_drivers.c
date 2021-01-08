/*
 * stm32f446xx_timer_drivers.c
 *
 *  Created on: 5 thg 9, 2020
 *      Author: toan
 */

#include "stm32f446xx_timer_drivers.h"

void TIM_PeriClockControl(TIM_Basic_Regdef_t *pTIMx, uint8_t state){
	if(state == ENABLE){
		if(pTIMx == TIM6){
			TIM6_PCLK_EN();
		}else if(pTIMx == TIM7){
			TIM7_PCLK_EN();
		}
	}else if(state == DISABLE){
		if(pTIMx == TIM6){
			TIM6_PCLK_DIS();
		}else if(pTIMx == TIM7){
			TIM7_PCLK_DIS();
		}
	}
}

void TIM_BASIC_INIT(TIM_Basic_Handle_t *TIM_Basic_Handle){
	//1. Configure Prescaler value
	TIM_Basic_Handle -> pTIM_Bx -> PSC = TIM_Basic_Handle -> TIM_Basic_Config.PSC;
	//2. Configure Period
	TIM_Basic_Handle -> pTIM_Bx -> CNT = TIM_Basic_Handle -> TIM_Basic_Config.Period;
	//3. Trigger Event Selection

}
