/*
 * stm32f446xx.h
 *
 *  Created on: May 25, 2020
 *      Author: VIET TOAN
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>
#define __vo volatile

#define NVIC_ISER_BASEADDR			0xE000E100U
#define NVIC_ICER_BASEADDR			0XE000E180U
#define NVIC_ISPR_BASEADDR			0XE000E200U
#define NVIC_ICPR_BASEADDR			0XE000E280U
#define NVIC_IABR_BASEADDR			0xE000E300U
#define NVIC_IPR_BASEADDR			0xE000E400U
#define STIR_BASEADDR				0xE000EF00U

#define ENABLE 						1
#define DISABLE						0
#define SET							1
#define RESET 						0
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET

#define FLASH_BASEADDR 				0x08000000U
#define SRAM1_BASEADDR 				0x20000000U
#define SRAM2_BASEADDR				0x2001C000U
#define ROM_BASEADDR				0x1FFF0000U

#define PERIPH_BASEADDR				0x40000000U
#define APB1_BASEADDR				PERIPH_BASEADDR
#define AHB1_BASEADDR 				0x40020000U
#define APB2_BASEADDR				0x40010000U
#define AHB2_BASEADDR				0x50000000U
#define AHB3_BASEADDR				0x60000000U

#define RCC_BASEADDR				0x40023800U
#define EXTI_BASEADDR				0x40013C00U

#define GPIOA_BASEADDR				0x40020000U
#define GPIOB_BASEADDR				0x40020400U
#define GPIOC_BASEADDR				0x40020800U
#define GPIOD_BASEADDR				0x40020C00U
#define GPIOE_BASEADDR				0x40021000U
#define GPIOF_BASEADDR				0x40021400U
#define GPIOG_BASEADDR				0x40021800U
#define GPIOH_BASEADDR				0x40021C00U

//General Purpose Timers
#define TIM2_BASEADDR				0x40000000U
#define TIM3_BASEADDR				0x40000400U
#define TIM4_BASEADDR				0x40000800U
#define TIM5_BASEADDR				0x40000C00U
#define TIM9_BASEADDR				0x40014000U
#define TIM10_BASEADDR				0x40014400U
#define TIM11_BASEADDR				0x40014800U
#define TIM12_BASEADDR				0x40001800U
#define TIM13_BASEADDR				0x40001C00U
#define TIM14_BASEADDR				0x40002000U
//Basic Timers
#define TIM6_BASEADDR				0x40001000U
#define TIM7_BASEADDR				0x40001400U
//Advanced-Control Timers
#define TIM1_BASEADDR				0x40010000U
#define TIM8_BASEADDR				0x40010400U

#define SPI1_BASEADDR				0x40013000U
#define SPI2_BASEADDR 				0x40003800U
#define SPI3_BASEADDR				0x40003C00U
#define SPI4_BASEADDR				0x40013400U

#define USART1_BASEADDR				0x40011000U
#define USART2_BASEADDR				0x40004400U
#define USART3_BASEADDR				0x40004800U
#define UART4_BASEADDR				0x40004C00U
#define UART5_BASEADDR				0x40005000U
#define USART6_BASEADDR				0x40011400U

#define I2C1_BASEADDR				0x40005400U
#define I2C2_BASEADDR				0x40005800U
#define I2C3_BASEADDR				0x40005C00U

#define EXTI_BASEADDR				0x40013C00U
#define SYSCFG_BASEADDR				0x40013800U

#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_EXTI15_10			40

#define IRQ_NO_SPI1					35
#define IRQ_NO_SPI2					36
#define IRQ_NO_SPI3					51
#define IRQ_NO_SPI4					84

#define IRQ_NO_USART1 				37
#define IRQ_NO_USART2 				38
#define IRQ_NO_USART3 				39
#define IRQ_NO_UART4 				52
#define IRQ_NO_UART5 				53
#define IRQ_NO_USART6				71

#define IRQ_NO_I2C1_EV				31
#define IRQ_NO_I2C1_ER				32
#define IRQ_NO_I2C2_EV				33
#define IRQ_NO_I2C2_ER				34
#define IRQ_NO_I2C3_EV				72
#define IRQ_NO_I2C3_ER				73

typedef struct{
	__vo uint32_t ISER[8];
}NVIC_ISER_Reg_t;

typedef struct{
	__vo uint32_t ICER[8];
}NVIC_ICER_Reg_t;

typedef struct{
	__vo uint32_t IPR[60];
}NVIC_IPR_Reg_t;

#define NVIC_ISER					((NVIC_ISER_Reg_t *)NVIC_ISER_BASEADDR)
#define NVIC_ICER					((NVIC_ICER_Reg_t *)NVIC_ICER_BASEADDR)
#define NVIC_IPR					((NVIC_IPR_Reg_t *)NVIC_IPR_BASEADDR)


typedef struct
{
	__vo uint32_t MODER;			/*port mode register 											Offset: 0x00*/
	__vo uint32_t OTYPER;			/*port output type register 									Offset: 0x04*/
	__vo uint32_t OSPEEDER;			/*port output speed register 									Offset: 0x08*/
	__vo uint32_t PUPDR;			/*port pull-up/pull-down register 								Offset: 0x0C*/
	__vo uint32_t IDR;				/*port input data register 										Offset: 0x10*/
	__vo uint32_t ODR;				/*port output data register		 								Offset: 0x14*/
	__vo uint32_t BSRR;				/*port bit set/reset register									Offset: 0x18*/
	__vo uint32_t LCKR;				/*port configuration lock register 								Offset: 0x1C*/
	__vo uint32_t AFR[2];			/*alternate function register									Offset: 0x20 - 0x24*/
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;				/*RCC clock control register									Offset: 0x00*/
	__vo uint32_t PLLCFGR;			/*RCC PLL configuration register 								Offset: 0x04*/
	__vo uint32_t CFGR;				/*RCC clock configuration register								Offset: 0x08*/
	__vo uint32_t CIR;				/*RCC clock interrupt register									Offset: 0x0C*/
	__vo uint32_t AHB1RSTR;			/*RCC AHB1 peripheral reset register							Offset: 0x10*/
	__vo uint32_t AHB2RSTR;			/*RCC AHB2 peripheral reset register							Offset: 0x14*/
	__vo uint32_t AHB3RSTR;			/*RCC AHB3 peripheral reset register							Offset: 0x18*/
	uint32_t RESERVE0;
	__vo uint32_t APB1RSTR;			/*RCC APB1 peripheral reset register 							Offset: 0x20*/
	__vo uint32_t APB2RSTR;			/*RCC APB2 peripheral reset register							Offset: 0x24*/
	uint32_t RESERVE1[2];
	__vo uint32_t AHB1ENR;			/*RCC AHB1 peripheral CER										Offset: 0x30*/
	__vo uint32_t AHB2ENR;			/*RCC AHB2 peripheral CER										Offset: 0x34*/
	__vo uint32_t AHB3ENR;			/*RCC AHB3 peripheral CER										Offset: 0x38*/
	uint32_t RESERVE2;
	__vo uint32_t APB1ENR;			/*RCC APB1 peripheral CER										Offset: 0x40*/
	__vo uint32_t APB2ENR;			/*RCC APB2 peripheral CER										Offset: 0x44*/
	uint32_t RESERVE3[2];
	__vo uint32_t AHB1LPENR;		/*RCC AHB1 peripheral clock enable in low power mode register	Offset: 0x50*/
	__vo uint32_t AHB2LPENR;		/*RCC AHB2 peripheral clock enable in low power mode register	Offset: 0x54*/
	__vo uint32_t AHB3LPENR;		/*RCC AHB3 peripheral clock enable in low power mode register	Offset: 0x58*/
	uint32_t RESERVE4;
	__vo uint32_t APB1LPENR;		/*RCC APB1 peripheral clock enable in low power mode register	Offset: 0x60*/
	__vo uint32_t APB2LPENR;		/*RCC APB2 peripheral clock enabled in low power mode register	Offset: 0x64*/
	uint32_t RESERVE5[2];
	__vo uint32_t BDCR;				/*RCC Backup domain control register							Offset: 0x70*/
	__vo uint32_t CSR;				/*RCC clock control & status register							Offset: 0x74*/
	uint32_t RESERVE6[2];
	__vo uint32_t SSCGR;			/*RCC spread spectrum clock generation register 				Offset: 0x80*/
	__vo uint32_t PLLI2SCFGR;		/*CC PLLI2S configuration register								Offset: 0x84*/
	__vo uint32_t PLLSAICFGR;		/*RCC PLL configuration register								Offset: 0x88*/
	__vo uint32_t DCKCFGR;			/*RCC Dedicated Clock Configuration Register					Offset: 0x8C*/
	__vo uint32_t CKGATENR;			/*RCC clocks gated enable register								Offset: 0x90*/
	__vo uint32_t DCKCFGR2;			/*RCC dedicated clocks configuration register 2					Offset: 0x94*/

}RCC_RegDef_t;

typedef struct{
	__vo uint32_t EXTI_IMR;			/*Interrupt mask register										Offset: 0x00*/
	__vo uint32_t EXTI_EMR;			/*Event mask register											Offset: 0x04*/
	__vo uint32_t EXTI_RTSR;		/*Rising trigger selection register								Offset: 0x08*/
	__vo uint32_t EXTI_FTSR;		/*Falling trigger selection register							Offset: 0x0C*/
	__vo uint32_t EXTI_SWIER;		/*Software interrupt event register								Offset: 0X10*/
	__vo uint32_t EXTI_PR;			/*Pending register 												Offset: 0x14*/
}EXTI_RegDef_t;

typedef struct{
	__vo uint32_t MEMRMP;			/*SYSCFG memory remap register									Offset: 0x00*/
	__vo uint32_t PMC;				/*SYSCFG peripheral mode configuration register					Offset: 0x04*/
	__vo uint32_t EXTICR[4];		/*SYSCFG external interrupt configuration register				Offset: 0x08 - 0x14*/
	uint32_t RESERVERSYS1[2];
	__vo uint32_t CMPCR;			/*Compensation cell control register							Offset: 0x20*/
	uint32_t RESERVERSYS2[2];
	__vo uint32_t CFGR;				/*SYSCFG configuration register									Offset: 0x2C*/
}SYSCFG_RegDef_t;

typedef struct{
	__vo uint32_t CR1;				/*SPI control register 1										Offset: 0x00*/
	__vo uint32_t CR2;				/*SPI control register 2										Offset: 0x04*/
	__vo uint32_t SR;				/*SPI status register											Offset: 0x08*/
	__vo uint32_t DR;				/*SPI data register												Offset: 0x0C*/
	__vo uint32_t CRCPR;			/*SPI CRC polynomial register									Offset: 0x10*/
	__vo uint32_t RXCRCR;			/*SPI RX CRC register											Offset: 0x14*/
	__vo uint32_t TXCRCR;			/*SPI TX CRC register											Offset: 0x18*/
}SPI_RegDef_t;

typedef struct {
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
}USART_Regdef_t;

typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLRT;
}I2C_Regdef_t;

typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t RESERVED1;
	__vo uint32_t DIER;
	__vo uint32_t SR;
	__vo uint32_t ERG;
	__vo uint32_t RESERVED2[3];
	__vo uint32_t CNT;
	__vo uint32_t PSC;
	__vo uint32_t ARR;
}TIM_Basic_Regdef_t;



#define GPIOA 	((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB 	((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC 	((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD 	((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE 	((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF 	((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG 	((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH 	((GPIO_RegDef_t *)GPIOH_BASEADDR)

#define SPI1	((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2	((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3	((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4 	((SPI_RegDef_t *)SPI4_BASEADDR)

#define USART1	((USART_Regdef_t *)USART1_BASEADDR)
#define USART2	((USART_Regdef_t *)USART2_BASEADDR)
#define USART3	((USART_Regdef_t *)USART3_BASEADDR)
#define USART6	((USART_Regdef_t *)USART6_BASEADDR)
#define UART4	((USART_Regdef_t *)UART4_BASEADDR)
#define UART5	((USART_Regdef_t *)UART5_BASEADDR)

#define I2C1 	((I2C_Regdef_t *) I2C1_BASEADDR)
#define I2C2 	((I2C_Regdef_t *) I2C2_BASEADDR)
#define I2C3	((I2C_Regdef_t *) I2C3_BASEADDR)

#define TIM6	((TIM_Basic_Regdef_t *)TIM6_BASEADDR)
#define TIM7	((TIM_Basic_Regdef_t *)TIM7_BASEADDR)



#define RCC 	((RCC_RegDef_t *)RCC_BASEADDR)
#define EXTI	((EXTI_RegDef_t *)EXTI_BASEADDR)
#define SYSCFG	((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

/*
 * Clock Enable Macros for GPIOx Peripherals
 */
#define GPIOA_PCLK_EN() 		(RCC -> AHB1ENR |= (1 << 0));
#define GPIOB_PCLK_EN() 		(RCC -> AHB1ENR |= (1 << 1));
#define GPIOC_PCLK_EN() 		(RCC -> AHB1ENR |= (1 << 2));
#define GPIOD_PCLK_EN() 		(RCC -> AHB1ENR |= (1 << 3));
#define GPIOE_PCLK_EN() 		(RCC -> AHB1ENR |= (1 << 4));
#define GPIOF_PCLK_EN() 		(RCC -> AHB1ENR |= (1 << 5));
#define GPIOG_PCLK_EN() 		(RCC -> AHB1ENR |= (1 << 6));
#define GPIOH_PCLK_EN() 		(RCC -> AHB1ENR |= (1 << 7));

/*
 * Clock Enable Macros for I2Cx
 */

#define I2C1_PCLK_EN()			(RCC -> APB1ENR |= (1 << 21));
#define I2C2_PCLK_EN()			(RCC -> APB1ENR |= (1 << 22));
#define I2C3_PCLK_EN()			(RCC -> APB1ENR |= (1 << 23));

/*
 * Clock Enable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_EN()			(RCC -> APB2ENR |= (1 << 12));
#define SPI2_PCLK_EN()			(RCC -> APB1ENR |= (1 << 14));
#define SPI3_PCLK_EN()			(RCC -> APB1ENR |= (1 << 15));
#define SPI4_PCLK_EN()			(RCC -> APB2ENR |= (1 << 13));

/*
 * Clock Enable Macros for USARTx/UARTx Peripherals
 */
#define USART2_PCLK_EN()		(RCC -> APB1ENR |= (1 << 17));
#define USART3_PCLK_EN()		(RCC -> APB1ENR |= (1 << 18));
#define UART4_PCLK_EN()			(RCC -> APB1ENR |= (1 << 19));
#define UART5_PCLK_EN()			(RCC -> APB1ENR |= (1 << 20));
#define USART1_PCLK_EN()		(RCC -> APB2ENR |= (1 << 4));
#define USART6_PCLK_EN()		(RCC -> APB2ENR |= (1 << 5));

/*
 * Clock Enable Macros for Basic Timer Peripherals
 */
#define TIM6_PCLK_EN()			(RCC -> APB1ENR |= 1 << 4);
#define TIM7_PCLK_EN()			(RCC -> APB1ENR |= 1 << 5);

/*
 * Clock Enable Macros for System Configuration
 */
#define SYSCFG_PCLK_EN()		(RCC -> APB2ENR |= (1 << 14));

/*
 * Clock Disable Macros for GPIOx Peripherals
 */
#define GPIOA_PCLK_DIS() 		(RCC -> AHB1ENR &= ~(1 << 0));
#define GPIOB_PCLK_DIS() 		(RCC -> AHB1ENR &= ~(1 << 1));
#define GPIOC_PCLK_DIS() 		(RCC -> AHB1ENR &= ~(1 << 2));
#define GPIOD_PCLK_DIS() 		(RCC -> AHB1ENR &= ~(1 << 3));
#define GPIOE_PCLK_DIS() 		(RCC -> AHB1ENR &= ~(1 << 4));
#define GPIOF_PCLK_DIS() 		(RCC -> AHB1ENR &= ~(1 << 5));
#define GPIOG_PCLK_DIS() 		(RCC -> AHB1ENR &= ~(1 << 6));
#define GPIOH_PCLK_DIS() 		(RCC -> AHB1ENR &= ~(1 << 7));

/*
 * Clock Disable Macros for I2Cx
 */

#define I2C1_PCLK_DIS()			(RCC -> APB1ENR &= ~(1 << 21));
#define I2C2_PCLK_DIS()			(RCC -> APB1ENR &= ~(1 << 22));
#define I2C3_PCLK_DIS()			(RCC -> APB1ENR &= ~(1 << 23));

/*
 * Clock Disable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_DIS()			(RCC -> APB2ENR &= ~(1 << 12));
#define SPI2_PCLK_DIS()			(RCC -> APB1ENR &= ~(1 << 14));
#define SPI3_PCLK_DIS()			(RCC -> APB1ENR &= ~(1 << 15));
#define SPI4_PCLK_DIS()			(RCC -> APB2ENR &= ~(1 << 13));

/*
 * Clock Disable Macros for USARTx/UARTx Peripherals
 */
#define USART2_PCLK_DIS()		(RCC -> APB1ENR &= ~(1 << 17));
#define USART3_PCLK_DIS()		(RCC -> APB1ENR &= ~(1 << 18));
#define UART4_PCLK_DIS()		(RCC -> APB1ENR &= ~(1 << 19));
#define UART5_PCLK_DIS()		(RCC -> APB1ENR &= ~(1 << 20));
#define USART1_PCLK_DIS()		(RCC -> APB2ENR &= ~(1 << 4));
#define USART6_PCLK_DIS()		(RCC -> APB2ENR &= ~(1 << 5));

/*
 * Clock Disable Macros for Basic TImer Peripherals
 */
#define TIM6_PCLK_DIS()			(RCC -> APB1ENR &= ~(1 << 4))
#define TIM7_PCLK_DIS()			(RCC -> APB1ENR &= ~(1 << 5))

/*
 * Clock Disable Macros for System Configuration
 */
#define SYSCFG_PCLK_DIS()		(RCC -> APB2ENR &= ~(1 << 14));

/*
 * GPIOx Reset
 */
#define GPIOA_RESET()			do{RCC -> AHB1RSTR |= (1 << 0); RCC -> AHB1RSTR &= ~(1 << 0);}while(0)
#define GPIOB_RESET()			do{RCC -> AHB1RSTR |= (1 << 1); RCC -> AHB1RSTR &= ~(1 << 1);}while(0)
#define GPIOC_RESET()			do{RCC -> AHB1RSTR |= (1 << 2); RCC -> AHB1RSTR &= ~(1 << 2);}while(0)
#define GPIOD_RESET()			do{RCC -> AHB1RSTR |= (1 << 3); RCC -> AHB1RSTR &= ~(1 << 3);}while(0)
#define GPIOE_RESET()			do{RCC -> AHB1RSTR |= (1 << 4); RCC -> AHB1RSTR &= ~(1 << 4);}while(0)
#define GPIOF_RESET()			do{RCC -> AHB1RSTR |= (1 << 5); RCC -> AHB1RSTR &= ~(1 << 5);}while(0)
#define GPIOG_RESET()			do{RCC -> AHB1RSTR |= (1 << 6); RCC -> AHB1RSTR &= ~(1 << 6);}while(0)
#define GPIOH_RESET()			do{RCC -> AHB1RSTR |= (1 << 7); RCC -> AHB1RSTR &= ~(1 << 7);}while(0)





#endif /* INC_STM32F446XX_H_ */
