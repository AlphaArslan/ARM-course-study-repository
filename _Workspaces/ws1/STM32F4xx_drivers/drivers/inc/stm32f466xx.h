/*
 * stm32f466xx.h
 *
 *  Created on: Sep 14, 2021
 *      Author: alpha_arslan
 */

#ifndef INC_STM32F466XX_H_
#define INC_STM32F466XX_H_

#include<stdint.h>

/*
 * Generic Definitions
 */
#define		__vo				volatile
#define		ENABLE				1
#define		DISABLE				0
#define		SET					1
#define		RESET				0
#define		HIGH				1
#define		LOW					0
#define		USR_BTN_PRESSED		LOW


/*
 * Cortex-M4 NVIC register Addresses
 */
#define		HAL_NVIC_ISER0			((__vo uint32_t*) 0xE000E100UL)
#define		HAL_NVIC_ISER1			((__vo uint32_t*) 0xE000E104UL)
#define		HAL_NVIC_ISER2			((__vo uint32_t*) 0xE000E108UL)
#define		HAL_NVIC_ISER3			((__vo uint32_t*) 0xE000E10CUL)
#define		HAL_NVIC_ISER4			((__vo uint32_t*) 0xE000E110UL)
#define		HAL_NVIC_ISER5			((__vo uint32_t*) 0xE000E114UL)
#define		HAL_NVIC_ISER6			((__vo uint32_t*) 0xE000E118UL)
#define		HAL_NVIC_ISER7			((__vo uint32_t*) 0xE000E11CUL)

#define		HAL_NVIC_ICER0			((__vo uint32_t*) 0xE000E180UL)
#define		HAL_NVIC_ICER1			((__vo uint32_t*) 0xE000E184UL)
#define		HAL_NVIC_ICER2			((__vo uint32_t*) 0xE000E188UL)
#define		HAL_NVIC_ICER3			((__vo uint32_t*) 0xE000E18CUL)
#define		HAL_NVIC_ICER4			((__vo uint32_t*) 0xE000E190UL)
#define		HAL_NVIC_ICER5			((__vo uint32_t*) 0xE000E194UL)
#define		HAL_NVIC_ICER6			((__vo uint32_t*) 0xE000E198UL)
#define		HAL_NVIC_ICER7			((__vo uint32_t*) 0xE000E19CUL)

#define		HAL_NVIC_IPR0			((__vo uint32_t*) 0xE000E400UL)

// the number of bits that are not applicable in each section in IPRx
#define		HAL_NVIC_PR_NABITS		4

/***************************************************/

/*
 * Flash and SRAM base addresses
 */

#define		HAL_FLASH_BASEADDR		0x08000000UL
#define		HAL_SRAM1_BASEADDR		0x20000000UL
#define 	HAL_SRAM				SRAM1_BASEADDR
#define 	HAL_SRAM2_BASEADDR		0x2001C000UL
#define 	HAL_ROM					0x1FFF0000UL

/**************************************************/


/*
 * Peripheral Base Address Definition
 */
// GPIO
#define		HAL_GPIOA_BASEADDR		0x40020000UL
#define		HAL_GPIOB_BASEADDR		0x40020400UL
#define		HAL_GPIOC_BASEADDR		0x40020800UL
#define		HAL_GPIOD_BASEADDR		0x40020C00UL
#define		HAL_GPIOE_BASEADDR		0x40021000UL
#define		HAL_GPIOF_BASEADDR		0x40021400UL
#define		HAL_GPIOG_BASEADDR		0x40021800UL
#define		HAL_GPIOH_BASEADDR		0x40021C00UL

#define		HAL_RCC_BASEADDR		0x40023800UL

#define		HAL_EXTI_BASEADDR		0x40013C00UL

#define		HAL_SYSCFG_BASEADDR		0x40013800UL
/**************************************************/


/*
 * Peripherals register structures definition and macros
 */
// GPIO
typedef struct{
	__vo uint32_t	MODER;			/*!< Mode Register >*/
	__vo uint32_t	OTYPER;			/*!< Output Type Register >*/
	__vo uint32_t	OSPEEDER;		/*!< Output Speed Register >*/
	__vo uint32_t	PUPDR;			/*!< Pull Up Down Register >*/
	__vo uint32_t	IDR;			/*!< Input Data Register >*/
	__vo uint32_t	ODR;			/*!< Output Data Register >*/
	__vo uint32_t	BSRR;			/*!< Bit Set Reset Register >*/
	__vo uint32_t	LCKR;			/*!< Config. Lock Register >*/
	__vo uint32_t	AFRL;			/*!< Alt. Function Register Low >*/
	__vo uint32_t	AFRH;			/*!< Alt. Function Register High>*/

}HAL_GPIO_RegDef_t;

// 98.02.21

// RCC
typedef struct{
	__vo uint32_t 	CR;
	__vo uint32_t 	PLL_CFGR;
	__vo uint32_t 	CFGR;
	__vo uint32_t 	CIR;
	__vo uint32_t 	AHB1RSTR;
	__vo uint32_t 	AHB2RSTR;
	__vo uint32_t 	AHB3RSTR;
	__vo uint32_t 	_R0;
	__vo uint32_t	APB1RSTR;
	__vo uint32_t 	APB2RSTR;
	__vo uint32_t 	_R1;
	__vo uint32_t 	_R2;
	__vo uint32_t 	AHB1ENR;
	__vo uint32_t 	AHB2ENR;
	__vo uint32_t 	AHB3ENR;
	__vo uint32_t 	_R4;
	__vo uint32_t 	APB1ENR;
	__vo uint32_t 	APB2ENR;
	__vo uint32_t 	_R5;
	__vo uint32_t 	_R6;
	__vo uint32_t 	AHB1_LPENR;
	__vo uint32_t 	AHB2_LPENR;
	__vo uint32_t 	AHB3_LPENR;
	__vo uint32_t 	_R7;
	__vo uint32_t 	APB1_LPENR;
	__vo uint32_t 	APB2_LPENR;
	__vo uint32_t 	_R8;
	__vo uint32_t 	_R9;
	__vo uint32_t 	BDCR;
	__vo uint32_t 	CSR;
	__vo uint32_t 	_R10;
	__vo uint32_t 	_R11;
	__vo uint32_t 	SS_CGR;
	__vo uint32_t 	PLLI2_SCFGR;
	__vo uint32_t 	PLL_SAI_CFGR;
	__vo uint32_t	DCK_CFGR;
	__vo uint32_t	CK_GATENR;
	__vo uint32_t	DCK_CFGR2;

}HAL_RCC_RegDef_t;

// EXTI
typedef struct{
	__vo uint32_t	IMR;		// interrupt mask register
	__vo uint32_t	EMR;		// event mask register
	__vo uint32_t	RTSR;		// rising trigger selection register
	__vo uint32_t	FTSR;		// falling trigger selection register
	__vo uint32_t	SWIER;		// software interrupt event register
	__vo uint32_t	PR;			// pending register

}HAL_EXTI_RegDef_t;

// SYSCFG
typedef struct{
	__vo uint32_t	MEMRMP;		// memory remap
	__vo uint32_t	PMC;		// peripheral mode configuration
	__vo uint32_t	EXTI_CR[4];	// EXTI control registers
	__vo uint32_t	_R[2];
	__vo uint32_t	CMPCR;		// comparison cell control
	__vo uint32_t	_R2[2];
	__vo uint32_t	CFGR;		// configuration

}HAL_SYSCFG_RegDef_t;
/********************* 98-10-11 *******************/



/*
 * Peripheral Definition
 */
// GPIO
#define		HAL_GPIOA		((HAL_GPIO_RegDef_t*) HAL_GPIOA_BASEADDR )
#define		HAL_GPIOB		((HAL_GPIO_RegDef_t*) HAL_GPIOB_BASEADDR )
#define		HAL_GPIOC		((HAL_GPIO_RegDef_t*) HAL_GPIOC_BASEADDR )
#define		HAL_GPIOD		((HAL_GPIO_RegDef_t*) HAL_GPIOD_BASEADDR )
#define		HAL_GPIOE		((HAL_GPIO_RegDef_t*) HAL_GPIOE_BASEADDR )
#define		HAL_GPIOF		((HAL_GPIO_RegDef_t*) HAL_GPIOF_BASEADDR )
#define		HAL_GPIOG		((HAL_GPIO_RegDef_t*) HAL_GPIOG_BASEADDR )
#define		HAL_GPIOH		((HAL_GPIO_RegDef_t*) HAL_GPIOH_BASEADDR )

#define 	HAL_RCC			((HAL_RCC_RegDef_t*) HAL_RCC_BASEADDR )

#define		HAL_EXTI		((HAL_EXTI_RegDef_t*) HAL_EXTI_BASEADDR)

#define		HAL_SYSCFG		((HAL_SYSCFG_RegDef_t*) HAL_SYSCFG_BASEADDR)

/**************************************************/

/*
 * Peripheral clock control macros
 */
#define 	HAL_GPIOA_PCLK_En()		( HAL_RCC->AHB1ENR |= (1 << 0) )
#define 	HAL_GPIOB_PCLK_En()		( HAL_RCC->AHB1ENR |= (1 << 1) )
#define 	HAL_GPIOC_PCLK_En()		( HAL_RCC->AHB1ENR |= (1 << 2) )
#define 	HAL_GPIOD_PCLK_En()		( HAL_RCC->AHB1ENR |= (1 << 3) )
#define 	HAL_GPIOE_PCLK_En()		( HAL_RCC->AHB1ENR |= (1 << 4) )
#define 	HAL_GPIOF_PCLK_En()		( HAL_RCC->AHB1ENR |= (1 << 5) )
#define 	HAL_GPIOG_PCLK_En()		( HAL_RCC->AHB1ENR |= (1 << 6) )
#define 	HAL_GPIOH_PCLK_En()		( HAL_RCC->AHB1ENR |= (1 << 7) )
#define 	HAL_GPIOA_PCLK_Di()		( HAL_RCC->AHB1ENR &= ~(1 << 0) )
#define 	HAL_GPIOB_PCLK_Di()		( HAL_RCC->AHB1ENR &= ~(1 << 1) )
#define 	HAL_GPIOC_PCLK_Di()		( HAL_RCC->AHB1ENR &= ~(1 << 2) )
#define 	HAL_GPIOD_PCLK_Di()		( HAL_RCC->AHB1ENR &= ~(1 << 3) )
#define 	HAL_GPIOE_PCLK_Di()		( HAL_RCC->AHB1ENR &= ~(1 << 4) )
#define 	HAL_GPIOF_PCLK_Di()		( HAL_RCC->AHB1ENR &= ~(1 << 5) )
#define 	HAL_GPIOG_PCLK_Di()		( HAL_RCC->AHB1ENR &= ~(1 << 6) )
#define 	HAL_GPIOH_PCLK_Di()		( HAL_RCC->AHB1ENR &= ~(1 << 7) )

#define		HAL_SYSCFG_PCLK_En()	( HAL_RCC->APB2ENR |= (1 << 14) )
#define		HAL_SYSCFG_PCLK_Di()	( HAL_RCC->APB2ENR &= ~(1 << 14) )

/**************************************************/

/*
 * Peripheral reset macros
 */
// GPIO
#define 	HAL_GPIOA_RST()		do{ HAL_RCC->AHB1RSTR |= (1<<0); HAL_RCC->AHB1RSTR &= ~(1<<0);}while(0);
#define 	HAL_GPIOB_RST()		do{ HAL_RCC->AHB1RSTR |= (1<<1); HAL_RCC->AHB1RSTR &= ~(1<<1);}while(0);
#define 	HAL_GPIOC_RST()		do{ HAL_RCC->AHB1RSTR |= (1<<2); HAL_RCC->AHB1RSTR &= ~(1<<2);}while(0);
#define 	HAL_GPIOD_RST()		do{ HAL_RCC->AHB1RSTR |= (1<<3); HAL_RCC->AHB1RSTR &= ~(1<<3);}while(0);
#define 	HAL_GPIOE_RST()		do{ HAL_RCC->AHB1RSTR |= (1<<4); HAL_RCC->AHB1RSTR &= ~(1<<4);}while(0);
#define 	HAL_GPIOF_RST()		do{ HAL_RCC->AHB1RSTR |= (1<<5); HAL_RCC->AHB1RSTR &= ~(1<<5);}while(0);
#define 	HAL_GPIOG_RST()		do{ HAL_RCC->AHB1RSTR |= (1<<6); HAL_RCC->AHB1RSTR &= ~(1<<6);}while(0);
#define 	HAL_GPIOH_RST()		do{ HAL_RCC->AHB1RSTR |= (1<<7); HAL_RCC->AHB1RSTR &= ~(1<<7);}while(0);

/**************************************************/

/*
 * IRQ numbers
 */
// EXTI
#define		HAL_IRQ_NUM_EXTI_0		6
#define		HAL_IRQ_NUM_EXTI_1		7
#define		HAL_IRQ_NUM_EXTI_2		8
#define		HAL_IRQ_NUM_EXTI_3		9
#define		HAL_IRQ_NUM_EXTI_4		10
#define		HAL_IRQ_NUM_EXTI_5		23
#define		HAL_IRQ_NUM_EXTI_6		23
#define		HAL_IRQ_NUM_EXTI_7		23
#define		HAL_IRQ_NUM_EXTI_8		23
#define		HAL_IRQ_NUM_EXTI_9		23
#define		HAL_IRQ_NUM_EXTI_10		40
#define		HAL_IRQ_NUM_EXTI_11		40
#define		HAL_IRQ_NUM_EXTI_12		40
#define		HAL_IRQ_NUM_EXTI_13		40
#define		HAL_IRQ_NUM_EXTI_14		40
#define		HAL_IRQ_NUM_EXTI_15		40

// 2021-05-31

/**************************************************/


/*
 * Extra Macros
 */
// return a code from 0 to 6 for GPIOA to GPIOH
#define		HAL_GPIOX_CODE(x)  ((x == HAL_GPIOA)? 0:\
								(x == HAL_GPIOB)? 1:\
								(x == HAL_GPIOC)? 2:\
								(x == HAL_GPIOD)? 3:\
								(x == HAL_GPIOE)? 4:\
								(x == HAL_GPIOF)? 5:\
								(x == HAL_GPIOG)? 6:\
								(x == HAL_GPIOH)? 7: -1)

/**************************************************/

#endif /* INC_STM32F466XX_H_ */
