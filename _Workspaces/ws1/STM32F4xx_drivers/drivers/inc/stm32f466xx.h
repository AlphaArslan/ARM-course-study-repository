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
#define		__vo		volatile
#define		ENABLE		1
#define		DISABLE		0
#define		SET			1
#define		RESET		0


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


/**************************************************/

/*
 * Peripherals register structures definition
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

/**************************************************/

/*
 * Peripheral Definition
 */

#define		HAL_GPIOA		((HAL_GPIO_RegDef_t*) HAL_GPIOA_BASEADDR )
#define		HAL_GPIOB		((HAL_GPIO_RegDef_t*) HAL_GPIOB_BASEADDR )
#define		HAL_GPIOC		((HAL_GPIO_RegDef_t*) HAL_GPIOC_BASEADDR )
#define		HAL_GPIOD		((HAL_GPIO_RegDef_t*) HAL_GPIOD_BASEADDR )
#define		HAL_GPIOE		((HAL_GPIO_RegDef_t*) HAL_GPIOE_BASEADDR )
#define		HAL_GPIOF		((HAL_GPIO_RegDef_t*) HAL_GPIOF_BASEADDR )
#define		HAL_GPIOG		((HAL_GPIO_RegDef_t*) HAL_GPIOG_BASEADDR )
#define		HAL_GPIOH		((HAL_GPIO_RegDef_t*) HAL_GPIOH_BASEADDR )

/**************************************************/


#endif /* INC_STM32F466XX_H_ */