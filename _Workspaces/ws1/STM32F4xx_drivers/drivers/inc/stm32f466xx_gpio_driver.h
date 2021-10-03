/*
 * STM32F4xx_gpio.h
 *
 *  Created on: Sep 17, 2021
 *      Author: Alpha_Arslan
 */

#ifndef INC_STM32F466XX_GPIO_DRIVER_H_
#define INC_STM32F466XX_GPIO_DRIVER_H_

#include"stm32f466xx.h"

/*
 * @GPIO_MODE_X
 * Macros - GPIO pin modes
 */
#define		GPIO_MODE_IN		0
#define		GPIO_MODE_OUT		1
#define		GPIO_MODE_ALTFN		2
#define		GPIO_MODE_ANALOG	3
#define		GPIO_MODE_IT_FT		4	// interrupt mode - falling edge trigger
#define		GPIO_MODE_IT_RT		5	// interrupt mode - rising edge trigger
#define		GPIO_MODE_IT_FRT	6

/*
 * @GPIO_OP_TYPE_X
 * Macros - GPIO pin output types
 */
#define		GPIO_OP_TYPE_PP		0	// push pull
#define		GPIO_OP_TYPE_OD		1	// open drain

/*
 * @GPIO_SPEED_X
 * Macros - GPIO pin output speed
 */
#define		GPIO_SPEED_LOW		0
#define		GPIO_SPEED_MEDUIM	1
#define		GPIO_SPEED_FAST		2
#define		GPIO_SPEED_HIGH		3

/*
 * @GPIO_PUPD_X
 * Macros - GPIO pin Pull up/down
 */
#define		GPIO_PUPD_NO		0
#define		GPIO_PUPD_PU		1
#define		GPIO_PUPD_PD		2

/***************************************/

/*
 * GPIO pin configuration structure
 */
typedef struct{
	uint8_t	PinNumber;
	uint8_t	PinMode;				/*!< Possible values: @GPIO_MODE_X>*/
	uint8_t	PinSpeed;				/*!< Possible values: @GPIO_SPEED_X>*/
	uint8_t	PinPuPdControl;			/*!< Possible values: @GPIO_PUPD_X>*/
	uint8_t	PinOPType;				/*!< Possible values: @GPIO_OP_TYPE_X>*/
	uint8_t	PinAltFunMode;
}DRV_GPIO_PinConfig_t;


/*
 * GPIO pin handler structure
 */
typedef struct{
	HAL_GPIO_RegDef_t		*pGPIOx;		/*!< GPIO port of the pin >*/
	DRV_GPIO_PinConfig_t	PinConfig;		/*!< Config. Structure of the pin >*/
}DRV_GPIO_PinHandle_t;



/****************************************
 * 		APIs Supported by this driver
 ****************************************/

/*
 * Peripheral Clock Handling
 */
void DRV_GPIO_PCLKControl(HAL_GPIO_RegDef_t* pGPIOx, uint8_t En_Di);

/*
 * Initialization and De-initialization
 */
void DRV_GPIO_PinInit(DRV_GPIO_PinHandle_t* pGPIO_PinHandle);
void DRV_GPIO_PortDeInit(HAL_GPIO_RegDef_t* pGPIOx);

/*
 * Data Read Write
 */
uint8_t 	DRV_GPIO_PinRead(HAL_GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);
uint16_t 	DRV_GPIO_PortRead(HAL_GPIO_RegDef_t* pGPIOx);
void 		DRV_GPIO_PinWrite(HAL_GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t data);
void 		DRV_GPIO_PortWrite(HAL_GPIO_RegDef_t* pGPIOx, uint16_t data);
void 		DRV_GPIO_PinToggle(HAL_GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);

/*
 * Interrupt Handling
 */
void DRV_GPIO_IRQEnable(uint8_t IRQNumber, uint8_t En_Di);
void DRV_GPIO_IRQPriority(uint8_t IRQNumber, uint8_t priority);
void DRV_GPIO_IRQHandle(uint8_t PinNumber);



#endif /* INC_STM32F466XX_GPIO_DRIVER_H_ */
