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
 * GPIO pin configuration structure
 */
typedef struct{
	uint8_t	PinNumber;
	uint8_t	PinMode;
	uint8_t	PinSpeed;
	uint8_t	PinPuPdControl;
	uint8_t	PinOPType;
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
void DRV_GPIO_PinDeInit(HAL_GPIO_RegDef_t* pGPIOx);

/*
 * Data Read Write
 */
uint8_t DRV_GPIO_PinRead(HAL_GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);
uint32_t DRV_GPIO_PortRead(HAL_GPIO_RegDef_t* pGPIOx);
void DRV_GPIO_PinWrite(HAL_GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t data);
void DRV_GPIO_PortWrite(HAL_GPIO_RegDef_t* pGPIOx, uint16_t data);
void DRV_GPIO_PinToggle(HAL_GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);

/*
 * Interrupt Handling
 */
void DRV_GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EN_Di);
void DRV_GPIO_IRQHandle(uint8_t PinNumber);



#endif /* INC_STM32F466XX_GPIO_DRIVER_H_ */