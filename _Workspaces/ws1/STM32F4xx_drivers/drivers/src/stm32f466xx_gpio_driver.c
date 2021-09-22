/*
 * STM32F4xx_gpio_driver.c
 *
 *  Created on: Sep 17, 2021
 *      Author: Alpha_Arslan
 */

#include "stm32f466xx_gpio_driver.h"

/****************************************
 * 		APIs Supported by this driver
 ****************************************/

/**********************************************
 * @fn			-	DRV_GPIO_PCLKControl
 *
 * @breif		-	Enables/Disables the Peripheral clock for GPIO port
 *
 * @param[in]	-	GPIO port base address
 * @param[in]	-	ENABLE or DISABLE macros
 * @param[in]	-
 *
 * @return		-	none
 *
 * @Note		-	none
 **********************************************/
void DRV_GPIO_PCLKControl(HAL_GPIO_RegDef_t* pGPIOx, uint8_t En_Di){

}




/**********************************************
 * @fn			-	DRV_GPIO_PinInit
 *
 * @breif		-	Initializes a GPIO pin
 *
 * @param[in]	-	pointer to the pin handle structure
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-	none
 *
 * @Note		-	none
 **********************************************/
void DRV_GPIO_PinInit(DRV_GPIO_PinHandle_t* pGPIO_PinHandle){

}


/**********************************************
 * @fn			-	DRV_GPIO_PinDeInit
 *
 * @breif		-	De-initializes a GPIO pin
 *
 * @param[in]	-	GPIO port base address
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-	none
 *
 * @Note		-	none
 **********************************************/
void DRV_GPIO_PinDeInit(HAL_GPIO_RegDef_t* pGPIOx){

}




/**********************************************
 * @fn			-	DRV_GPIO_PinRead
 *
 * @breif		-	Reads an input pin
 *
 * @param[in]	-	GPIO port base address
 * @param[in]	-	Pin number
 * @param[in]	-
 *
 * @return		-	Input data
 *
 * @Note		-	none
 **********************************************/
uint8_t DRV_GPIO_PinRead(HAL_GPIO_RegDef_t* pGPIOx, uint8_t PinNumber){

}


/**********************************************
 * @fn			-	DRV_GPIO_PortRead
 *
 * @breif		-	Reads an input port
 *
 * @param[in]	-	GPIO port base address
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-	Input data
 *
 * @Note		-	none
 **********************************************/
uint32_t DRV_GPIO_PortRead(HAL_GPIO_RegDef_t* pGPIOx){

}


/**********************************************
 * @fn			-	DRV_GPIO_PinWrite
 *
 * @breif		-	writes data to an output pin
 *
 * @param[in]	-	GPIO port base address
 * @param[in]	-	Pin number
 * @param[in]	-	data
 *
 * @return		-	none
 *
 * @Note		-	none
 **********************************************/
void DRV_GPIO_PinWrite(HAL_GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t data){

}


/**********************************************
 * @fn			-	DRV_GPIO_PortWrite
 *
 * @breif		-	writes data to an output port
 *
 * @param[in]	-	GPIO port base address
 * @param[in]	-	data
 * @param[in]	-
 *
 * @return		-	none
 *
 * @Note		-	none
 **********************************************/
void DRV_GPIO_PortWrite(HAL_GPIO_RegDef_t* pGPIOx, uint16_t data){

}


/**********************************************
 * @fn			-	DRV_GPIO_PinToggle
 *
 * @breif		-	toggles an output pin
 *
 * @param[in]	-	GPIO port base address
 * @param[in]	-	Pin number
 * @param[in]	-
 *
 * @return		-	none
 *
 * @Note		-	none
 **********************************************/
void DRV_GPIO_PinToggle(HAL_GPIO_RegDef_t* pGPIOx, uint8_t PinNumber){

}



/**********************************************
 * @fn			-	DRV_GPIO_IRQConfig
 *
 * @breif		-	Config the IRQ
 *
 * @param[in]	-	IRQ number
 * @param[in]	-	IRQ priority
 * @param[in]	-	ENABLE / DISABLE macros
 *
 * @return		-	none
 *
 * @Note		-	none
 **********************************************/
void DRV_GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EN_Di){

}


/**********************************************
 * @fn			-	DRV_GPIO_IRQHandle
 *
 * @breif		-	Handles the interrupt
 *
 * @param[in]	-	Interrupt pin number
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-	none
 *
 * @Note		-	none
 **********************************************/
void DRV_GPIO_IRQHandle(uint8_t PinNumber){

}