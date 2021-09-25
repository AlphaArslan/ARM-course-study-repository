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
 * @brief		-	Enables/Disables the Peripheral clock for GPIO port
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
	if(En_Di == ENABLE){
			 if(pGPIOx == HAL_GPIOA)	{HAL_GPIOA_PCLK_En();}
		else if(pGPIOx == HAL_GPIOB)	{HAL_GPIOB_PCLK_En();}
		else if(pGPIOx == HAL_GPIOC)	{HAL_GPIOC_PCLK_En();}
		else if(pGPIOx == HAL_GPIOD)	{HAL_GPIOD_PCLK_En();}
		else if(pGPIOx == HAL_GPIOE)	{HAL_GPIOE_PCLK_En();}
		else if(pGPIOx == HAL_GPIOF)	{HAL_GPIOF_PCLK_En();}
		else if(pGPIOx == HAL_GPIOG)	{HAL_GPIOG_PCLK_En();}
		else if(pGPIOx == HAL_GPIOH)	{HAL_GPIOH_PCLK_En();}
	}else{
			 if(pGPIOx == HAL_GPIOA)	HAL_GPIOA_PCLK_Di();
		else if(pGPIOx == HAL_GPIOB)	HAL_GPIOB_PCLK_Di();
		else if(pGPIOx == HAL_GPIOC)	HAL_GPIOC_PCLK_Di();
		else if(pGPIOx == HAL_GPIOD)	HAL_GPIOD_PCLK_Di();
		else if(pGPIOx == HAL_GPIOE)	HAL_GPIOE_PCLK_Di();
		else if(pGPIOx == HAL_GPIOF)	HAL_GPIOF_PCLK_Di();
		else if(pGPIOx == HAL_GPIOG)	HAL_GPIOG_PCLK_Di();
		else if(pGPIOx == HAL_GPIOH)	HAL_GPIOH_PCLK_Di();
	}
}




/**********************************************
 * @fn			-	DRV_GPIO_PinInit
 *
 * @brief		-	Initializes a GPIO pin
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

	uint32_t temp = 0;
	uint32_t shift = 0;

	// 1. mode configuration
	if(pGPIO_PinHandle->PinConfig.PinMode <= GPIO_MODE_ANALOG){
		// non interrupt modes
		// get the two config bits in place
		shift = 2 * pGPIO_PinHandle->PinConfig.PinNumber;
		temp = (pGPIO_PinHandle->PinConfig.PinMode << shift);
		// reset the respective two bits in register
		pGPIO_PinHandle->pGPIOx->MODER &= ~(0x11 << shift);
		// write the two config bits
		pGPIO_PinHandle->pGPIOx->MODER |= temp;


	}else{
		// interrupt modes
		// will be coded later
	}


	// speed
	// shift = 2 * pGPIO_PinHandle->PinConfig.PinNumber;
	temp = (pGPIO_PinHandle->PinConfig.PinSpeed << shift);
	pGPIO_PinHandle->pGPIOx->OSPEEDER &= ~(0x11 << shift);
	pGPIO_PinHandle->pGPIOx->MODER |= temp;


	// pull up pull down
	// shift = 2 * pGPIO_PinHandle->PinConfig.PinNumber;
	temp = (pGPIO_PinHandle->PinConfig.PinPuPdControl << shift);
	pGPIO_PinHandle->pGPIOx->PUPDR &= ~(0x11 << shift);
	pGPIO_PinHandle->pGPIOx->PUPDR |= temp;

	// output type
	shift = 1 * pGPIO_PinHandle->PinConfig.PinNumber;
	temp = (pGPIO_PinHandle->PinConfig.PinOPType << shift);
	pGPIO_PinHandle->pGPIOx->OTYPER &= ~(0x1 << shift);
	pGPIO_PinHandle->pGPIOx->OTYPER |= temp;


	// alternative function
	if( pGPIO_PinHandle->PinConfig.PinMode == GPIO_MODE_ALTFN){
		// configure the Alternate Function
		if(pGPIO_PinHandle->PinConfig.PinNumber < 8){
			// use the low register
			shift = 4 * pGPIO_PinHandle->PinConfig.PinNumber;
			temp = (pGPIO_PinHandle->PinConfig.PinAltFunMode << shift);
			pGPIO_PinHandle->pGPIOx->AFRL &= ~(0x1111 << shift);
			pGPIO_PinHandle->pGPIOx->AFRL |= temp;

		}else{
			// use the high register
			shift = 4 * (pGPIO_PinHandle->PinConfig.PinNumber - 8);
			temp = (pGPIO_PinHandle->PinConfig.PinAltFunMode << shift);
			pGPIO_PinHandle->pGPIOx->AFRH &= ~(0x1111 << shift);
			pGPIO_PinHandle->pGPIOx->AFRH |= temp;

		}
	}
}


/**********************************************
 * @fn			-	DRV_GPIO_PortDeInit
 *
 * @brief		-	De-initializes a GPIO pin
 *
 * @param[in]	-	GPIO port base address
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-	none
 *
 * @Note		-	none
 **********************************************/
void DRV_GPIO_PortDeInit(HAL_GPIO_RegDef_t* pGPIOx){
		 if(pGPIOx == HAL_GPIOA)	{HAL_GPIOA_RST();}
	else if(pGPIOx == HAL_GPIOB)	{HAL_GPIOB_RST();}
	else if(pGPIOx == HAL_GPIOC)	{HAL_GPIOC_RST();}
	else if(pGPIOx == HAL_GPIOD)	{HAL_GPIOD_RST();}
	else if(pGPIOx == HAL_GPIOE)	{HAL_GPIOE_RST();}
	else if(pGPIOx == HAL_GPIOF)	{HAL_GPIOF_RST();}
	else if(pGPIOx == HAL_GPIOG)	{HAL_GPIOG_RST();}
	else if(pGPIOx == HAL_GPIOH)	{HAL_GPIOH_RST();}
}




/**********************************************
 * @fn			-	DRV_GPIO_PinRead
 *
 * @brief		-	Reads an input pin
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
 * @brief		-	Reads an input port
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
 * @brief		-	writes data to an output pin
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
 * @brief		-	writes data to an output port
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
 * @brief		-	toggles an output pin
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
 * @brief		-	Config the IRQ
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
 * @brief		-	Handles the interrupt
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
