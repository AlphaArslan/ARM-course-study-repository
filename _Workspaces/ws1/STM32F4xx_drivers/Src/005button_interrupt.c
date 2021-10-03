/*
 * 005button_interrupt.c
 *
 *  Created on: Oct 3, 2021
 *      Author: Alpha Arslan
 */

#include "stm32f466xx.h"
#include "stm32f466xx_gpio_driver.h"

DRV_GPIO_PinHandle_t user_led_h;		// handle structure for the on-board LED ( PA5 )
DRV_GPIO_PinHandle_t button_h;			// user button is connect to PC13

int main(void){

	// set the LED port
	user_led_h.pGPIOx = HAL_GPIOA;			// the on board LED is connected to Port A
	// configure the LED pin
	user_led_h.PinConfig.PinNumber = 5;
	user_led_h.PinConfig.PinMode = GPIO_MODE_OUT;
	user_led_h.PinConfig.PinOPType = GPIO_OP_TYPE_PP;
	user_led_h.PinConfig.PinSpeed = GPIO_SPEED_FAST;
	user_led_h.PinConfig.PinPuPdControl = GPIO_PUPD_NO;
	// enable the peripheral clock and init the pin
	DRV_GPIO_PCLKControl(user_led_h.pGPIOx, ENABLE);
	DRV_GPIO_PinInit(&user_led_h);

	// Button pin handle
	button_h.pGPIOx = HAL_GPIOC;
	button_h.PinConfig.PinNumber = 13;
	button_h.PinConfig.PinMode = GPIO_MODE_IT_FT;		// interrupt falling edge
	button_h.PinConfig.PinSpeed = GPIO_SPEED_FAST;
	button_h.PinConfig.PinPuPdControl = GPIO_PUPD_NO;
	// enable the peripheral clock and init the pin
	DRV_GPIO_PCLKControl(button_h.pGPIOx, ENABLE);
	HAL_SYSCFG_PCLK_En();	// SYSCFG needed for EXTI GPIO switching
	DRV_GPIO_PinInit(&button_h);

	// IRQ settings
	DRV_GPIO_IRQEnable(HAL_IRQ_NUM_EXTI_13, ENABLE);
	DRV_GPIO_IRQPriority(HAL_IRQ_NUM_EXTI_13, 15);


	for(;;);

	return 0;
}

void EXTI15_10_IRQHandler(){
	DRV_GPIO_IRQHandle(button_h.PinConfig.PinNumber);

	DRV_GPIO_PinToggle(user_led_h.pGPIOx, user_led_h.PinConfig.PinNumber);
}
