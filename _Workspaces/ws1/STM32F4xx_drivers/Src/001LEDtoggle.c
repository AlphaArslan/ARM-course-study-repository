/*
 * 001LEDtoggle.c
 *
 *  Created on: Oct 1, 2021
 *      Author: Alpha Arslan
 */

#include "stm32f466xx.h"
#include "stm32f466xx_gpio_driver.h"

int main(void){
	DRV_GPIO_PinHandle_t user_led_h;		// handle structure for the on-board LED ( PA5 )

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

	for(;;){
		DRV_GPIO_PinToggle(user_led_h.pGPIOx, user_led_h.PinConfig.PinNumber);
		for(uint32_t i = 0; i < 500000; i++);		// pseudo delay
	}


	return 0;
}
