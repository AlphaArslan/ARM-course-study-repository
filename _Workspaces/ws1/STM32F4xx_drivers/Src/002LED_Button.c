/*
 * 002LED_Button.c
 *
 * toggle the LED only when the button is pressed
 *
 *  Created on: Oct 1, 2021
 *      Author: Alpha Arslan
 */


#include "stm32f466xx.h"
#include "stm32f466xx_gpio_driver.h"

int main(void){

	// LED pin handle
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

	// Button pin handle
	DRV_GPIO_PinHandle_t button_h;
	button_h.pGPIOx = HAL_GPIOC;			// user button is connect to PC13
	button_h.PinConfig.PinNumber = 13;
	button_h.PinConfig.PinMode = GPIO_MODE_IN;
	button_h.PinConfig.PinSpeed = GPIO_SPEED_FAST;
	button_h.PinConfig.PinPuPdControl = GPIO_PUPD_NO;
	// enable the peripheral clock and init the pin
	DRV_GPIO_PCLKControl(button_h.pGPIOx, ENABLE);
	DRV_GPIO_PinInit(&button_h);


	for(;;){
		if( DRV_GPIO_PinRead(button_h.pGPIOx, button_h.PinConfig.PinNumber ) == USR_BTN_PRESSED){
			DRV_GPIO_PinToggle(user_led_h.pGPIOx, user_led_h.PinConfig.PinNumber);
			for(uint32_t i = 0; i < 200000; i++);		// lazy debouncing
		}
	}

	return 0;
}
