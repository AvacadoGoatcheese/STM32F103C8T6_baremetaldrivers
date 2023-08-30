/*
 * 001led_toggle.c
 *
 *  Created on: Aug 19, 2023
 *      Author: aghos
 */

#include "stm32f103xx.h"
#include "stm32f103xx_gpio_driver.h"


void delay(int volatile count) {
	while (count-- > 0);
}

int main(void) {
	GPIO_Handle_t LEDGPIO;
	LEDGPIO.pGPIOx = GPIOC;

	LEDGPIO.GPIO_PinConfig.GPIO_PinNumber = 13;
	LEDGPIO.GPIO_PinConfig.GPIO_PinConfig = GPIO_CNFOUT_PUSHPULL;

	LEDGPIO.GPIO_PinConfig.GPIO_PinModeY = GPIO_MODE_OUT_2MHZ;
	GPIO_PCLKControl(GPIOC, ENABLE);

	GPIO_Init(&LEDGPIO);

	while (1) {
		GPIO_ToggleOutputPin(GPIOC, 13);
		delay(500000);
	}

	return 0;
}
