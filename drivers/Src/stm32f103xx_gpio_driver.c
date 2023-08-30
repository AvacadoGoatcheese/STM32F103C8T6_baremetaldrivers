/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Aug 14, 2023
 *      Author: aghos
 */
#include "stm32f103xx_gpio_driver.h"

// GPIO Config

/*******************************************
 * @fxn					- GPIO_Init
 *
 * @brief				- none
 *
 * @param				- GPIO Handler with pin config and register mapping
 *
 * @return				- none
 *
 * @Note				- none
********************************************/
uint8_t GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint8_t *pinNum 	= &((pGPIOHandle->GPIO_PinConfig).GPIO_PinNumber);
	uint8_t *pinMode 	= &((pGPIOHandle->GPIO_PinConfig).GPIO_PinModeY);
	uint8_t *pinConfig 	= &((pGPIOHandle->GPIO_PinConfig).GPIO_PinConfig);

	if (*pinConfig == GPIO_CNFIN_RESERVED || *pinNum > 16) {
		return ERROR;
	}

	if (*pinMode < GPIO_MODE_INT_FT) {
		/*
			 * There are 2 sets of registers. One for pins 1-7 and the other for pins 8-15.
			 * In interest of keeping code length to a minimum, we are only changing the registers
			 * and the effective pin number (with regards to the register numbering) to keep the code
			 * identical for the low and high control registers (CRL/CRH).
			 */
			uint32_t _vo *GPIO_Ctrl_Reg = &(pGPIOHandle->pGPIOx->GPIO_CRL);
			if (*pinNum >= 8) {
				*pinNum -= 8;
				GPIO_Ctrl_Reg = &(pGPIOHandle->pGPIOx->GPIO_CRH);
			}

			if (*pinMode > GPIO_MODE_INP) {
				/*
				 * GPIO_PinConfig has a 2 sets of possible values: 0->3 and 4->7. If GPIO_PinModeY is
				 * isn't input, then we need to use the second range. Otherwise we use the first range.
				 *
				 * If we use the second range, then we need to subtract 4 from the second range to keep it
				 * representable in binary with 2 bits (AKA 0, 1, 2, 3).
				 */
				*pinConfig -= 4;

			}

			*GPIO_Ctrl_Reg 		&= ~( 0x0000000F << (4 * (*pinNum)) ); // Clearing bits that were there beforehand.
			*GPIO_Ctrl_Reg 		|= ( *pinMode << (4 * (*pinNum)) );
			*GPIO_Ctrl_Reg 		|= ( *pinConfig << (4 * (*pinNum) + 2) );
	} else {
		uint8_t port = 0;

		if		(pGPIOHandle->pGPIOx == GPIOA) 		{ port = 0; }
		else if (pGPIOHandle->pGPIOx == GPIOB) 		{ port = 1; }
		else if (pGPIOHandle->pGPIOx == GPIOC) 		{ port = 2; }
		else if (pGPIOHandle->pGPIOx == GPIOD) 		{ port = 3; }
		else if (pGPIOHandle->pGPIOx == GPIOE) 		{ port = 4; }
		else if (pGPIOHandle->pGPIOx == GPIOF) 		{ port = 5; }
		else if (pGPIOHandle->pGPIOx == GPIOG) 		{ port = 6; }

		// Set EXTIx line to selected port (x=A..G) using EXTICRy registers (y=1..4)
		if (*pinNum <= 3) {
			// pins 0 -> 3
			pGPIOHandle->pGPIOx->AFIO_EXTICR1 &= ~(0b1111) << 4 * pinNum;
			pGPIOHandle->pGPIOx->AFIO_EXTICR1 |= port << 4 * pinNum;
		} else if (*pinNum <= 7) {
			// pins 4 -> 7
			pGPIOHandle->pGPIOx->AFIO_EXTICR2 &= ~(0b1111) << 4 * (pinNum - 4);
			pGPIOHandle->pGPIOx->AFIO_EXTICR2 |= port << 4 * (pinNum - 4);
		} else if (*pinNum <= 11) {
			// pins 8 -> 11
			pGPIOHandle->pGPIOx->AFIO_EXTICR3 &= ~(0b1111) << 4 * (pinNum - 8);
			pGPIOHandle->pGPIOx->AFIO_EXTICR3 |= port << 4 * (pinNum - 8);
		} else (*pinNum <= 15) {
			// pins 12 -> 15
			pGPIOHandle->pGPIOx->AFIO_EXTICR4 &= ~(0b1111) << 4 * (pinNum - 12);
			pGPIOHandle->pGPIOx->AFIO_EXTICR4 |= port << 4 * (pinNum - 12);
		}

		if (*pinMode == GPIO_MODE_INT_FT) {
			EXTI_LINE_ENABLE_FT(*pinNum);
		} else if (*pinMode == GPIO_MODE_INT_RT) {
			EXTI_LINE_ENABLE_RT(*pinNum);
		} else if (*pinMode == GPIO_MODE_INT_RFT) {
			EXTI_LINE_ENABLE_FT(*pinNum);
			EXTI_LINE_ENABLE_RT(*pinNum);
		}
	}



}

/*******************************************
 * @fxn					- GPIO_DeInit
 *
 * @brief				- resets GPIO pin
 *
 * @param				- pointer to GPIO Port's registers
 *
 * @return				- none
 *
 * @Note				- none
********************************************/
uint8_t GPIO_DeInit(IO_RegDef_t *pGPIOx) {
	if		(pGPIOx == GPIOA) 		{ GPIOA_REG_RESET(); }
	else if (pGPIOx == GPIOB) 		{ GPIOB_REG_RESET(); }
	else if (pGPIOx == GPIOC) 		{ GPIOC_REG_RESET(); }
	else if (pGPIOx == GPIOD) 		{ GPIOD_REG_RESET(); }
	else if (pGPIOx == GPIOE) 		{ GPIOE_REG_RESET(); }
	else if (pGPIOx == GPIOF) 		{ GPIOF_REG_RESET(); }
	else if (pGPIOx == GPIOG) 		{ GPIOG_REG_RESET(); }

	return 0; // future error testing
}

/*******************************************
 * @fxn					- GPIO_PCLKControl
 *
 * @brief				- Initializes Peripheral Clock for GPIO (APB2 for stm32f103xx)
 *
 * @param				- Registers of GPIO port
 * @param				- Enable or Disable macro (defined in board header file)
 *
 * @return				- none (uint8_t now for possible error code expansion)
 *
 * @Note				- none
********************************************/
uint8_t GPIO_PCLKControl(IO_RegDef_t *pGPIOx, uint8_t EnOrDi){
	if (EnOrDi == ENABLE) {
		// Figure out a nicer way to do this
		if		(pGPIOx == GPIOA) 		{ GPIOA_PCLK_EN(); }
		else if (pGPIOx == GPIOB) 		{ GPIOB_PCLK_EN(); }
		else if (pGPIOx == GPIOC) 		{ GPIOC_PCLK_EN(); }
		else if (pGPIOx == GPIOD) 		{ GPIOD_PCLK_EN(); }
		else if (pGPIOx == GPIOE) 		{ GPIOE_PCLK_EN(); }
		else if (pGPIOx == GPIOF) 		{ GPIOF_PCLK_EN(); }
		else if (pGPIOx == GPIOG) 		{ GPIOG_PCLK_EN(); }
	} else {
		//disable
		if		(pGPIOx == GPIOA) 		{ GPIOA_PCLK_DI(); }
		else if (pGPIOx == GPIOB) 		{ GPIOB_PCLK_DI(); }
		else if (pGPIOx == GPIOC) 		{ GPIOC_PCLK_DI(); }
		else if (pGPIOx == GPIOD) 		{ GPIOD_PCLK_DI(); }
		else if (pGPIOx == GPIOE) 		{ GPIOE_PCLK_DI(); }
		else if (pGPIOx == GPIOF) 		{ GPIOF_PCLK_DI(); }
		else if (pGPIOx == GPIOG) 		{ GPIOG_PCLK_DI(); }
	}

	return 0; // for future error testing
}

/*******************************************
 * @fxn					- GPIO_ToggleOutputPin
 *
 * @brief				- Turns on Output pin
 *
 * @param				-
 * @param				-
 *
 * @return				-
 *
 * @Note				-
********************************************/
uint8_t GPIO_ToggleOutputPin(IO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->GPIO_ODR ^= (1U << PinNumber);
}

// GPIO IO

/*******************************************
 * @fxn					- GPIO_ReadFromInputPin
 *
 * @brief				- Returns Input Data Register value
 *
 * @param				- GPIO Register for PortX
 * @param				- Pin number (int)
 *
 * @return				- 0 or 1
 *
 * @Note				- none
********************************************/
uint8_t GPIO_ReadFromInputPin(IO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	return (uint8_t)((pGPIOx->GPIO_IDR >> PinNumber) & 1U);
}

/*******************************************
 * @fxn					- GPIO_Init
 *
 * @brief				-
 *
 * @param				-
 * @param				-
 *
 * @return				-
 *
 * @Note				-
********************************************/
uint16_t GPIO_ReadFromInputPort(IO_RegDef_t *pGPIOx) {
	return (uint16_t)( pGPIOx->GPIO_IDR & 0x00FF);
}

/*******************************************
 * @fxn					- GPIO_Init
 *
 * @brief				-
 *
 * @param				-
 * @param				-
 *
 * @return				-
 *
 * @Note				-
********************************************/
uint8_t GPIO_WriteToOutputPin(IO_RegDef_t *pGPIOx, uint8_t PinNumber,
							uint8_t value) {
	pGPIOx->GPIO_ODR |= (uint32_t)((value & 1U)) << PinNumber;
	return 0;
}

/*******************************************
 * @fxn					- GPIO_Init
 *
 * @brief				-
 *
 * @param				-
 * @param				-
 *
 * @return				-
 *
 * @Note				-
********************************************/
uint8_t GPIO_WriteToOutputPort(IO_RegDef_t *pGPIOx, uint16_t value) {
	pGPIOx->GPIO_IDR = value;
}

// GPIO Interrupt stuff

/*******************************************
 * @fxn					- GPIO_Init
 *
 * @brief				-
 *
 * @param				-
 * @param				-
 *
 * @return				-
 *
 * @Note				-
********************************************/
uint8_t GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi) {

}

/*******************************************
 * @fxn					- GPIO_Init
 *
 * @brief				-
 *
 * @param				-
 * @param				-
 *
 * @return				-
 *
 * @Note				-
********************************************/
uint8_t GPIO_IRQHandling(uint8_t PinNumber) {

}





