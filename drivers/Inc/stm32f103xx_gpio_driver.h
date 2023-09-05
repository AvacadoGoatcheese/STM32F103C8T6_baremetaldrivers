/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Aug 14, 2023
 *      Author: aghos
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_

#include <inttypes.h>
#include "stm32f103xx.h"

// Struct for handlers/config
typedef struct {
	uint8_t GPIO_PinNumber;				// Possible Values from 1-15 for stm32f103xx
	uint8_t GPIO_PinModeY; 				// Possible Values from @GPIO_PIN_MODE
	uint8_t GPIO_PinConfig;				// Possible Values from @GPIO_PINCONFIG_VALUES
//	uint8_t GPIO_PinAltFxnMode;  		// Will Be implemented later
} GPIO_PinConfig_t;

typedef struct {
//	pointer to base address of GPIO periph
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

//@GPIO_PIN_MODE
#define GPIO_MODE_INP				0
#define GPIO_MODE_OUT_10MHZ			1
#define GPIO_MODE_OUT_2MHZ			2
#define GPIO_MODE_OUT_50MHZ			3

#define GPIO_MODE_INT_FT			4
#define GPIO_MODE_INT_RT			5
#define GPIO_MODE_INT_RFT			6

//
/*
 * @GPIO_PINCONFIG_VALUES
 * Note				- Usage of CNFIN or CNFOUT depends on GPIO_PIN_MODE being input (CNFIN)
 * 						or output (CNFOUT).
 */
#define GPIO_CNFIN_ANALOG				0x0
#define GPIO_CNFIN_FLOATING				0x1
#define GPIO_CNFIN_PULLUPDOWN			0x2
#define GPIO_CNFIN_RESERVED				0x3

#define GPIO_CNFOUT_PUSHPULL			0x4
#define GPIO_CNFOUT_OPDR				0x5
#define AFIO_CNFOUT_PUSHPULL			0x6
#define AFIO_CNFOUT_OPDR				0x7

//// @GPIO_OUTPUT_TYPE_CTRL
//#define GPIO_MODE_OUT_PSPL  0
//#define GPIO_MODE_OUT_OPDR	1
//#define AFIO_MODE_OUT_PSPL	2
//#define AFIO_MODE_OUT_OPDR	3
//
//// @GPIO_PIN_SPEEDS
//#define GPIO_MODE_2MHZ		0
//#define GPIO_MODE_10MHZ		1
//#define GPIO_MODE_50MHZ		2
//
//// @GPIO_PULLUP_PULLDOWN
//#define GPIO_NO_PUPD		0
//#define GPIO_MODE_PU		1
//#define GPIO_MODE_PD		2
//


// GPIO Config
uint8_t GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

void GPIO_PCLKControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// GPIO IO
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);

// GPIO Interrupt stuff
uint8_t GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
