/*
 * stm32f103c8t6.h
 *
 *  Created on: Aug 11, 2023
 *      Author: aghos
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include "inttypes.h"

//Generic
#define _vo 							volatile
#define ENABLE 							1
#define DISABLE 						0
#define SET								ENABLE
#define RESET 							DISABLE
#define GPIO_PIN_SET					SET
#define GPIO_PIN_RESET					RESET
#define ERROR							1
#define OK								0


/*
 * PROCESSOR SPECIFIC DETAILS/MEMORY ADDRESSES		--START
 */


// NVIC Interrupt Enable/Disable
#define P_NVIC_ISER_BASEREG				(_vo uint32_t*)(0xE000E100)
#define P_NVIC_ICER_BASEREG				(_vo uint32_t*)(0XE000E180)

// IRQ Priority Register Base Register
#define P_NVIC_IPR_BASEREG				(_vo uint32_t*)(0xE000E400)

/*
 * PERIPHERAL SPECIFIC DETAILS/MEMORY ADDRESS 		--START
 */


/* base addresses of flash and SRAM */
#define FLASH_BASEADDR					(0x08000000U)
#define SRAM_BASEADDR 					(0x20000000U)

/* ROM = System memory in Reference Manual*/
#define ROM_BASEADDR					(0x1FFFF000U)

#define PERIPH_BASEADDR					(0x40000000U)
#define APB1PERIPH_BASEADDR				(PERIPH_BASEADDR)
#define APB2PERIPH_BASEADDR				(0x40010000U)
#define AHBPERIPH_BASEADDR				(0x40018000U)

// GPIO addresses
#define GPIOA_BASEADDR 					(APB2PERIPH_BASEADDR + 0X0800)
#define GPIOB_BASEADDR 					(APB2PERIPH_BASEADDR + 0X0C00)
#define GPIOC_BASEADDR 					(APB2PERIPH_BASEADDR + 0X1000)
#define GPIOD_BASEADDR 					(APB2PERIPH_BASEADDR + 0X1400)
#define GPIOE_BASEADDR 					(APB2PERIPH_BASEADDR + 0X1800)
#define GPIOF_BASEADDR 					(APB2PERIPH_BASEADDR + 0X1C00)
#define GPIOG_BASEADDR 					(APB2PERIPH_BASEADDR + 0X2000)

//I2C/SPI
#define SPI1_BASEADDR  					(APB2PERIPH_BASEADDR + 0X3000)
#define SPI2_BASEADDR  					(APB1PERIPH_BASEADDR + 0X3800)
#define SPI3_BASEADDR  					(APB1PERIPH_BASEADDR + 0X3C00)
#define I2C1_BASEADDR  					(APB1PERIPH_BASEADDR + 0X5400)
#define I2C2_BASEADDR  					(APB1PERIPH_BASEADDR + 0X5800)

//U(S)ART
#define USART1_BASEADDR 				(APB2PERIPH_BASEADDR + 0X3800)
#define USART2_BASEADDR 				(APB1PERIPH_BASEADDR + 0X4400)
#define USART3_BASEADDR 				(APB1PERIPH_BASEADDR + 0X4800)
#define UART4_BASEADDR 					(APB1PERIPH_BASEADDR + 0X4C00)
#define UART5_BASEADDR 					(APB1PERIPH_BASEADDR + 0X5000)

//MISC STUFF
#define AFIO_BASEADDR					(APB2PERIPH_BASEADDR)
#define EXTI_BASEADDR 					(APB2PERIPH_BASEADDR + 0x0400U)
#define RCC_BASEADDR					(AHBPERIPH_BASEADDR + 0X00009000U)

typedef struct {
	// GPIO Registers
	_vo uint32_t CRL; 				/* Config for Pins 1->7 w/ IO mode */
	_vo uint32_t CRH; 				/* Config for Pins 8->15 w/ IO mode */
	_vo uint32_t IDR; 				/* Input Data Register */
	_vo uint32_t ODR; 				/* Output Data Register */
	_vo uint32_t BSRR; 				/* Bit Set/Reset Register */
	_vo uint32_t BRR; 				/* Another Bit Reset?? Confused a bit on why there's two */
	_vo uint32_t LCKR; 				/* Cannot change port config till reset. */
} GPIO_RegDef_t;

typedef struct {
	_vo uint32_t CR;
	_vo uint32_t CFGR;
	_vo uint32_t CIR;
	_vo uint32_t APB2RSTR;
	_vo uint32_t APB1RSTR;
	_vo uint32_t AHBENR;
	_vo uint32_t APB2ENR;
	_vo uint32_t APB1ENR;
	_vo uint32_t BDCR;
	_vo uint32_t CSR;
} RCC_RegDef_t;

typedef struct {
	_vo uint32_t IMR;
	_vo uint32_t EMR;
	_vo uint32_t RTSR;
	_vo uint32_t FTSR;
	_vo uint32_t SWIER;
	_vo uint32_t PR;
} EXTI_RegDef_t;

typedef struct {
	// AFIO Registers
	_vo uint32_t AFIO_EVCR; 			/* used for Cortex EVENTOUT (SEV instr. to wakeup another MCU?) */
	_vo uint32_t AFIO_MAPR; 			/* Remapping Fxns */
	_vo uint32_t AFIO_EXTICR[4];		/* Choose Ports to connect to EXTI Lines */
	_vo uint32_t AFIO_MAPR2;			/* Remapping Fxns */
} AFIO_RegDef_t;


// Peripheral Definitions (structs stored at base address of peripherals)
#define GPIOA 							((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 							((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 							((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 							((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 							((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 							((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 							((GPIO_RegDef_t*)GPIOG_BASEADDR)

#define RCC								((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI							((EXTI_RegDef_t*)EXTI_BASEADDR)
#define AFIO							((AFIO_RegDef_t*)AFIO_BASEADDR)

// GPIO Register Reset
#define GPIOA_REG_RESET()				do { RCC->APB2RSTR |= (1U << 2); RCC->APB2RSTR &= ~(1U << 2); } while (0)
#define GPIOB_REG_RESET()				do { RCC->APB2RSTR |= (1U << 3); RCC->APB2RSTR &= ~(1U << 3); } while (0)
#define GPIOC_REG_RESET()				do { RCC->APB2RSTR |= (1U << 4); RCC->APB2RSTR &= ~(1U << 4); } while (0)
#define GPIOD_REG_RESET()				do { RCC->APB2RSTR |= (1U << 5); RCC->APB2RSTR &= ~(1U << 5); } while (0)
#define GPIOE_REG_RESET()				do { RCC->APB2RSTR |= (1U << 6); RCC->APB2RSTR &= ~(1U << 6); } while (0)
#define GPIOF_REG_RESET()				do { RCC->APB2RSTR |= (1U << 7); RCC->APB2RSTR &= ~(1U << 7); } while (0)
#define GPIOG_REG_RESET()				do { RCC->APB2RSTR |= (1U << 8); RCC->APB2RSTR &= ~(1U << 8); } while (0)

// EXTI Line Enable
#define EXTI_LINE_ENABLE_FT(num) 		(EXTI->FTSR |= (1U << num))
#define EXTI_LINE_ENABLE_RT(num) 		(EXTI->RTSR |= (1U << num))

//Clock Enable AFIO
#define AFIO_PCLK_EN()					(RCC->APB2ENR |= (1U))
#define AFIO_PCLK_DI()					(RCC->APB2ENR &= ~(1U))


// Clock Enables for GPIO
#define GPIOA_PCLK_EN()					(RCC->APB2ENR |= (1U << 2))
#define GPIOB_PCLK_EN()					(RCC->APB2ENR |= (1U << 3))
#define GPIOC_PCLK_EN()					(RCC->APB2ENR |= (1U << 4))
#define GPIOD_PCLK_EN()					(RCC->APB2ENR |= (1U << 5))
#define GPIOE_PCLK_EN()					(RCC->APB2ENR |= (1U << 6))
#define GPIOF_PCLK_EN()					(RCC->APB2ENR |= (1U << 7))
#define GPIOG_PCLK_EN()					(RCC->APB2ENR |= (1U << 8))

//Clock enable for I2C/SPI peripherals
#define I2C1_PCLK_EN()					(RCC->APB1ENR |= (1U << 21))
#define I2C2_PCLK_EN()					(RCC->APB1ENR |= (1U << 22))
#define SPI1_PCLK_EN()					(RCC->APB2ENR |= (1U << 12))
#define SPI2_PCLK_EN()					(RCC->APB1ENR |= (1U << 14))
#define SPI3_PCLK_EN()					(RCC->APB1ENR |= (1U << 15))

//Clock enable for U(S)ART peripherals
#define USART1_PCLK_EN()				(RCC->APB2ENR |= (1U << 14))
#define USART2_PCLK_EN()				(RCC->APB1ENR |= (1U << 17))
#define USART3_PCLK_EN()				(RCC->APB1ENR |= (1U << 18))
#define UART4_PCLK_EN()					(RCC->APB1ENR |= (1U << 19))
#define UART5_PCLK_EN()					(RCC->APB1ENR |= (1U << 20))

// Clock disables for GPIO
#define GPIOA_PCLK_DI()					(RCC->APB2ENR &= ~(1U << 2))
#define GPIOB_PCLK_DI()					(RCC->APB2ENR &= ~(1U << 3))
#define GPIOC_PCLK_DI()					(RCC->APB2ENR &= ~(1U << 4))
#define GPIOD_PCLK_DI()					(RCC->APB2ENR &= ~(1U << 5))
#define GPIOE_PCLK_DI()					(RCC->APB2ENR &= ~(1U << 6))
#define GPIOF_PCLK_DI()					(RCC->APB2ENR &= ~(1U << 7))
#define GPIOG_PCLK_DI()					(RCC->APB2ENR &= ~(1U << 8))

//Clock disables for I2C/SPI peripherals
#define I2C1_PCLK_DI()					(RCC->APB1ENR &= ~(1U << 21))
#define I2C2_PCLK_DI()					(RCC->APB1ENR &= ~(1U << 22))
#define SPI1_PCLK_DI()					(RCC->APB2ENR &= ~(1U << 12))
#define SPI2_PCLK_DI()					(RCC->APB1ENR &= ~(1U << 14))
#define SPI3_PCLK_DI()					(RCC->APB1ENR &= ~(1U << 15))

//Clock disables for U(S)ART peripherals
#define USART1_PCLK_DI()				(RCC->APB2ENR &= ~(1U << 14))
#define USART2_PCLK_DI()				(RCC->APB1ENR &= ~(1U << 17))
#define USART3_PCLK_DI()				(RCC->APB1ENR &= ~(1U << 18))
#define UART4_PCLK_DI()					(RCC->APB1ENR &= ~(1U << 19))
#define UART5_PCLK_DI()					(RCC->APB1ENR &= ~(1U << 20))

// EXTI Line IRQ Number
#define IRQ_NO_EXTI0					6
#define IRQ_NO_EXTI1					7
#define IRQ_NO_EXTI2					8
#define IRQ_NO_EXTI3					9
#define IRQ_NO_EXTI4					10
#define IRQ_NO_EXTI9_5					23
#define IRQ_NO_EXTI15_10				40



#endif /* INC_STM32F103XX_H_ */
