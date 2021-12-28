/*
 * stm32f446re.h
 *
 *  Created on: 17.08.2021
 *  Author: Maximilian Altrichter
 */
#ifndef DRIVERS_INC_STM32F446RE_H_
#define DRIVERS_INC_STM32F446RE_H_
/*
 * base addresses of Flash and SRAM memories
 */


#include <stdint.h>


#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET	RESET

#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO: Complete for all other peripherals
 */

#define GPIOA_BASEADDR			AHB1PERIPH_BASEADDR
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400UL)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800UL)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00UL)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000UL)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400UL)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800UL)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00UL)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800UL)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO: Complete for all other peripherals
 */

#define TIM1_BASEADDR			APB2PERIPH_BASEADDR
#define TIM8_BASEADDR			(APB2PERIPH_BASEADDR + 0x0400UL)
#define TIM9_BASEADDR			(APB2PERIPH_BASEADDR + 0x4000UL)
#define TIM10_BASEADDR			(APB2PERIPH_BASEADDR + 0x4400UL)
#define TIM11_BASEADDR			(APB2PERIPH_BASEADDR + 0x4800UL)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO: Complete for all other peripherals
 */

#define TIM2_BASEADDR			APB1PERIPH_BASEADDR
#define TIM3_BASEADDR			(APB1PERIPH_BASEADDR + 0x0400UL)
#define TIM4_BASEADDR			(APB1PERIPH_BASEADDR + 0x0800UL)
#define TIM5_BASEADDR			(APB1PERIPH_BASEADDR + 0x0C00UL)
#define TIM6_BASEADDR			(APB1PERIPH_BASEADDR + 0x1000UL)
#define TIM7_BASEADDR			(APB1PERIPH_BASEADDR + 0x1400UL)
#define TIM12_BASEADDR			(APB1PERIPH_BASEADDR + 0x1800UL)
#define TIM13_BASEADDR			(APB1PERIPH_BASEADDR + 0x1C00UL)
#define TIM14_BASEADDR			(APB1PERIPH_BASEADDR + 0x2000UL)

/******peripheral register definition structures******/

/*
 * GPIO
 */

typedef struct
{
	volatile uint32_t MODER;	/*!< GPIO port mode register     					Address offset : 0x00*/
	volatile uint32_t OTYPER;	/*!< GPIO port output type register     			Address offset : 0x04*/
	volatile uint32_t OSPEEDR;	/*!< GPIO port output speed register     			Address offset : 0x08*/
	volatile uint32_t PUPDR;	/*!< GPIO port pull-up/pull-down register     		Address offset : 0x0C*/
	volatile uint32_t IDR;		/*!< GPIO port input data register			    	Address offset : 0x10*/
	volatile uint32_t ODR;		/*!< GPIO port output data register		    		Address offset : 0x14*/
	volatile uint32_t BSRR;		/*!< GPIO port bit set/reset register     			Address offset : 0x18*/
	volatile uint32_t LCKR;		/*!< GPIO port configuration lock register	    	Address offset : 0x1C*/
	volatile uint32_t AFRL;		/*!< GPIO port alternate function low register    	Address offset : 0x20*/
	volatile uint32_t AFRH;		/*!< GPIO port alternate function high register   	Address offset : 0x24*/
}GPIO_RegDef_t;

/******peripheral register definition structures******/

/*
 * Timer Tim10/11/13/14
 */

typedef struct
{
	volatile uint32_t CR1;						/* control register 1						Address offset : 0x00*/
	volatile uint32_t CR1_reset;				/* control register 1 reset										 */
	volatile uint32_t SMCR;						/* SMCR										Address offset : 0x08*/
	volatile uint32_t SMCR_reset;				/* SMCR reset								  					 */
	volatile uint32_t DIER;						/* Interrupt enable register				Address offset : 0x0C*/
	volatile uint32_t DIER_reset;				/* Interrupt enable register reset								 */
	volatile uint32_t SR;						/* status register							Address offset : 0x10*/
	volatile uint32_t SR_reset;					/* status register status										 */
	volatile uint32_t EGR;						/* event generation register				Address offset : 0x14*/
	volatile uint32_t EGR_reset;				/* event generation register reset				  				 */
	volatile uint32_t CCMR1_output;				/* capture/compare mode register 1			Address offset : 0x18*/
	volatile uint32_t CCMR1_output_reset;		/* capture/compare mode register 1 reset						 */
	volatile uint32_t CCMR1_input;				/* capture/compare mode register 1								 */
	volatile uint32_t CCMR1_input_reset;		/* capture/compare mode register 1 reset						 */
	volatile uint32_t RESERVED_1;				/* reserved														 */
	volatile uint32_t CCER;						/* capture/compare enable register			Address offset : 0x20*/
	volatile uint32_t CCER_reset;				/* capture/compare enable register reset						 */
	volatile uint32_t CNT;						/* counter									Address offset : 0x24*/
	volatile uint32_t CNT_reset;				/* counter reset												 */
	volatile uint32_t PSC;						/* prescaler								Address offset : 0x28*/
	volatile uint32_t PSC_reset;				/* prescaler reset												 */
	volatile uint32_t ARR;						/* auto-reload register						Address offset : 0x2C*/
	volatile uint32_t ARR_reset;				/* auto-reload register reset									 */
	volatile uint32_t RESERVED_2;				/* reserved														 */
	volatile uint32_t CCR1;						/* capture/compare register 1				Address offset : 0x34*/
	volatile uint32_t CCR1_reset;				/* capture/compare register 1 reset								 */
	volatile uint32_t RESERVED_3;				/* reserved														 */
	volatile uint32_t OR;						/* option register							Address offset : 0x50*/
	volatile uint32_t OR_reset;					/* option register reset										 */
}TIM_10_11_12_14_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */

typedef struct
{
	volatile uint32_t CR;			/*!< RCC clock control register										Address offset : 0x00*/
	volatile uint32_t PLLCFGR;  	/*!< RCC PLL configuration register									Address offset : 0x04*/
	volatile uint32_t CFGR;			/*!< RCC clock configuration register								Address offset : 0x08*/
	volatile uint32_t CIR;			/*!< RCC clock interrupt register									Address offset : 0x0C*/
	volatile uint32_t AHB1RSTR; 	/*!< RCC AHB1 peripheral reset register								Address offset : 0x10*/
	volatile uint32_t AHB2RSTR;		/*!< RCC AHB2 peripheral reset register								Address offset : 0x14*/
	volatile uint32_t AHB3RSTR;		/*!< RCC AHB3 peripheral reset register								Address offset : 0x18*/
	volatile uint32_t RESERVED1;	/*!< RCC Reserved													Address offset : 0x1C*/
	volatile uint32_t APB1RSTR;		/*!< RCC APB1 peripheral reset register								Address offset : 0x20*/
	volatile uint32_t APB2RSTR;		/*!< RCC APB2 peripheral reset register								Address offset : 0x24*/
	volatile uint32_t RESERVED2;	/*!< RCC Reserved													Address offset : 0x28*/
	volatile uint32_t RESERVED3;	/*!< RCC Reserved													Address offset : 0x2C*/
	volatile uint32_t AHB1ENR;		/*!< RCC AHB1 peripheral clock enable register						Address offset : 0x30*/
	volatile uint32_t AHB2ENR;		/*!< RCC AHB2 peripheral clock enable register						Address offset : 0x34*/
	volatile uint32_t AHB3ENR;		/*!< RCC AHB3 peripheral clock enable register						Address offset : 0x38*/
	volatile uint32_t RESERVED4;	/*!< RCC Reserved													Address offset : 0x3C*/
	volatile uint32_t APB1ENR;		/*!< RCC APB1 peripheral clock enable register						Address offset : 0x40*/
	volatile uint32_t APB2ENR;		/*!< RCC APB2 peripheral clock enable register						Address offset : 0x44*/
	volatile uint32_t RESERVED5;	/*!< RCC Reserved													Address offset : 0x48*/
	volatile uint32_t RESERVED6;	/*!< RCC Reserved													Address offset : 0x4C*/
	volatile uint32_t AHB1LPENR;	/*!< RCC AHB1 peripheral clock enable in low power mode register	Address offset : 0x50*/
	volatile uint32_t AHB2LPENR;	/*!< RCC AHB2 peripheral clock enable in low power mode register	Address offset : 0x54*/
	volatile uint32_t AHB3LPENR;	/*!< RCC AHB3 peripheral clock enable in low power mode register	Address offset : 0x58*/
	volatile uint32_t RESERVED7;	/*!< RCC Reserved													Address offset : 0x5C*/
	volatile uint32_t APB1LPENR;	/*!< RCC APB1 peripheral clock enable in low power mode register	Address offset : 0x60*/
	volatile uint32_t APB2LPENR;	/*!< RCC APB2 peripheral clock enable in low power mode register	Address offset : 0x64*/
	volatile uint32_t RESERVED8;	/*!< RCC Reserved													Address offset : 0x68*/
	volatile uint32_t RESERVED9;	/*!< RCC Reserved													Address offset : 0x6C*/
	volatile uint32_t BDCR;			/*!< RCC Backup domain control register								Address offset : 0x70*/
	volatile uint32_t CSR;			/*!< RCC clock control & status register							Address offset : 0x74*/
	volatile uint32_t RESERVED10;	/*!< RCC Reserved													Address offset : 0x78*/
	volatile uint32_t RESERVE11;	/*!< RCC Reserved													Address offset : 0x7C*/
	volatile uint32_t SSCGR;		/*!< RCC spread spectrum clock generation register					Address offset : 0x80*/
	volatile uint32_t PLLI2SCFGR;	/*!< RCC PLLI2S configuration register								Address offset : 0x84*/
	volatile uint32_t PLLSAICFGR;	/*!< RCC PLL configuration register									Address offset : 0x88*/
	volatile uint32_t DCKCFGR;		/*!< RCC dedicated clock configuration register						Address offset : 0x8C*/
	volatile uint32_t CKGATENR;		/*!< RCC clocks gated enable register								Address offset : 0x90*/
	volatile uint32_t DCKCFGR2;		/*!< RCC dedicated clocks configuration register 2					Address offset : 0x94*/
	//TODO : Fill out by DENRUP
}RCC_RegDef_t;

/*
 * peripheral definitions (Peripheral bas addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * Timer
 */

#define TIM1			((TIMER_10_11_12_14_RegDef_t*)TIM1_BASEADDR)
#define TIM2			((TIMER_10_11_12_14_RegDef_t*)TIM2_BASEADDR)
#define TIM3			((TIMER_10_11_12_14_RegDef_t*)TIM3_BASEADDR)
#define TIM4			((TIMER_10_11_12_14_RegDef_t*)TIM4_BASEADDR)
#define TIM5			((TIMER_10_11_12_14_RegDef_t*)TIM5_BASEADDR)
#define TIM6			((TIMER_10_11_12_14_RegDef_t*)TIM6_BASEADDR)
#define TIM7			((TIMER_10_11_12_14_RegDef_t*)TIM7_BASEADDR)
#define TIM8			((TIMER_10_11_12_14_RegDef_t*)TIM8_BASEADDR)
#define TIM9			((TIMER_10_11_12_14_RegDef_t*)TIM9_BASEADDR)
#define TIM10			((TIMER_10_11_12_14_RegDef_t*)TIM10_BASEADDR)
#define TIM11			((TIMER_10_11_12_14_RegDef_t*)TIM11_BASEADDR)
#define TIM12			((TIMER_10_11_12_14_RegDef_t*)TIM12_BASEADDR)
#define TIM13			((TIMER_10_11_12_14_RegDef_t*)TIM13_BASEADDR)
#define TIM14			((TIMER_10_11_12_14_RegDef_t*)TIM14_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (0x1UL << 0U))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (0x1UL << 1U))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (0x1UL << 2U))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (0x1UL << 3U))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (0x1UL << 4U))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= (0x1UL << 5U))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= (0x1UL << 6U))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (0x1UL << 7U))

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~(0x1UL << 0U))
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~(0x1UL << 1U))
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~(0x1UL << 2U))
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~(0x1UL << 3U))
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~(0x1UL << 4U))
#define GPIOF_PCLK_DI()	(RCC->AHB1ENR &= ~(0x1UL << 5U))
#define GPIOG_PCLK_DI()	(RCC->AHB1ENR &= ~(0x1UL << 6U))
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~(0x1UL << 7U))

/*
 * Clock Enable Macros for Timer peripherals
 */

#define TIM1_EN()		(RCC->APB2ENR |= (0x1UL << 0U))
#define TIM2_EN()		(RCC->APB1ENR |= (0x1UL << 0U))
#define TIM3_EN()		(RCC->APB1ENR |= (0x1UL << 1U))
#define TIM4_EN()		(RCC->APB1ENR |= (0x1UL << 2U))
#define TIM5_EN()		(RCC->APB1ENR |= (0x1UL << 3U))
#define TIM6_EN()		(RCC->APB1ENR |= (0x1UL << 4U))
#define TIM7_EN()		(RCC->APB1ENR |= (0x1UL << 5U))
#define TIM8_EN()		(RCC->APB2ENR |= (0x1UL << 1U))
#define TIM9_EN()		(RCC->APB2ENR |= (0x1UL << 16U))
#define TIM10_EN()		(RCC->APB2ENR |= (0x1UL << 17U))
#define TIM11_EN()		(RCC->APB2ENR |= (0x1UL << 18U))
#define TIM12_EN()		(RCC->APB1ENR |= (0x1UL << 6U))
#define TIM13_EN()		(RCC->APB1ENR |= (0x1UL << 7U))
#define TIM14_EN()		(RCC->APB1ENR |= (0x1UL << 8U))

/*
 * Clock Disable Macros for Timer peripherals
 */

#define TIM1_DI()		(RCC->APB2ENR &= ~(0x1UL << 0U))
#define TIM2_DI()		(RCC->APB1ENR &= ~(0x1UL << 0U))
#define TIM3_DI()		(RCC->APB1ENR &= ~(0x1UL << 1U))
#define TIM4_DI()		(RCC->APB1ENR &= ~(0x1UL << 2U))
#define TIM5_DI()		(RCC->APB1ENR &= ~(0x1UL << 3U))
#define TIM6_DI()		(RCC->APB1ENR &= ~(0x1UL << 4U))
#define TIM7_DI()		(RCC->APB1ENR &= ~(0x1UL << 5U))
#define TIM8_DI()		(RCC->APB2ENR &= ~(0x1UL << 1U))
#define TIM9_DI()		(RCC->APB2ENR &= ~(0x1UL << 16U))
#define TIM10_DI()		(RCC->APB2ENR &= ~(0x1UL << 17U))
#define TIM11_DI()		(RCC->APB2ENR &= ~(0x1UL << 18U))
#define TIM12_DI()		(RCC->APB1ENR &= ~(0x1UL << 6U))
#define TIM13_DI()		(RCC->APB1ENR &= ~(0x1UL << 7U))
#define TIM14_DI()		(RCC->APB1ENR &= ~(0x1UL << 8U))

#include "GPIO.h"

#endif
