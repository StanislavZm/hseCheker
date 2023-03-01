/*
 * hseChecker.h
 *
 *  Created on: 17 feb. 2023 г.
 *      Author: ZmitrovichStanislau
 */

#ifndef HSECHECKER_H_
#define HSECHECKER_H_

//INCLUDERS
#include "stm32f2xx.h"
#include "stm32f2xx_hal_rcc.h"
#include "stdint.h"
#include "stdbool.h"

//DEFINES
#define HSE_OSCILATOR &(RCC->CR), RCC_CR_HSEON, RCC_CR_HSERDY
#define HSI_OSCILATOR &(RCC->CR), RCC_CR_HSION, RCC_CR_HSIRDY
#define LSI_OSCILATOR &(RCC->CSR), RCC_CSR_LSION, RCC_CSR_LSIRDY

#define PLLSRC_HSE 0b1
#define PLLSRC_HSI 0b0

#define PLLP_PRESCALER_2 (uint32_t)0b00
#define PLLP_PRESCALER_4 (uint32_t)0b01
#define PLLP_PRESCALER_6 (uint32_t)0b10
#define PLLP_PRESCALER_8 (uint32_t)0b11

#define AHB_PRESCALER_1 (uint32_t)0b0000
#define AHB_PRESCALER_2 (uint32_t)0b1000
#define AHB_PRESCALER_4 (uint32_t)0b1001
#define AHB_PRESCALER_8 (uint32_t)0b1010
#define AHB_PRESCALER_16 (uint32_t)0b1011
#define AHB_PRESCALER_64 (uint32_t)0b1100
#define AHB_PRESCALER_128 (uint32_t)0b1101
#define AHB_PRESCALER_256 (uint32_t)0b1110
#define AHB_PRESCALER_512 (uint32_t)0b1111

#define APB_PRESCALER_1 (uint32_t)0b001
#define APB_PRESCALER_2 (uint32_t)0b100
#define APB_PRESCALER_4 (uint32_t)0b101
#define APB_PRESCALER_8 (uint32_t)0b110
#define APB_PRESCALER_16 (uint32_t)0b111

#define RTC_HSE_NOT_CLOCKED 0
#define RTC_HSE_PRESCALER_2 2
#define RTC_HSE_PRESCALER_3 3
#define RTC_HSE_PRESCALER_4 4
#define RTC_HSE_PRESCALER_5 5
#define RTC_HSE_PRESCALER_6 6
#define RTC_HSE_PRESCALER_7 7
#define RTC_HSE_PRESCALER_8 8
#define RTC_HSE_PRESCALER_9 9
#define RTC_HSE_PRESCALER_10 10
#define RTC_HSE_PRESCALER_11 11
#define RTC_HSE_PRESCALER_12 12
#define RTC_HSE_PRESCALER_13 13
#define RTC_HSE_PRESCALER_14 14
#define RTC_HSE_PRESCALER_15 15
#define RTC_HSE_PRESCALER_16 16
#define RTC_HSE_PRESCALER_17 17
#define RTC_HSE_PRESCALER_18 18
#define RTC_HSE_PRESCALER_19 19
#define RTC_HSE_PRESCALER_20 20
#define RTC_HSE_PRESCALER_21 21
#define RTC_HSE_PRESCALER_22 22
#define RTC_HSE_PRESCALER_23 23
#define RTC_HSE_PRESCALER_24 24
#define RTC_HSE_PRESCALER_25 25
#define RTC_HSE_PRESCALER_26 26
#define RTC_HSE_PRESCALER_27 27
#define RTC_HSE_PRESCALER_28 28
#define RTC_HSE_PRESCALER_29 29
#define RTC_HSE_PRESCALER_30 30
#define RTC_HSE_PRESCALER_31 31

//VARIABLES
enum hseFrequencies_t{
	HSE_ISNT_ENABLE = 0,
	HSE_FREQ_MHZ_4,
	HSE_FREQ_MHZ_8,
	HSE_FREQ_MHZ_16,
	HSE_FREQ_ISNT_RECOGNIZED,
};

typedef struct MulDivConfiguration_t {
	// PLL configuration
	uint32_t PLLM : 6;
	uint32_t PLLN : 9;
	uint32_t RESERVED_BITS_1 : 1;
	uint32_t PLLP : 2;
	uint32_t RESERVED_BITS_2 : 4;
	uint32_t PLLSRC : 1;
	uint32_t RESERVED_BITS_3 : 1;
	uint32_t PLLQ : 4;
	uint32_t RESERVED_BITS_4 : 4;

	//Prescellers configuration
	uint32_t SW : 2; //System clock switch
	uint32_t SWS : 2; //System clock switch status
	uint32_t HPRE : 4; //AHB prescaler
	uint32_t RESERVED_BITS_5 : 2;
	uint32_t PPRE1 : 3; //APB Low speed prescaler (APB1)
	uint32_t PPRE2 : 3; //APB Low speed prescaler (APB2)
	uint32_t RTCPRE : 5; //HSE division factor for RTC clock
	uint32_t MCO1 : 2; //Microcontroller clock output 1
	uint32_t I2SSCR : 1; //I2S clock selection
	uint32_t MCO1_PRE : 3; //MCO1 prescaler
	uint32_t MCO2_PRE : 3; //MCO2 prescaler
	uint32_t MCO2 : 2; //Microcontroller clock output 2
} MulDivConfiguration_t;

typedef union oscilatorConfiguration_t {
	volatile uint32_t config[2];
	volatile MulDivConfiguration_t configDetailed;
}  oscilatorConfiguration_t;

//EXTERNS

//FUNCTIONS PROTOTYPES
uint8_t getHseFreq(void);
void rccInit(volatile uint8_t oscilatorType);

#endif /* HSECHECKER_H_ */
