/*
 * hseChecker.c
 *
 *  Created on: 17 feb. 2023 г.
 *      Author: ZmitrovichStanislau
 */

//INCLUDERS
#include "hseChecker.h"

//DEFINES
#define defaultHsiAccurancy 15
#define hsiPeriodCheckingStage 0
#define hsePeriodCheckingStage 1

//FUNCTIONS PROTOTYPES
static inline bool isOscilatorInWorkingOrder(__IO uint32_t* oscReg, uint32_t oscOnBitMsk, uint32_t oscRdyBitMsk);
static inline void rccReset(void);
static void hseCheckingTim5IRQHandler(void);
static void timerInit(void);

//VARIABLES
volatile uint32_t prevCCR = 0;
volatile uint32_t actCCR = 0;
volatile uint8_t periodCheckingStage = 0;
volatile bool hseCheckingFlag = 0;
volatile bool lsiPeriodCounted = 0;
volatile uint32_t hsiToLsiPeriod = 0;
volatile uint32_t hseToLsiPeriod = 0;

volatile oscilatorConfiguration_t HSE_16MHz_Ocliator = {.configDetailed = {.PLLSRC = PLLSRC_HSE,
																		.PLLM = 8,
																		.PLLN = 216,
																		.PLLP = PLLP_PRESCALER_4,
																		.PLLQ = 9,
																		.SW = RCC_CFGR_SW_PLL,
																		.HPRE =  AHB_PRESCALER_1,
																		.PPRE1 = APB_PRESCALER_4,
																		.PPRE2 = APB_PRESCALER_2,
																		.RTCPRE = RTC_HSE_NOT_CLOCKED
																		}};

volatile oscilatorConfiguration_t HSE_8MHz_Ocliator = {.configDetailed = {.PLLSRC = PLLSRC_HSE,
																		.PLLM = 4,
																		.PLLN = 216,
																		.PLLP = PLLP_PRESCALER_4,
																		.PLLQ = 9,
																		.SW = RCC_CFGR_SW_PLL,
																		.HPRE =  AHB_PRESCALER_1,
																		.PPRE1 = APB_PRESCALER_4,
																		.PPRE2 = APB_PRESCALER_2,
																		.RTCPRE = RTC_HSE_NOT_CLOCKED
																		}};

volatile oscilatorConfiguration_t HSE_4MHz_Ocliator = {.configDetailed = { .PLLSRC = PLLSRC_HSE,
																		.PLLM = 2,
																		.PLLN = 216,
																		.PLLP = PLLP_PRESCALER_4,
																		.PLLQ = 9,
																		.SW = RCC_CFGR_SW_PLL,
																		.HPRE =  AHB_PRESCALER_1,
																		.PPRE1 = APB_PRESCALER_4,
																		.PPRE2 = APB_PRESCALER_2,
																		.RTCPRE = RTC_HSE_NOT_CLOCKED
																		}};

volatile oscilatorConfiguration_t HSI_16MHz_Ocliator = {.configDetailed = {.PLLSRC = PLLSRC_HSI,
																		.PLLM = 8,
																		.PLLN = 216,
																		.PLLP = PLLP_PRESCALER_4,
																		.PLLQ = 9,
																		.SW = RCC_CFGR_SW_PLL,
																		.HPRE =  AHB_PRESCALER_1,
																		.PPRE1 = APB_PRESCALER_4,
																		.PPRE2 = APB_PRESCALER_2,
																		.RTCPRE = RTC_HSE_NOT_CLOCKED
																		}};

//EXTERNS
extern uint32_t SystemCoreClock;

//FUNCTIONS
static inline void rccReset(void){
	RCC->CFGR = (uint32_t)0x00;
	while(RCC->CFGR & RCC_CFGR_SWS);
	RCC->CR &= ~RCC_CR_PLLON;
	RCC->CR &= ~RCC_CR_HSEON;
	RCC->PLLCFGR = 0x24003010;
	RCC->CIR = (uint32_t)0x00;
	RCC->BDCR |= RCC_BDCR_BDRST;
	RCC->CSR &= ~RCC_CSR_LSION;
	RCC->CSR |= RCC_CSR_RMVF;
	RCC->PLLI2SCFGR = 0x20003000;
}

static inline bool isOscilatorInWorkingOrder(__IO uint32_t* oscReg, uint32_t oscOnBitMsk, uint32_t oscRdyBitMsk){
	*oscReg |= oscOnBitMsk;

	for (uint32_t i = 0; i < 10000; i++){
		if(*oscReg & oscRdyBitMsk){
			return true;
		}
	}

	*oscReg &= ~oscOnBitMsk;
	return false;
}

static void timerInit(void){

	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // TIM5 clocking enable

	TIM5->CCMR2 = (TIM5->CCMR2 & (~TIM_CCMR2_CC4S)) | TIM_CCMR2_CC4S_0; // CC4 channel is configured as input, IC4 is mapped on TI4
	TIM5->CCMR2 &= ~TIM_CCMR2_IC4F; // input capture filter = 0
	TIM5->CCMR2 &= ~TIM_CCMR2_IC4PSC; // input capture prescaler = 0

	TIM5->OR = TIM_OR_TI4_RMP_0; //the LSI internal clock is connected to the TIM5_CH4 input for calibration purposes

	TIM5->CCER &= 	~(TIM_CCER_CC4NP | TIM_CCER_CC4P);   // noninverted/rising edge
	TIM5->CCER |= TIM_CCER_CC4E; // Capture enabling
	TIM5->CR1 |= TIM_CR1_CEN; // Enabling counting

	TIM5->ARR = 0xFFFFFFFF; //timer counting size

	NVIC_EnableIRQ(TIM5_IRQn); //enabling global interrupt
}


static uint8_t checkHseFreq(void){
	uint16_t hsePeriodPlusDefAccur = (hseToLsiPeriod + (hseToLsiPeriod / 100 * defaultHsiAccurancy));
	uint16_t hsePeriodMinusDefAccur = (hseToLsiPeriod - (hseToLsiPeriod / 100 * defaultHsiAccurancy));

	if((hsiToLsiPeriod <= hsePeriodPlusDefAccur) && (hsiToLsiPeriod >= hsePeriodMinusDefAccur)){
		return HSE_FREQ_MHZ_16;
	} else if (((hsiToLsiPeriod / 2) <= hsePeriodPlusDefAccur) && ((hsiToLsiPeriod / 2) >= hsePeriodMinusDefAccur)) {
		return HSE_FREQ_MHZ_8;
	} else if (((hsiToLsiPeriod / 4) <= hsePeriodPlusDefAccur) && ((hsiToLsiPeriod / 4) >= hsePeriodMinusDefAccur)) {
		return HSE_FREQ_MHZ_4;
	} else {
		return HSE_FREQ_ISNT_RECOGNIZED;
	}
}

uint8_t getHseFreq(void){

	hseCheckingFlag = true;

	if(!isOscilatorInWorkingOrder(HSE_OSCILATOR)){
		return HSE_ISNT_ENABLE;
	}

	rccReset();

	isOscilatorInWorkingOrder(HSI_OSCILATOR);
	isOscilatorInWorkingOrder(LSI_OSCILATOR);

	timerInit();
	lsiPeriodCounted = false;
	periodCheckingStage = hsiPeriodCheckingStage;

	TIM5->DIER |= TIM_DIER_CC4IE; //Update interrupt enable

	while(!lsiPeriodCounted){};

	if(!isOscilatorInWorkingOrder(HSE_OSCILATOR)){
		return HSE_ISNT_ENABLE;
	}

	RCC->CFGR |= RCC_CFGR_SW_HSE; // main clock switching to HSE
	while ((RCC->CFGR & RCC_CFGR_SWS_HSE) != RCC_CFGR_SWS_HSE) {}; //waiting HSE stabilization

	lsiPeriodCounted = false;
	periodCheckingStage = hsePeriodCheckingStage;
	TIM5->DIER |= TIM_DIER_CC4IE; //Update interrupt enable

	while(!lsiPeriodCounted){};

	hseCheckingFlag = false;

	RCC->APB1RSTR = RCC_APB1RSTR_TIM5RST; //TIM5 to reset state

	return checkHseFreq();
}

void rccInit(volatile uint8_t oscilatorType){

	oscilatorConfiguration_t oscilatorConfiruration;


	switch(oscilatorType){
	case (HSE_FREQ_MHZ_16):
		oscilatorConfiruration = HSE_16MHz_Ocliator;
		break;

	case (HSE_FREQ_MHZ_8):
		oscilatorConfiruration = HSE_8MHz_Ocliator;
		break;

	case (HSE_FREQ_MHZ_4):
		oscilatorConfiruration = HSE_4MHz_Ocliator;
		break;

	case (HSE_FREQ_ISNT_RECOGNIZED):
	case (HSE_ISNT_ENABLE):
	default:
		RCC->CFGR &= ~RCC_CFGR_SW_HSI; // main clock reset and switching to HSI
		while ((RCC->CFGR & RCC_CFGR_SWS_HSI) != RCC_CFGR_SWS_HSI) {}; //waiting HSI stabilization
		RCC->CR &= ~RCC_CR_HSEON; // HSE oscillator OFF
		oscilatorConfiruration = HSI_16MHz_Ocliator;
		break;
	}

	RCC->PLLCFGR = oscilatorConfiruration.config[0];

	FLASH->ACR = FLASH_ACR_LATENCY_4WS; //Setting latency

	RCC->CFGR = oscilatorConfiruration.config[1];

	RCC->CR |= RCC_CR_PLLON; //Enabling PLL
	while (!(RCC->CR & RCC_CR_PLLRDY)) {}; //waiting PLL stabilization

	RCC->CFGR &= ~RCC_CFGR_SW_HSI; // main clock reset
	RCC->CFGR |= RCC_CFGR_SW_PLL; // main clock switching to PLL
	while ((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL) {}; //waiting PLL stabilization

}

static void hseCheckingTim5IRQHandler(void){

	if(hseCheckingFlag){

		static uint8_t interruptsCounter = 0;
		interruptsCounter++;
		actCCR = TIM5->CCR4;

		if(interruptsCounter == 100){
			if(periodCheckingStage == hsiPeriodCheckingStage){
				hsiToLsiPeriod = actCCR - prevCCR;
			} else {
				hseToLsiPeriod = actCCR - prevCCR;
			}
			lsiPeriodCounted = true;
			TIM5->DIER &= ~TIM_DIER_CC4IE;
			interruptsCounter = 0;
		}

		prevCCR = actCCR;

	}
}

void TIM5_IRQHandler(){
	hseCheckingTim5IRQHandler();
}

