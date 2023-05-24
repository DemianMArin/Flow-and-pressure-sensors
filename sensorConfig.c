/*
 * sensorConfig.c
 *
 *  Created on: May 2, 2023
 *      Author: demian
 */

#include "sensorConfig.h"
#include "main.h"
//--------------USART 2 & 3
void USER_USART2_RCC_Init(void){
	RCC->APB1ENR	|=	 RCC_APB1ENR_USART2EN; //USART2 clock enable
}
void USER_USART2_GPIO_Init(void){
	//pin PA2 (USART2_TX) as alternate function output push-pull, max speed 10MHz
	GPIOA->CRL	&=	~GPIO_CRL_CNF2_0 & ~GPIO_CRL_MODE2_1;
	GPIOA->CRL	|=	 GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2_0;
}
void USER_USART2_Init(void){
	USART2->CR1	|=	 USART_CR1_UE;//		USART enabled
	USART2->CR1	&=	~USART_CR1_M//		  	1 start bit, 8 data bits
			&	~USART_CR1_PCE;//		parity control disabled
	USART2->CR2	&=	~USART_CR2_STOP;//  		1 stop bit
	USART2->BRR	 =	 0x115;//			115200 bps -> 17.361 || tt9600 bps -> 208.33,
	USART2->CR1	|=	 USART_CR1_TE;//		        transmitter enabled

	USART2->CR1 |= USART_CR1_RE; //                 receiver enabled
}

void USER_USART3_RCC_Init(void){
	RCC->APB1ENR	|=	 RCC_APB1ENR_USART3EN;//  	USART3 clock enable

}
void USER_USART3_GPIO_Init(void){
	//pin PB10 (USART3_TX) as alternate function output push-pull, max speed 10MHz
	GPIOB->CRH	&=	~GPIO_CRH_CNF10_0 & ~GPIO_CRH_MODE10_1;
	GPIOB->CRH	|=	 GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_0;
}
void USER_USART3_Init(void){
	//USART3
	USART3->CR1	|=	 USART_CR1_UE;//		USART enabled
	USART3->CR1	&=	~USART_CR1_M//		  	1 start bit, 8 data bits
			&	~USART_CR1_PCE;//		parity control disabled
	USART3->CR2	&=	~USART_CR2_STOP;//  		1 stop bit
	USART3->BRR	 =	 0xD05;//			tt9600 bps -> 208.33,
	USART3->CR1	|=	 USART_CR1_TE;//		        transmitter enabled
}
void USER_USART3_Transmit(uint8_t *pData, uint16_t size ){
	for( int i = 0; i < size; i++ ){
		while( ( USART3->SR & USART_SR_TXE ) == 0 ){}//	wait until transmit reg is empty
		USART3->DR = *pData++;//			transmit data
	}
}


//--------------Time Modules
void USER_TIMER2_RCC_Init(void){
	RCC->APB1ENR	|=	 RCC_APB1ENR_TIM2EN; //TIMER2 clock enable
}
void USER_TIMER3_RCC_Init(void){
	RCC->APB1ENR	|=	 RCC_APB1ENR_TIM3EN; //TIMER3 clock enable
}
void USER_TIMER2_CAPTUREMODE_GPIO_Init(void){
	//PA0 (TIM2_CH1) as input floating
	GPIOA->CRL	&=	~GPIO_CRL_CNF0_1 & ~GPIO_CRL_MODE0;
	GPIOA->CRL	|=	 GPIO_CRL_CNF0_0;
}
void USER_TIMER3_TIMER_Init(void){
	//For 1000ms
	TIM3->SMCR &= ~TIM_SMCR_SMS; //Enable clock structure
	TIM3->CR1 &= ~TIM_CR1_UDIS & ~TIM_CR1_DIR & ~TIM_CR1_CMS; //Up, overflow, edge-aligned
	TIM3->SR &= ~TIM_SR_UIF; //Update Interrupt Flag
	TIM3->CNT = 29; //Initial Count
	TIM3->PSC = 976; //Prescaler
	TIM3->CR1 |= TIM_CR1_CEN;
	while( !(TIM3->SR & TIM_SR_UIF) ){};
	TIM3->CR1 &= ~TIM_CR1_CEN;
}
void USER_TIMER2_Capture_Init(void){
	TIM2->CR1	&=	~TIM_CR1_CKD_0;
	TIM2->CR1	|=	 TIM_CR1_CKD_1;//	sampling (DTS) = TIM_CLK/4
	TIM2->CCMR1 	&=	~TIM_CCMR1_CC1S_1;
	TIM2->CCMR1 	|=	 TIM_CCMR1_CC1S_0;//	CC1 channel as input, mapped on TI1
	TIM2->CCMR1 	|=	 TIM_CCMR1_IC1F;//	filter -> DTS/32, N=8
	TIM2->CCER	|=	 TIM_CCER_CC1P;//	capture is done on falling edge
	TIM2->CCMR1 	&=	~TIM_CCMR1_IC1PSC;//	no prescaler
	TIM2->CCER	|=	 TIM_CCER_CC1E;//	capture enabled
	//TIM2->PSC	 =	 65535;//		maximum prescaler
	TIM2->PSC = 97;
	TIM2->CR1	|=	 TIM_CR1_CEN;//		counter enabled
}
uint16_t USER_TIMER2_Capture_Event(void){
	while( !(TIM2->SR & TIM_SR_CC1IF) );//		wait until a capture occurrs
	return TIM2->CCR1;//				return the captured value
}

//--------------ADC
void USER_ADC_RCC_Init(void){
	RCC->APB2ENR |=	 RCC_APB2ENR_ADC1EN;//	ADC 1 clock enable
	RCC->CFGR |=	 RCC_CFGR_ADCPRE;//	ADC prescaler 1:8 for 8 MHz
}
void USER_ADC_GPIO_Init(void){
	//PC0 (ADC12_IN10) as analog
	GPIOC->CRL	&=	~GPIO_CRL_CNF0 & ~GPIO_CRL_MODE0;
}
void USER_ADC_Init(void){
	ADC1->CR1	&=	~ADC_CR1_DUALMOD;//	independent mode
	ADC1->CR2	&=	~ADC_CR2_ALIGN;//	right alignment for the result
	ADC1->CR2	|=	 ADC_CR2_CONT;//	continuous conversion mode
	ADC1->SMPR2	&=	~ADC_SMPR1_SMP10;//	1.5 cycles channel sample time
	ADC1->SQR1	&=	~ADC_SQR1_L;//		1 conversion on regular channels
	ADC1->SQR3 	&=	~ADC_SQR3_SQ1_4//	first and only conversion in Ch10
				& ~ADC_SQR3_SQ1_2
				& ~ADC_SQR3_SQ1_0;
	ADC1->SQR3 	|=	ADC_SQR3_SQ1_1
				| ADC_SQR3_SQ1_3;
	ADC1->CR2	|=	 ADC_CR2_ADON;//	ADC enabled
	HAL_Delay(1);//					tstab(1us) after ADC enabled, real 1ms
}
void USER_ADC_Calibration(void){
	ADC1->CR2	|=	 ADC_CR2_CAL;//		start calibration
	while( ADC1->CR2 & ADC_CR2_CAL );//		wait until calibration is done
}
uint16_t USER_ADC_Read(void){
	while( !( ADC1->SR & ADC_SR_EOC ) );//		wait until conversion is done
	return (uint16_t)ADC1->DR;//			return ADC data
}

//--------------Matrix Keypad
void USER_MATRIXKEYPAD_GPIO_Init(void){
	   //PA10 as input pull-up -horizontal
		GPIOA->CRH &= ~GPIO_CRH_MODE10 & ~GPIO_CRH_CNF10_0;
		GPIOA->CRH |= GPIO_CRH_CNF10_1;
		GPIOA->ODR |= GPIO_ODR_ODR10;
		//PA6 as input pull-up -horizontal
		GPIOA->CRL &= ~GPIO_CRL_MODE6 & ~GPIO_CRL_CNF6_0;
		GPIOA->CRL |= GPIO_CRL_CNF6_1;
		GPIOA->ODR |= GPIO_ODR_ODR6;
		//PA11 as input pull-up -horizontal
		GPIOA->CRH &= ~GPIO_CRH_MODE11 & ~GPIO_CRH_CNF11_0;
		GPIOA->CRH |= GPIO_CRH_CNF11_1;
		GPIOA->ODR |= GPIO_ODR_ODR11;
		//PA7 as input pull-up -horizontal
		GPIOA->CRL &= ~GPIO_CRL_MODE7 & ~GPIO_CRL_CNF7_0;
		GPIOA->CRL |= GPIO_CRL_CNF7_1;
		GPIOA->ODR |= GPIO_ODR_ODR7;

		//PC8 as output push-pull -vertical
		GPIOC->BSRR = GPIO_BSRR_BR8;
		GPIOC->CRH &= ~GPIO_CRH_MODE8_1 & ~GPIO_CRH_CNF8;
		GPIOC->CRH |= GPIO_CRH_MODE8_0;
		//PC6 as output push-pull -vertical
		GPIOC->BSRR = GPIO_BSRR_BS6;
		GPIOC->CRL &= ~GPIO_CRL_MODE6_1 & ~GPIO_CRL_CNF6;
		GPIOC->CRL |= GPIO_CRL_MODE6_0;
		//PC5 as output push-pull -vertical
		GPIOC->BSRR = GPIO_BSRR_BS5;
		GPIOC->CRL &= ~GPIO_CRL_MODE5_1 & ~GPIO_CRL_CNF5;
		GPIOC->CRL |= GPIO_CRL_MODE5_0;
		//PC9 as output push-pull -vertical
		GPIOC->BSRR = GPIO_BSRR_BS9;
		GPIOC->CRH &= ~GPIO_CRH_MODE9_1 & ~GPIO_CRH_CNF9;
		GPIOC->CRH |= GPIO_CRH_MODE9_0;
}

int USER_MXKeyboard_SelectKey(void){
	//First Column
	GPIOC->ODR &= ~GPIO_ODR_ODR8;
	GPIOC->ODR |= GPIO_ODR_ODR6;
	GPIOC->ODR |= GPIO_ODR_ODR5;
	GPIOC->ODR |= GPIO_ODR_ODR9;
	  if(!(GPIOA->IDR & GPIO_IDR_IDR10)){ //4
		  GPIOA->ODR ^= GPIO_ODR_ODR5;
		  return 4;
		  while(!(GPIOA->IDR & GPIO_IDR_IDR10)){}
		  HAL_Delay(10);
	  }
//	if(!(GPIOA->IDR & GPIO_IDR_IDR6)){ //7
//		  GPIOA->ODR ^= GPIO_ODR_ODR5;
//		  return 7;
//		  while(!(GPIOA->IDR & GPIO_IDR_IDR6)){}
//		  HAL_Delay(10);
//	  }
//	if(!(GPIOA->IDR & GPIO_IDR_IDR11)){ //* delete
//		  GPIOA->ODR ^= GPIO_ODR_ODR5;
//		  return 10;
//		  while(!(GPIOA->IDR & GPIO_IDR_IDR11)){}
//		  HAL_Delay(10);
//	  }
	if(!(GPIOA->IDR & GPIO_IDR_IDR7)){ //1
		  GPIOA->ODR ^= GPIO_ODR_ODR5;
		  return 1;
		  while(!(GPIOA->IDR & GPIO_IDR_IDR7)){}
		  HAL_Delay(10);
	  }

	//Second Column
	GPIOC->ODR |= GPIO_ODR_ODR8;
	GPIOC->ODR &= ~GPIO_ODR_ODR6;
	GPIOC->ODR |= GPIO_ODR_ODR5;
	GPIOC->ODR |= GPIO_ODR_ODR9;
//	if(!(GPIOA->IDR & GPIO_IDR_IDR10)){ //5
//		  GPIOA->ODR ^= GPIO_ODR_ODR5;
//		  return 5;
//		  while(!(GPIOA->IDR & GPIO_IDR_IDR10)){}
//		  HAL_Delay(10);
//	  }
//	if(!(GPIOA->IDR & GPIO_IDR_IDR6)){ //8
//		  GPIOA->ODR ^= GPIO_ODR_ODR5;
//		  return 8;
//		  while(!(GPIOA->IDR & GPIO_IDR_IDR6)){}
//		  HAL_Delay(10);
//	  }
//	if(!(GPIOA->IDR & GPIO_IDR_IDR11)){ //0
//		  GPIOA->ODR ^= GPIO_ODR_ODR5;
//		  return 0;
//		  while(!(GPIOA->IDR & GPIO_IDR_IDR11)){}
//		  HAL_Delay(10);
//	  }
	if(!(GPIOA->IDR & GPIO_IDR_IDR7)){ //2
		  GPIOA->ODR ^= GPIO_ODR_ODR5;
		  return 2;
		  while(!(GPIOA->IDR & GPIO_IDR_IDR7)){}
		  HAL_Delay(10);
	  }

	//Third Column
	 GPIOC->ODR |= GPIO_ODR_ODR8;
	 GPIOC->ODR |= GPIO_ODR_ODR6;
	 GPIOC->ODR &= ~GPIO_ODR_ODR5;
	 GPIOC->ODR |= GPIO_ODR_ODR9;
//	  if(!(GPIOA->IDR & GPIO_IDR_IDR10)){ //6
//		  GPIOA->ODR ^= GPIO_ODR_ODR5;
//		  return 6;
//		  while(!(GPIOA->IDR & GPIO_IDR_IDR10)){}
//		  HAL_Delay(10);
//	  }
//	 if(!(GPIOA->IDR & GPIO_IDR_IDR6)){ //9
//		  GPIOA->ODR ^= GPIO_ODR_ODR5;
//		  return 9;
//		  while(!(GPIOA->IDR & GPIO_IDR_IDR6)){}
//		  HAL_Delay(10);
//	  }
//	 if(!(GPIOA->IDR & GPIO_IDR_IDR11)){ //# space
//		  GPIOA->ODR ^= GPIO_ODR_ODR5;
//		  return 11;
//		  while(!(GPIOA->IDR & GPIO_IDR_IDR11)){}
//		  HAL_Delay(10);
//	  }
	 if(!(GPIOA->IDR & GPIO_IDR_IDR7)){ //3
		  GPIOA->ODR ^= GPIO_ODR_ODR5;
		  return 3;
		  while(!(GPIOA->IDR & GPIO_IDR_IDR7)){}
		  HAL_Delay(10);
	  }

	//Fourth Column
	GPIOC->ODR |= GPIO_ODR_ODR8;
	GPIOC->ODR |= GPIO_ODR_ODR6;
	GPIOC->ODR |= GPIO_ODR_ODR5;
	GPIOC->ODR &= ~GPIO_ODR_ODR9;
//	if(!(GPIOA->IDR & GPIO_IDR_IDR10)){ //B
//	  GPIOA->ODR ^= GPIO_ODR_ODR5;
//	  return 13;
//	  while(!(GPIOA->IDR & GPIO_IDR_IDR10)){}
//	  HAL_Delay(10);
//	}
//	if(!(GPIOA->IDR & GPIO_IDR_IDR6)){ //C
//	  GPIOA->ODR ^= GPIO_ODR_ODR5;
//	  return 14;
//	  while(!(GPIOA->IDR & GPIO_IDR_IDR6)){}
//	  HAL_Delay(10);
//	}
//	if(!(GPIOA->IDR & GPIO_IDR_IDR11)){ //D
//	  GPIOA->ODR ^= GPIO_ODR_ODR5;
//	  return 15;
//	  while(!(GPIOA->IDR & GPIO_IDR_IDR11)){}
//	  HAL_Delay(10);
//	}
	if(!(GPIOA->IDR & GPIO_IDR_IDR7)){ //A
	  GPIOA->ODR ^= GPIO_ODR_ODR5;
	  return 12;
	  while(!(GPIOA->IDR & GPIO_IDR_IDR7)){}
	  HAL_Delay(10);
	}

	return 2;
}

