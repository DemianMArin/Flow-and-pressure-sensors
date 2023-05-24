/*
 * sensorConfig.h
 *
 *  Created on: May 2, 2023
 *      Author: demian
 */

#ifndef INC_SENSORCONFIG_H_
#define INC_SENSORCONFIG_H_

#include <stdint.h>

//USART
void USER_USART2_RCC_Init(void);
void USER_USART2_GPIO_Init(void);
void USER_USART2_Init(void);

//TIMER2
void USER_TIMER2_RCC_Init(void);
void USER_TIMER3_RCC_Init(void);
void USER_TIMER2_CAPTUREMODE_GPIO_Init(void);
void USER_TIMER3_TIMER_Init(void); //Timer for N seconds
void USER_TIMER2_Capture_Init(void); //Init for Capture Mode
uint16_t USER_TIMER2_Capture_Event(void); //Returns time of Capture Event

//ADC
void USER_ADC_RCC_Init(void);
void USER_ADC_GPIO_Init(void);
void USER_ADC_Init(void);
void USER_ADC_Calibration(void);
uint16_t USER_ADC_Read(void);

//MATRIX KEYPAD
void USER_MATRIXKEYPAD_GPIO_Init(void);
int USER_MXKeyboard_SelectKey(void);


#endif /* INC_SENSORCONFIG_H_ */
