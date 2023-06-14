/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "myprintf.h"
#include "sensorConfig.h"
#include "lcd.h"
#include "hx711.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h> //For random
#include <time.h> //For random

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
//osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
osThreadId measureTaskHandle; //T1/*Tasks Handles*/
osThreadId sendDataTaskHandle; //T2
osThreadId mxkeypadTaskHandle; //T3
osThreadId lcdTaskHandle; //T4
osThreadId weightSensorHandle; //T5 (extra)
osThreadId testTaskHandle; //T6 (extra)

osMessageQId sendValuesQueueHandle; //T1->T2
osMessageQId sendValuesLCDQueueHandle; //T1->T4
osMessageQId sendButtonQueueHandle; //T3->T4
osMessageQId confirmationQueueHandle; //T4->T3
osMessageQId startMeasureQueueHandle; //T3->T1
osMessageQId weightDataQueueHandle; //T5->T1

hx711_t loadcell;/*Weight sensor variables*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);


/* USER CODE BEGIN PFP */
void measureTask(void const * argument); //T1 /*Tasks */
void sendDataTask(void const * argument); //T2
void mxkeypadTask(void const * argument); //T3
void lcdTask(void const * argument);	  //T4
void weightTask(void const * argument); //T5 (extra)
void testTask(void const * argument);

void USER_RCC_Init(void); /*Init of clocks, GPIO and LCD func*/
void USER_GPIO_Init(void);
void USER_LCD_Init(void);

void USER_pressure_sensor(uint16_t dataADC, float * voltage); /*Data recovery functions*/
void USER_flow_sensor(uint16_t event_val_1, uint16_t event_val_2, uint16_t event_val, float * frequency);

void convert2char(float f1, float f2, char *result); /*Interface functions*/
void outputInLCD(int stateprev, char * measurement);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	srand(time(NULL));

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  USER_RCC_Init();
  USER_GPIO_Init();
  //Sensor functions Init()
  USER_USART2_Init(); // USART 2
  USER_USART3_Init(); // USART 3
  USER_ADC_Init(); // ADC
  USER_ADC_Calibration();
  USER_TIMER2_Capture_Init(); //Timer (Capture mode)
  USER_LCD_Init(); //Matrix Keyboard

  loadcell.lock=0; //Weight sensor
  loadcell.offset=0;
  hx711_init(&loadcell, GPIOC, GPIO_PIN_3, GPIOC, GPIO_PIN_2); //3 clk, 2 pin
  hx711_coef_set(&loadcell, 500); // read after calibration //min 3750, max 3900-3950  354.5
  hx711_tare(&loadcell, 10);/*morado 5v, negro ground, blanco data, gris clk*/

  printf("Starting...\r\n");
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  osMessageQDef(sendValues, 1, uint32_t);
  sendValuesQueueHandle = osMessageCreate(osMessageQ(sendValues), NULL);
  osMessageQDef(sendButton, 1, uint32_t);
  sendButtonQueueHandle = osMessageCreate(osMessageQ(sendButton), NULL);
  osMessageQDef(confirmation, 1, uint32_t);
  confirmationQueueHandle = osMessageCreate(osMessageQ(confirmation), NULL);
  osMessageQDef(startMeasure, 1, uint32_t);
  startMeasureQueueHandle = osMessageCreate(osMessageQ(startMeasure), NULL);
  osMessageQDef(sendValuesLCD, 1, uint32_t);
  sendValuesLCDQueueHandle = osMessageCreate(osMessageQ(sendValuesLCD), NULL);
  osMessageQDef(weightValues, 1, uint32_t);
  weightDataQueueHandle = osMessageCreate(osMessageQ(weightValues), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(Task1, measureTask, osPriorityNormal, 0, 256);
  measureTaskHandle = osThreadCreate(osThread(Task1), NULL);
  osThreadDef(Task2, sendDataTask, osPriorityNormal, 0, 128);
  sendDataTaskHandle = osThreadCreate(osThread(Task2), NULL);
//  osThreadDef(Task3, mxkeypadTask, osPriorityNormal, 0, 128);
//  mxkeypadTaskHandle = osThreadCreate(osThread(Task3), NULL);
//  osThreadDef(Task4, lcdTask, osPriorityNormal, 0, 256);
//  lcdTaskHandle = osThreadCreate(osThread(Task4), NULL);
  osThreadDef(Task5, weightTask, osPriorityNormal, 0, 256);
  weightSensorHandle = osThreadCreate(osThread(Task5), NULL);
//  osThreadDef(Task6, testTask, osPriorityNormal, 0, 384);
//  testTaskHandle = osThreadCreate(osThread(Task6), NULL);

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void USER_RCC_Init(void){
	RCC->APB2ENR	|=	 RCC_APB2ENR_IOPAEN;// I/O port A clock enable
	RCC->APB2ENR	|=	 RCC_APB2ENR_IOPBEN;// I/O port B clock enable
	RCC->APB2ENR	|=	 RCC_APB2ENR_IOPCEN;// I/O port C clock enable

	USER_TIMER2_RCC_Init(); // TIMER2 clock enable
	USER_TIMER3_RCC_Init(); //TIMER3 clock enable
	USER_USART2_RCC_Init(); // USART2 clock enable
	USER_USART3_RCC_Init(); //USART3 clock enable
	USER_ADC_RCC_Init(); // ADC clock enable
}
void USER_GPIO_Init(void){

	GPIOA->BSRR = GPIO_BSRR_BR5;//PA5 -> 0, LD2 OFF
	GPIOA->CRL &= ~GPIO_CRL_CNF5 & ~GPIO_CRL_MODE5_1;
	GPIOA->CRL |= GPIO_CRL_MODE5_0;

	USER_USART2_GPIO_Init(); //USART 2
	USER_USART3_GPIO_Init(); //USART 3
	USER_MATRIXKEYPAD_GPIO_Init(); //Matrix Keypad
	USER_ADC_GPIO_Init(); //ADC
	USER_TIMER2_CAPTUREMODE_GPIO_Init(); //Timer (Capture Mode)
	USER_GPIO_Init_Valve_WaterPump(); //Valve in, out and water pump
}
void USER_pressure_sensor(uint16_t dataADC, float *voltage){
  dataADC = USER_ADC_Read();
  *voltage = (3.3)*((float)dataADC/4095);
//  printf("Voltage: %.2f \r\n",voltage);
//  return voltage;
}

void USER_flow_sensor(uint16_t event_val_1, uint16_t event_val_2, uint16_t event_val, float * frequency){
	float period = 0;
//	if(!(TIM2->SR & TIM_SR_CC1IF)){
//		period=0;
//		*frequency=0;
//	}else{
		event_val_1 = USER_TIMER2_Capture_Event();
		event_val_2 = USER_TIMER2_Capture_Event();
		event_val = event_val_2 - event_val_1;
		period = ( 1.0 / 64000000.0 ) * event_val * (TIM2->PSC + 1);
		*frequency = 1/period;
//	}

//	printf("Period: %.5f\r\n",period);
//	printf("Frequency: %.5f\r\n",frequency);
//	return frequency;
}

void USER_LCD_Init(void){
	LCD_Init( );//				inicializamos la libreria del LCD
	LCD_Clear( );//			borra la pantalla
	LCD_Set_Cursor(1,0);
	LCD_Put_Str("IDLE");
}

void convert2char(float f1, float f2, char *result) {
	sprintf(result, "%03d.%06d,%03d.%06d", (int)f1, (int)(f1 * 1000000) % 1000000,
	          (int)f2, (int)(f2 * 1000000) % 1000000);
}

void outputInLCD(int stateprev, char * measurement){
	char show[21] = "aaa.aaaaaa,bbb.bbbbbb";
	char voltage_c[10]="333.333333"; //Creating chars for display LCD
	char frequency_c[10]="444.444444";
	char voltage_c2[15]="Psi: ";

	memcpy(show, measurement, 22 * sizeof(uint8_t));
//	printf("lcdfunc %s\r\n",show);

	char* token = strtok(show, ",");
    if (token != NULL) {
        strcpy(voltage_c, token);
    }
    token = strtok(NULL,",");
    if (token != NULL) {
		strcpy(frequency_c, token);
	}
//    printf("%s\r\n",voltage_c);
//    printf("%s\r\n",frequency_c);
	strcat(voltage_c2,voltage_c);

	switch(stateprev){
	case 1:
		LCD_Clear();
		LCD_Set_Cursor(1,0);
		LCD_Put_Str("IDLE");
		break;

	case 2:
		LCD_Clear();
		LCD_Set_Cursor(1,0);
		LCD_Put_Str("Fx: ");
		LCD_Set_Cursor(1,5);
		LCD_Put_Str(frequency_c);
		LCD_Set_Cursor(2,0);
		LCD_Put_Str(voltage_c2);
		break;

	default:
		LCD_Clear();
		LCD_Set_Cursor(1,0);
		LCD_Put_Str("IDLE");
		break;

	}
}

void measureTask(void const * argument)
{
	uint32_t temp; /*Period variables*/
	uint32_t counter = 0;
	int delay = 250;
	float voltage, frequency;/*Task variables*/
	uint16_t dataADC = 0; //Voltage in bits from (pressure sensor)
	uint16_t event_val = 0, event_val_2 = 0, event_val_1 = 0; //Time in bits (Flow sensor)
	uint8_t msg[21] = "123.456789,987.654321"; //message for other tasks
	int start = 1; //Control flow variable (start measure)
	osEvent r_event_start; //Event for receiving button
	osEvent r_event_weight; //Event for receiving weight;
	int weight = 0;
	ADC1->CR2	|=	 ADC_CR2_ADON;//	Starts the conversion of ADC
	for(;;)
	{
//		weight = hx711_weight(&loadcell, 10);
//		printf("%.4f",weight);
//		printf("Task1\r\n");
		r_event_start = osMessageGet(startMeasureQueueHandle, 0); //Receiving values measured
		if (r_event_start.status == osEventMessage){
			start = r_event_start.value.v;
		}
		while(start == 1){
			USER_pressure_sensor(dataADC, &voltage); //Pressure Sensor, ADC
			USER_flow_sensor(event_val_1,event_val_2,event_val,&frequency);//Flow Sensor, Timer Module(Capture Mode)
//			voltage = (float)rand() / RAND_MAX * 100; //Random values for testing
//			frequency = (float)rand() / RAND_MAX * 100;
			convert2char(voltage, frequency, msg);//Convert values into char array
			if( osMessagePut(sendValuesQueueHandle, msg, 0) != osOK ){} //Send data to uart
			if( osMessagePut(sendValuesLCDQueueHandle, msg, 0) != osOK ){} //Send data to lcd

		}
		temp = osKernelSysTick() - (delay*counter++);
		osDelay(delay-temp);
	}
}

void sendDataTask(void const * argument)
{
	uint32_t temp;/*Period variables*/
	int delay = 250;
	uint32_t counter = 0;
	osEvent r_event_values;/*Task variables*/
	uint8_t inputValues[21] = "000.000000,000.000000";
	for(;;)
	{
//		printf("Task2\r\n");
		r_event_values = osMessageGet(sendValuesQueueHandle, 0); //Receiving values measured
		if (r_event_values.status == osEventMessage){
//			memcpy(inputValues, (uint8_t*)r_event_values.value.p, 21 * sizeof(uint8_t));
//			printf("sendTask\r\n");
			printf("%s\r\n",(uint8_t*)r_event_values.value.p);
			USER_USART3_Transmit(r_event_values.value.p,21 * sizeof(uint8_t));
		}
		temp = osKernelSysTick() - (delay*counter++);
		osDelay(delay-temp);
	}

}

void mxkeypadTask(void const * argument)
{
	uint32_t temp;/*Period variables*/
	int delay = 250;
	uint32_t counter = 0;
	int key = 2;/*Task variables*/
	int counterStart = 0;
	uint32_t msg = 0;
	uint32_t input = 1;
	osEvent r_event;
	for(;;)
	{
		key = USER_MXKeyboard_SelectKey(); //Obtaining key
		msg = key;
		if (key == 1 && counterStart <= 0){ //Sending start message only once
			counterStart++;
			if( osMessagePut(startMeasureQueueHandle, msg, 0) != osOK ){}
		}

//		if( osMessagePut(sendButtonQueueHandle, msg, 0) != osOK ){} //Sending button pressed
//		while(input == 0){						//Receiving acknowledgement
//			r_event = osMessageGet(confirmationQueueHandle, 0);
//			if( r_event.status == osEventMessage ){
//				input = r_event.value.v;
//			}else{
//			}
//		}

		temp = osKernelSysTick() - (delay*counter++);
		osDelay(delay-temp);
	}
}

void lcdTask(void const * argument)
{
	uint32_t temp;/*Period variables*/
	int delay = 1000;
	uint32_t counter = 0;
	osEvent r_event;/*Task variables*/
	osEvent r_event_values;
	uint32_t inputButton = 1;
	int lcdState = 2;
	int counterStart = 0;
	uint8_t inputValues[21] = "111.111111,222.222222";
	uint32_t output = 1;
	for(;;)
	{
//		printf("Task3\r\n");
//		r_event = osMessageGet(sendButtonQueueHandle, 0); //Receiving button pressed
//		if( r_event.status == osEventMessage ){
//			output = 1;
//			if(r_event.value.v == 1 && counterStart <=0) { //Receive start only once
//				counterStart++;
//				inputButton = r_event.value.v;
//				lcdState = 4;
//			}
//		}else{
//			output = 0;
//		}
//
		r_event_values = osMessageGet(sendValuesLCDQueueHandle, 0); //Receiving values measured
		if (r_event_values.status == osEventMessage){
//			memcpy(inputValues, (uint8_t*)r_event_values.value.p, 21 * sizeof(uint8_t));
//			printf("lcdT: %s\r\n",(uint8_t*)r_event_values.value.p);
			outputInLCD(lcdState, r_event_values.value.p);
		}


//		if( osMessagePut(weightDataQueueHandle, output, 0) != osOK ){}

		temp = osKernelSysTick() - (delay*counter++);
		osDelay(delay-temp);
	}
}

void weightTask(void const * argument){
//	int delay = 500;
//	uint32_t counter = 0;
//	uint32_t temp = 0;
	float weight = 0;
	int counterF = 0;
	for(;;)
	{
//		printf("Task\r\n");
		weight = hx711_weight(&loadcell, 10);
//		weight=3000;
//		if( osMessagePut(weightDataQueueHandle, (uint32_t)(int)weight, 0) != osOK ){}
//		printf("%.4f\r\n",weight);
		if(weight<5000 && weight>3000){
			if(weight<=3740) {
				//turn valveIN on
				GPIOC->ODR &= ~GPIO_ODR_ODR1;
				//turn valveOUT and water pump off
				GPIOB->ODR |= GPIO_ODR_ODR0;
				GPIOA->ODR |= GPIO_ODR_ODR4;
//				 printf("Less\r\n");
			}else if(weight>=3930){
				//turn valveIN off
				GPIOC->ODR |= GPIO_ODR_ODR1;
				//turn valveOUT and water pump on
				GPIOB->ODR &= ~GPIO_ODR_ODR0;
				GPIOA->ODR &= ~GPIO_ODR_ODR4;
//				printf("More\r\n");
			}
		}
//		printf("%i",counterF);
		counterF++;
		if (counterF >100) counterF = 0;
//		temp = osKernelSysTick() - (delay*counter++);
//		osDelay(delay-temp);
	}
}

void testTask(void const * argument){
	int delay = 100;
	uint32_t counter = 0;
	uint32_t temp = 0;
	osEvent r_event_weight;
	int counterF = 0;
	for(;;)
	{
//		printf("Task 6\r\n");
		r_event_weight = osMessageGet(weightDataQueueHandle, 0); //Receiving values measured
		if (r_event_weight.status == osEventMessage){
			printf("weightM %i\r\n",(int)r_event_weight.value.v);
		}
//		printf("Repeat: %i\r\n",counterF);
		counterF++;
		if (counterF >100) counterF = 0;
		temp = osKernelSysTick() - (delay*counter++);
		osDelay(delay-temp);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
