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
#include "myprintf.h"
#include "sensorConfig.h"
#include "lcd.h"
#include <string.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void USER_RCC_Init(void);
void USER_GPIO_Init(void);

float USER_pressure_sensor(uint16_t dataADC, float voltage);
float USER_flow_sensor(uint16_t event_val_1, uint16_t event_val_2, uint16_t event_val, float period, float frequency);

void USER_LCD_Init(void);
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
  uint16_t dataADC = 0; //Voltage in bits from (pressure sensor)
  float voltage = 0.0; //Variable to convert values to float

  uint16_t event_val = 0, event_val_2 = 0, event_val_1 = 0; //Time in bits (Flow sensor)
  float period = 0, frequency = 0; //Variable to convert time to float

  int state = 2; //1 -> Idle, 2 -> Start, 3 -> Flow, A(12) -> pressure, 4 -> Both
  int stateprev = 2;

  char data_char[25];
  uint8_t data_uint8[25];
  uint8_t test[] = "Test in Project\n";

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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  ADC1->CR2	|=	 ADC_CR2_ADON;//	Starts the conversion of ADC
  printf("Idle");
  while (1)
  {
    /* USER CODE END WHILE */

	  state = USER_MXKeyboard_SelectKey();
	  if (state!=2){
		  stateprev=state;
	  }
	  if (stateprev == 1){
		  LCD_Clear();
		  LCD_Set_Cursor(1,0);
		  LCD_Put_Str("IDLE");
	  }
	  else if(stateprev == 2 || stateprev == 3 || stateprev == 12 || stateprev == 4){
		  printf("\n");

		  //Pressure Sensor, ADC
		  voltage = USER_pressure_sensor(dataADC, voltage);

		  //Flow Sensor, Timer Module(Capture Mode)
		  frequency = USER_flow_sensor(event_val_1,event_val_2,event_val,period,frequency);

		  char v_c[50]; //Creating chars for display LCD
		  char v_c2[50]="Psi: ";
		  char f_c[50];
		  sprintf(v_c, "%f", voltage);
		  sprintf(f_c, "%f", frequency);
		  strcat(v_c2,v_c);

		  //printf(data_char, "%.6f,%.6f\n", voltage, frequency); //Creating chars for UART
		  sprintf(data_char, "%d,%d\n", (int)voltage*1e+06,(int)frequency*1e+06 ); //Creating chars for UART
		  // Convert char array to uint8_t array
		  for (int i = 0; data_char[i] != '\n'; i++) {
			  data_uint8[i] = (uint8_t)data_char[i];
		  }


		  if(stateprev == 3){
			  LCD_Clear();
			  LCD_Set_Cursor(1,0);
			  LCD_Put_Str("FLux:");
			  LCD_Set_Cursor(2,0);
			  LCD_Put_Str(f_c);
		  }
		  if(stateprev == 12){
			  LCD_Clear();
			  LCD_Set_Cursor(1,0);
			  LCD_Put_Str("Pressure:");
			  LCD_Set_Cursor(2,0);
			  LCD_Put_Str(v_c);
		  }
		  if(stateprev == 4 || stateprev == 2){
			  LCD_Clear();
			  LCD_Set_Cursor(1,0);
			  LCD_Put_Str("Fx: ");
			  LCD_Set_Cursor(1,5);
			  LCD_Put_Str(f_c);
			  LCD_Set_Cursor(2,0);
			  LCD_Put_Str(v_c2);
		  }

		  //Timer, Timer Module(Timer Mode)
		  USER_USART3_Transmit(data_uint8,sizeof(data_uint8));
		  //USER_USART3_Transmit(test,sizeof(test));
		  printf("Wait 1000ms\r\n");
		  USER_TIMER3_TIMER_Init();
	  }


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
}
float USER_pressure_sensor(uint16_t dataADC, float voltage){
  dataADC = USER_ADC_Read();
  voltage = (3.3)*((float)dataADC/4095);
  printf("Voltage: %.2f \r\n",voltage);
  return voltage;
}

float USER_flow_sensor(uint16_t event_val_1, uint16_t event_val_2, uint16_t event_val, float period, float frequency){
	if(!(TIM2->SR & TIM_SR_CC1IF)){
		period=0;
		frequency=0;
	}else{
		event_val_1 = USER_TIMER2_Capture_Event();
		event_val_2 = USER_TIMER2_Capture_Event();
		event_val = event_val_2 - event_val_1;
		period = ( 1.0 / 64000000.0 ) * event_val * (TIM2->PSC + 1);
		frequency = 1/period;
	}

	printf("Period: %.5f\r\n",period);
	printf("Frequency: %.5f\r\n",frequency);
	return frequency;
}

void USER_LCD_Init(void){
	LCD_Init( );//				inicializamos la libreria del LCD
	LCD_Clear( );//			borra la pantalla
	LCD_Set_Cursor(1,0);
	LCD_Put_Str("IDLE");
}
/* USER CODE END 4 */

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
