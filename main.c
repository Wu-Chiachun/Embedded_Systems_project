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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "lcd.h"
#include "string.h"
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
 RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
char time[30];
char date[30];

RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us(int length){
	__HAL_TIM_SET_COUNTER(&htim1,0);	// set counter to 0
	while ((__HAL_TIM_GET_COUNTER(&htim1)) < length);	// breaks loop when length is reached
}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t pin_number)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = pin_number;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t pin_number)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = pin_number;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT22_Init(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	//LCD_DrawString(10, 60, "start output stuff");
	Set_Pin_Output(GPIOA, GPIO_PIN_5);							// set pin to output(will set to input later)
	//LCD_DrawString(10, 80, "pin set as output");
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	delay_us(1200);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//	LCD_DrawString(10, 100, "delay 20us");
//	delay_us(20);
	//LCD_DrawString(10, 120, "delay 30us success");
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	Set_Pin_Input(GPIOA, GPIO_PIN_5);
	LCD_DrawString(10, 140, "finished output stuff");
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET);
}

void DHT11_Init(){
	Set_Pin_Output(GPIOE, GPIO_PIN_6);// set pin to output(will set to input later)
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
	delay_us(1000);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
	delay_us(20000);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
	delay_us(20);
	Set_Pin_Input(GPIOE, GPIO_PIN_6);
}

uint8_t Check_Response_DHT11(){
	//LCD_DrawString(40, 180, "check");
	uint8_t response=0;
	delay_us(40);
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6) == GPIO_PIN_RESET){
		delay_us(80);
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6) == GPIO_PIN_SET){
			response=-1;
		}
		else{
			response=0;
		}
	}
	while(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6) == GPIO_PIN_SET);
	LCD_DrawString(40, 200, "*****");
	return response;
}

uint8_t DHT11_Read(){
	uint8_t result=0;
	for(int i=0; i<8; ++i){
		while(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6) == GPIO_PIN_RESET);
		LCD_DrawString(100, 200, "bits!");
		delay_us(40);
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6) == GPIO_PIN_SET){
			result |= (1 << (7-i));
		}
		while(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6) == GPIO_PIN_SET);
		LCD_DrawString(150, 200, "fin!");
	}
	return result;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_FSMC_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  LCD_INIT();
  HAL_TIM_Base_Start(&htim1);
  HAL_Delay(1000);
  // with HAL_Delay
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  delay_us(10000);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//PIR sensor and LCD on/off
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LCD_BK_PORT,LCD_BK_PIN,GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LCD_BK_PORT,LCD_BK_PIN,GPIO_PIN_SET);
	}

	//RTC
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	sprintf(date, "Date: %02d.%02d.%02d", sDate.Date, sDate.Month, sDate.Year);
	sprintf(time, "Time: %02d.%02d.%02d", sTime.Hours, sTime.Minutes, sTime.Seconds);

	LCD_DrawString(20, 20, date);
	LCD_DrawString(20, 40, time);

	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET){
		uint8_t sec=0x20, min=0x59, hr=0x05, date=0x12, year=0x23;
		// update RTC
		RTC_TimeTypeDef sTime2 = {0};
		RTC_DateTypeDef sDate2 = {0};

		/* Fill in the RTC_TimeTypeDef structure with the new values */
		sTime2.Hours = hr;
		sTime2.Minutes = min;
		sTime2.Seconds = sec;

		sDate2.WeekDay = RTC_WEEKDAY_WEDNESDAY;
		sDate2.Month = RTC_MONTH_APRIL;
		sDate2.Date = date;
		sDate2.Year = year;

		/* Set the new time in the RTC */
		HAL_RTC_SetTime(&hrtc, &sTime2, RTC_FORMAT_BCD);
		HAL_RTC_SetDate(&hrtc, &sDate2, RTC_FORMAT_BCD);
		HAL_Delay(500);
	}

	if(sTime.Hours == 6 && sTime.Minutes == 0 && sTime.Seconds==0){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_Delay(5000);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	}

	//pump and light and dht11 commented
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
		HAL_Delay(2000);
//		uint8_t rh1, rh2, temp1, temp2, check;
//		uint16_t rh, temp;
//		float temperature=0.0, humidity = 0.0;
//		DHT11_Init();
//		uint8_t B = Check_Response_DHT11();
//		//start receiving data
//		if(B==0){
//			rh1 = DHT11_Read();
//			rh2 = DHT11_Read();
//			temp1 = DHT11_Read();
//			temp2 = DHT11_Read();
//			check = DHT11_Read();
//			if(rh1 + rh2 + temp1 + temp2 == check){
//				LCD_DrawString(10, 220, "Checksum validated");
//			}
//			else{
//				LCD_DrawString(10, 220, "Signal corrupted");
//			}
//			rh = ((rh1 << 8) | rh2 );
//			temp = ((temp1 << 8) | temp2);
//		}
//		humidity = (float) (rh/10.0);
//		temperature = (float) (temp/10.0);
//		char TEMP[11], HUMID[11];
//		sprintf(HUMID, "%f", humidity);
//		sprintf(TEMP, "%f", temperature);
//		LCD_DrawString(10, 240, TEMP);
//		LCD_DrawString(20, 260, HUMID);

	}

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x22;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  DateToUpdate.Month = RTC_MONTH_APRIL;
  DateToUpdate.Date = 0x19;
  DateToUpdate.Year = 0x23;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DHT11_Pin|LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Strip_Pin|pump_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_alt_GPIO_Port, DHT11_alt_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DHT11_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : K1_Button_Pin */
  GPIO_InitStruct.Pin = K1_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(K1_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT22_Pin */
  GPIO_InitStruct.Pin = DHT22_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Strip_Pin pump_Pin */
  GPIO_InitStruct.Pin = LED_Strip_Pin|pump_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : button_for_set_time_Pin PIR___HC_SR501_Pin */
  GPIO_InitStruct.Pin = button_for_set_time_Pin|PIR___HC_SR501_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_BL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_alt_Pin */
  GPIO_InitStruct.Pin = DHT11_alt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DHT11_alt_GPIO_Port, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

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
