/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"
#include "ssd1306.h"
#include "fonts.h"
#include "stdio.h"
#include "bitmap.h"
#include "horse_anim.h"
#include "queue.h"
#include "dma_adc.h"
#include "UART.h"
#include "semphr.h"
#include "timers.h"
#include "SEGGER_SYSVIEW.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
FATFS fs;
FATFS *pfs;
FIL fil;
UINT br,bw;
FRESULT fresult;
DWORD fre_clust;
uint32_t total, free_space;
char buffer[256];
char OLEDbuffer[256];
char motionState[50];
char adcVal[50];
char tempState[50];
char SDCardUART[30];
char help_str[10];
uint16_t adc_buffer[ADC_SAMPLES * 2 * 2] = { 0 }; // ADC_SAMPLES samples, 2 channels, 2 buffers
volatile uint32_t pressStartTime = 0;
volatile uint32_t pressEndTime = 0;
volatile BaseType_t isButtonPressed = pdFALSE;
uint8_t help = 1;
uint32_t count=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* Definitions for idleTask */
osThreadId_t idleTaskHandle;
const osThreadAttr_t idleTask_attributes = {
  .name = "idleTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
#define MAX_QUEUE_SIZE 10
#define MAX_CHARS 40
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
//void MX_FREERTOS_Init(void);
void MX_FREERTOS_Init(void);
void SDCard_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void vRCWLtask(void *pvParameters);
void vUARTtask(void *pvParameters);
void vAQItask(void *pvParameters);
void vRTCtask(void *pvParameters);
void vMQ135task(void *pvParameters);
void vInternalTemptask(void *pvParameters);
void vSwitchtask(void *pvParameters);
void vTimerCallback(TimerHandle_t xTimer);
void printOLED(char*string, uint8_t y);
void send_SDCard(char*string);
TaskHandle_t RCWL_Handle=NULL;
TaskHandle_t AQI_Handle=NULL;
TaskHandle_t RTC_Handle=NULL;
TaskHandle_t UART_Handle=NULL;
TaskHandle_t MQ135_Handle=NULL;
TaskHandle_t InternalTemp_Handle=NULL;
TaskHandle_t Switch_Handle=NULL;
SemaphoreHandle_t MutexOne=NULL;
SemaphoreHandle_t binarySemaphoreISR=NULL;
TimerHandle_t buttonPressTimer=NULL;

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  MutexOne = xSemaphoreCreateMutex();
  binarySemaphoreISR = xSemaphoreCreateBinary();
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  buttonPressTimer = xTimerCreate("ButtonPressTimer", pdMS_TO_TICKS(150), pdTRUE, NULL, vTimerCallback);
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of idleTask */
  idleTaskHandle = osThreadNew(StartDefaultTask, NULL, &idleTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  xTaskCreate(vUARTtask,"UART_Task", 128, NULL, 1, &UART_Handle);// at 256-Stack Size HELP doesn't prints on OLED SCREEN. ONLY GOD KNOWS WHY!!
  xTaskCreate(vRCWLtask,"RCWL_Tsk", 512, NULL, 2, &RCWL_Handle);
  xTaskCreate(vMQ135task,"MQ135_Task", 512, NULL, 2, &MQ135_Handle);
  xTaskCreate(vInternalTemptask,"Temp_Task", 512, NULL, 2, &InternalTemp_Handle);
  xTaskCreate(vRTCtask,"RTC_Task", 512, NULL, 2, &RTC_Handle);///////
  xTaskCreate(vSwitchtask,"Switch_Task", 1024, NULL, 3, &Switch_Handle);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  SEGGER_SYSVIEW_Conf();
  vSetVarulMaxPRIGROUPValue();
  SEGGER_SYSVIEW_Start();


  SDCard_Init();
  MX_FREERTOS_Init();
  SSD1306_Init();
  SSD1306_Clear();

  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//  	SSD1306_GotoXY (10,10); // goto 10, 10
//		SSD1306_Puts("HELLOOOOOOOOWERT", &Font_7x10, 1);
//		SSD1306_UpdateScreen();
//  	char motionState[20];
//  	sprintf(motionState,"Motion");
//  	send_SDCard(motionState);
//  	HAL_Delay(500);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 17;
  sTime.Minutes = 42;
  sTime.Seconds = 30;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
  sDate.Month = RTC_MONTH_NOVEMBER;
  sDate.Date = 10;
  sDate.Year = 24;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 840-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RCWL_Pin */
  GPIO_InitStruct.Pin = RCWL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RCWL_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void SDCard_Init(void){
	fresult=f_mount(&fs,"",0);
	if(fresult!=FR_OK){
		send_uart("error in mounting SD card ....\n\r");
	}
	else{
		send_uart("SD card mounted\n\r");
	}
}


void MX_FREERTOS_Init(void){
	vQueueAddToRegistry(binarySemaphoreISR, "binary_semaphore_ISR");
	vQueueAddToRegistry(MutexOne, "Mutex_One");
}


void send_OLED(char*string, uint8_t y){
	if(help){
		SSD1306_GotoXY (5,y); // goto 10, 10
		SSD1306_Puts(string, &Font_7x10, 1);
		SSD1306_UpdateScreen();
	}
}


void send_SDCard(char*string){
		count++;
		fresult=f_open(&fil, "data.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_OPEN_APPEND);
		strcat(string, "\n\r");
		fresult = f_puts(string,&fil);
		f_close(&fil);
}

void vUARTtask(void *pvParameters){
//	uint16_t count=0;
	while(1){
//		count++;
//		sprintf(buffer,"count is %d\n\r",count);
//		send_uart(buffer);
//		vTaskDelay(50);
	}
}

void vSwitchtask(void *pvParameters){
	while(1){
		if(xSemaphoreTake(binarySemaphoreISR, portMAX_DELAY)==pdTRUE ){
			if (!isButtonPressed) {
				pressStartTime = xTaskGetTickCount();
				isButtonPressed = pdTRUE;
				xTimerStart(buttonPressTimer, 0);


				help=0;
				sprintf(help_str,"HELP");
				fresult=f_open(&fil, "data.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_OPEN_APPEND);
				strcat(help_str,"\n\r");
				fresult = f_puts(help_str,&fil);
				f_close(&fil);


				SSD1306_Clear();
				vTaskDelay(20);
				SSD1306_GotoXY (25,20);
				SSD1306_Puts("HELP", &Font_16x26, 1);
				SSD1306_UpdateScreen();
				send_uart("Button Pressed\r\n");
				vTaskDelay(970);


				help=1;
			}
			vTaskDelay(20);
		}
	}
}

void vInternalTemptask(void *pvParameters){
  HAL_TIM_Base_Start_IT(&htim3); // First get the timer running
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, ADC_SAMPLES * 2 * 2); // Now fire up the ADC DMA
	while(1){
		sprintf(tempState,"Temp = %4.2f C",temp);// to get raw value call temp_avg variable
		if(xSemaphoreTake(MutexOne, portMAX_DELAY)==pdTRUE){
			send_OLED(tempState, 50);
			send_SDCard(tempState);
			send_uart(tempState);
			xSemaphoreGive(MutexOne);
		}
		vTaskDelay(20);
	}
}


void vRCWLtask(void *pvParameters){
	while(1){
		uint8_t rcwl = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
		char *rcwl_state = (rcwl == 1) ? "HIGH" : "LOW ";
		sprintf(motionState,"Motion: %s", rcwl_state);
		if(xSemaphoreTake(MutexOne, portMAX_DELAY)==pdTRUE){
			send_OLED(motionState, 30);
			send_SDCard(motionState);
			send_uart(motionState);
			xSemaphoreGive(MutexOne);
		}
		vTaskDelay(5);
	}
}

void vRTCtask(void *pvParameters){
	while(1){
		char time[60];
		char date[60];
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		sprintf(date,"Date :%02d.%02d.%02d",sDate.Date, sDate.Month, sDate.Year);
		sprintf(time,"Time :%02d.%02d.%02d",sTime.Hours,sTime.Minutes,sTime.Seconds);
		if(xSemaphoreTake(MutexOne, portMAX_DELAY)==pdTRUE){
			send_OLED(date, 10);
			send_OLED(time, 20);
			send_SDCard(date);
			vTaskDelay(5);
			send_SDCard(time);
			vTaskDelay(5);
			send_uart(date);
			send_uart(time);
			xSemaphoreGive(MutexOne);
		}
		vTaskDelay(10);
	}
}

void vMQ135task(void *pvParameters){
	while(1){
		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
		uint16_t adc_value= HAL_ADC_GetValue(&hadc2);
		HAL_ADC_Stop(&hadc2);
		sprintf(adcVal,"AQI is %d   ",adc_value);
		if(xSemaphoreTake(MutexOne, portMAX_DELAY)==pdTRUE){
			send_OLED(adcVal, 40);
			send_SDCard(adcVal);
			send_uart(adcVal);
			xSemaphoreGive(MutexOne);
		}
		vTaskDelay(20);
	}
}

void vTimerCallback(TimerHandle_t xTimer) {
    // Handle cases where the button is held but not released
    if (isButtonPressed) {
        pressEndTime = xTaskGetTickCount();
        uint32_t pressDurationMs = (pressEndTime - pressStartTime) * portTICK_PERIOD_MS;

        if (pressDurationMs > 3000) {
            send_uart("Help Resolved by Nurse\r\n");
        }
        isButtonPressed = pdFALSE;
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc){
    process_adc_buffer(&adc_buffer[0]); // We're half way through the buffer, so can safely deal with first half
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
    process_adc_buffer(&adc_buffer[ADC_SAMPLES * 2]); // We're all the way through the buffer, so deal with second half
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( binarySemaphoreISR, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
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
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM3) {
		if (tim_cnt % 500 == 0)
//				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		tim_cnt++;
	}

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
