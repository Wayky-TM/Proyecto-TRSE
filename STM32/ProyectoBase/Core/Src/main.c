/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "server.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define READ_RAW_ANALOG_VALUES	(1)

#define VREF (3.3f)

#define ANALOG_BUFFER_SIZE	(11)

#define DIR_CANAL_X (0)
#define DIR_CANAL_Y (1)
#define DIR_CANAL_Z (2)
#define DIR_CANAL_T (3)

#define LED_ANIMATION_1	(1)
#define LED_ANIMATION_2	(2)

#define ENABLE_LED_ANIMATION_1	(0x1)
#define ENABLE_LED_ANIMATION_2	(0x2)

#define MAX_ANIM_PERIOD_FACTOR	(10)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for SerialCommTask */
osThreadId_t SerialCommTaskHandle;
const osThreadAttr_t SerialCommTask_attributes = {
  .name = "SerialCommTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LedAnimation1 */
osThreadId_t LedAnimation1Handle;
const osThreadAttr_t LedAnimation1_attributes = {
  .name = "LedAnimation1",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LedAnimation2 */
osThreadId_t LedAnimation2Handle;
const osThreadAttr_t LedAnimation2_attributes = {
  .name = "LedAnimation2",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LedArrayMutex */
osMutexId_t LedArrayMutexHandle;
const osMutexAttr_t LedArrayMutex_attributes = {
  .name = "LedArrayMutex"
};
/* Definitions for LedAnimationEvents */
osEventFlagsId_t LedAnimationEventsHandle;
const osEventFlagsAttr_t LedAnimationEvents_attributes = {
  .name = "LedAnimationEvents"
};
/* USER CODE BEGIN PV */


volatile uint16_t ADC_DMA_buffer[4];

float analog_buffer[4][ANALOG_BUFFER_SIZE];
uint8_t current_sample_index;


uint8_t animation_period_factor=5;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
void StartMainTask(void *argument);
void StartLedAnimation1(void *argument);
void StartLedAnimation2(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void led_on(uint8_t v)
{
	switch (v)
	{
		case 0:
			HAL_GPIO_WritePin(D3_LED1_GPIO_Port, D3_LED1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D6_LED2_GPIO_Port, D6_LED2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D7_LED3_GPIO_Port, D7_LED3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D8_LED4_GPIO_Port, D8_LED4_Pin, GPIO_PIN_SET);
			break;
		case 1:
			HAL_GPIO_WritePin(D3_LED1_GPIO_Port, D3_LED1_Pin, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(D6_LED2_GPIO_Port, D6_LED2_Pin, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(D7_LED3_GPIO_Port, D7_LED3_Pin, GPIO_PIN_SET);
			break;
		case 4:
			HAL_GPIO_WritePin(D8_LED4_GPIO_Port, D8_LED4_Pin, GPIO_PIN_SET);
			break;

	}
}

void led_off(uint8_t v)
{
	switch (v)
	{
		case 0:
			HAL_GPIO_WritePin(D3_LED1_GPIO_Port, D3_LED1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D6_LED2_GPIO_Port, D6_LED2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D7_LED3_GPIO_Port, D7_LED3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D8_LED4_GPIO_Port, D8_LED4_Pin, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(D3_LED1_GPIO_Port, D3_LED1_Pin, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(D6_LED2_GPIO_Port, D6_LED2_Pin, GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(D7_LED3_GPIO_Port, D7_LED3_Pin, GPIO_PIN_RESET);
			break;
		case 4:
			HAL_GPIO_WritePin(D8_LED4_GPIO_Port, D8_LED4_Pin, GPIO_PIN_RESET);
			break;
	}
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
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_DMA_buffer, 4);
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of LedArrayMutex */
  LedArrayMutexHandle = osMutexNew(&LedArrayMutex_attributes);

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
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SerialCommTask */
  SerialCommTaskHandle = osThreadNew(StartMainTask, NULL, &SerialCommTask_attributes);

  /* creation of LedAnimation1 */
  LedAnimation1Handle = osThreadNew(StartLedAnimation1, NULL, &LedAnimation1_attributes);

  /* creation of LedAnimation2 */
  LedAnimation2Handle = osThreadNew(StartLedAnimation2, NULL, &LedAnimation2_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of LedAnimationEvents */
  LedAnimationEventsHandle = osEventFlagsNew(&LedAnimationEvents_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 209;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
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
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|D7_LED3_Pin|D8_LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D6_LED2_Pin|D3_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin D7_LED3_Pin D8_LED4_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|D7_LED3_Pin|D8_LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D6_LED2_Pin D3_LED1_Pin */
  GPIO_InitStruct.Pin = D6_LED2_Pin|D3_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : D5_EXTI_Pin */
  GPIO_InitStruct.Pin = D5_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(D5_EXTI_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch( GPIO_Pin )
	{
	case GPIO_PIN_15: // Botón de la placa Nucleo (+ velocidad)
		animation_period_factor = (animation_period_factor==1 ? MAX_ANIM_PERIOD_FACTOR : animation_period_factor-1);
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		break;

	case GPIO_PIN_4: // Botón externo (- velocidad)
		animation_period_factor = (animation_period_factor==MAX_ANIM_PERIOD_FACTOR ? 1 : animation_period_factor+1);
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		break;
	}
}


/* Obtención y filtrado de las muestras */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	current_sample_index = (current_sample_index+1)%ANALOG_BUFFER_SIZE;

	for( uint8_t i=0; i<4; ++i )
		analog_buffer[i][current_sample_index] = ADC_DMA_buffer[i];
}




/*
 * Comandos definidos
 *
 * */


int read_analog( uint8_t address, const union Data * tx, union Data * rx )
{

	if( address > 3 )
	{
		return CMD_INVALID_ADDR;
	}

#if !READ_RAW_ANALOG_VALUES
	float Axout, Ayout, Azout;

	Axout = (((analog_buffer[DIR_CANAL_X][current_sample_index] * VREF)/4095.0)-1.6)/0.32;
	Ayout = (((analog_buffer[DIR_CANAL_Y][current_sample_index] * VREF)/4095.0)-1.6)/0.32;
	Azout = (((analog_buffer[DIR_CANAL_Z][current_sample_index] * VREF)/4095.0)-1.7)/0.32;
#endif

	switch( address )
	{
	case DIR_CANAL_X:

#if READ_RAW_ANALOG_VALUES
		rx->F = (analog_buffer[DIR_CANAL_X][current_sample_index] * VREF)/4095.0;
#else
		rx->F = atan2(Axout,(sqrt(pow(Ayout,2)+pow(Azout,2))))*(180/PI); // En grados, formula sacada de internet
#endif
		break;

	case DIR_CANAL_Y:

#if READ_RAW_ANALOG_VALUES
		rx->F = (analog_buffer[DIR_CANAL_Y][current_sample_index] * VREF)/4095.0;
#else
		rx->F = atan2(Ayout,(sqrt(pow(Axout,2)+pow(Azout,2))))*(180.0/PI); // En grados, formula sacada de internet
#endif
		break;

	case DIR_CANAL_Z:

#if READ_RAW_ANALOG_VALUES
		rx->F = (analog_buffer[DIR_CANAL_Z][current_sample_index] * VREF)/4095.0;
#else
		rx->F = atan2((sqrt(pow(Axout,2)+pow(Ayout,2))),Azout)*(180.0/PI); // En grados, formula sacada de internet
#endif
		break;

	case DIR_CANAL_T:
		rx->F = (analog_buffer[DIR_CANAL_T][current_sample_index]*VREF)/4095.0;
		break;
	}

	return CMD_SUCCESS;
}



int switch_led_animations( uint8_t address, const union Data * tx, union Data * rx )
{

	switch( tx->W )
	{
	case LED_ANIMATION_1:
		osEventFlagsClear(LedAnimationEventsHandle, ENABLE_LED_ANIMATION_2);
		osEventFlagsSet(LedAnimationEventsHandle, ENABLE_LED_ANIMATION_1);
		break;

	case LED_ANIMATION_2:
		osEventFlagsClear(LedAnimationEventsHandle, ENABLE_LED_ANIMATION_1);
		osEventFlagsSet(LedAnimationEventsHandle, ENABLE_LED_ANIMATION_2);
		break;

	default:
		return CMD_INVALID_DATA;
	}

	return CMD_SUCCESS;
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if( UartHandle == &huart2 )
	{
		UART_notify_RxCplt();
	}
}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if( UartHandle == &huart2 )
	{
		UART_notify_TxCplt();
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the MainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN 5 */

	UART_register_command( 0x1, switch_led_animations );
	UART_register_command( 0x3, read_analog );

	UART_initialize( &huart2, 100 );

	UART_start_server();

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLedAnimation1 */
/**
* @brief Function implementing the LedAnimation1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedAnimation1 */
void StartLedAnimation1(void *argument)
{
  /* USER CODE BEGIN StartLedAnimation1 */

	osEventFlagsSet(LedAnimationEventsHandle, ENABLE_LED_ANIMATION_1);


	for(;;)
	{
		osEventFlagsWait(LedAnimationEventsHandle, ENABLE_LED_ANIMATION_1, osFlagsWaitAny|osFlagsNoClear, osWaitForever);
		osMutexAcquire(LedArrayMutexHandle, osWaitForever);


	  	led_off(0);
		for (uint8_t i=1; i<5; i++)
		{
			led_on(i);
			osDelay(80*animation_period_factor);
		}

		for (uint8_t i=1;i<5;i++)
		{
			led_off(i);
			osDelay(80*animation_period_factor);
		}

		led_on(0);
		osDelay(80*animation_period_factor);

		led_off(0);
		osDelay(80*animation_period_factor);

		osMutexRelease(LedArrayMutexHandle);
  }
  /* USER CODE END StartLedAnimation1 */
}

/* USER CODE BEGIN Header_StartLedAnimation2 */
/**
* @brief Function implementing the LedAnimation2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedAnimation2 */
void StartLedAnimation2(void *argument)
{
  /* USER CODE BEGIN StartLedAnimation2 */

	osEventFlagsClear(LedAnimationEventsHandle, ENABLE_LED_ANIMATION_2);

	for(;;)
	{
		osEventFlagsWait(LedAnimationEventsHandle, ENABLE_LED_ANIMATION_2, osFlagsWaitAny|osFlagsNoClear, osWaitForever);
		osMutexAcquire(LedArrayMutexHandle, osWaitForever);

		led_off(0);
		led_on(1);
		led_off(2);
		led_on(3);
		osDelay(100*animation_period_factor);

		led_on(0);
		led_off(1);
		led_on(2);
		led_off(3);
		osDelay(100*animation_period_factor);

		osMutexRelease(LedArrayMutexHandle);
	}
  /* USER CODE END StartLedAnimation2 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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

