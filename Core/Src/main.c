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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
//#include <stdio.h>
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;

/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
  .name = "Task1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Task2 */
osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes = {
  .name = "Task2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Task3 */
osThreadId_t Task3Handle;
const osThreadAttr_t Task3_attributes = {
  .name = "Task3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task04 */
osThreadId_t Task04Handle;
const osThreadAttr_t Task04_attributes = {
  .name = "Task04",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
void IMU_Batter(void *argument);
void ThrottleADC(void *argument);
void LED_Control(void *argument);
void WriteBluetooth(void *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// IMU addresses
//static const uint8_t IMU_ADDR = 0x4A << 1;

// Bluetooth variables
uint8_t rx_char;
uint8_t UART4_rxBuffer[20] = {0};
int i = 0;

// Throttle variables
uint16_t throttle;

// Speedometer variables
uint32_t speed = 0;
uint32_t tick = 0;
const float hrs = 2.7777777777778E-7;
const float wheel_circum_miles = (2070/1609344)/6;

// LED variables
uint8_t red = 0;
uint8_t green = 0;
uint8_t blue = 0;
// 0 - 0
// 1 - 85
// 2 - 170
// 3 - 255

// Mode flags
uint8_t mode = 0;
uint8_t mode_changed = 0;
// mode 0 - idle
// mode 1 - throttle
// mode 2 - cruise

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart5.Instance)
	{
		if(rx_char == '\r' || rx_char == '\n')
		{
			HAL_UART_Receive_IT(&huart5, &rx_char, 1);
			return;
		}

		// Retrieve mode and set flag
		uint8_t new_mode = rx_char & 0x03;
		if (new_mode != mode)
		{
			mode = new_mode;
			mode_changed = 1;
		}
//		mode = rx_char & 0x03;

		// Retrieve red
		red = (rx_char >> 6) & 0x03;

		// Retrieve green
		green = (rx_char >> 4) & 0x03;

		// Retrieve blue
		blue = (rx_char >> 2) & 0x03;




//		if(rx_char == '1')
//		{
////			rx_char = '\0';
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
//		}
//		else if (rx_char == '0')
//		{
////			rx_char = '\0';
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//			memset(UART4_rxBuffer, 0, i);
//			i=0;
//
//		}
//		else if (rx_char == 'a')
//		{
////			rx_char = '\0';
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
//		}
//		else
//		{
//
//		}

		HAL_UART_Receive_IT(&huart5, &rx_char, 1);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
	if (GPIO_PIN == GPIO_PIN_1)
	{
//		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		// Calculate speed
		speed = HAL_GetTick() - tick;
		tick = HAL_GetTick();
	}
	else if (GPIO_PIN == GPIO_PIN_2 || GPIO_PIN == GPIO_PIN_0)
	{
		mode++;
		if (mode == 2)
		{
			mode = 0;
		}
		mode_changed = 1;
	}
	else
	{
		__NOP();
	}
}

uint8_t convert_color(uint8_t color)
{
	switch (color)
	{
		case 0:
			return 0;
		break;
		case 1:
			return 85;
		break;
		case 2:
			return 170;
		break;
		case 3:
			return 255;
		break;
		default:
			return 0;
		break;
	}

	return 0;
}

void mode_change_ack(uint8_t changed_mode)
{
	if (changed_mode == 1)
	{
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 255);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 255);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 255);
		osDelay(500);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
		osDelay(500);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 255);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 255);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 255);
		osDelay(500);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
		osDelay(500);

		mode_changed = 0;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_UART5_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
//  HAL_UART_Receive_DMA(&huart5, UART4_rxBuffer, sizeof(UART4_rxBuffer));
  HAL_UART_Receive_IT(&huart5, &rx_char, 1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of Task1 */
  Task1Handle = osThreadNew(IMU_Batter, NULL, &Task1_attributes);

  /* creation of Task2 */
  Task2Handle = osThreadNew(ThrottleADC, NULL, &Task2_attributes);

  /* creation of Task3 */
  Task3Handle = osThreadNew(LED_Control, NULL, &Task3_attributes);

  /* creation of Task04 */
  Task04Handle = osThreadNew(WriteBluetooth, NULL, &Task04_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 39;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 40960;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 390;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 256;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : mode_button_Pin */
  GPIO_InitStruct.Pin = mode_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(mode_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_IMU_Batter */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_IMU_Batter */
void IMU_Batter(void *argument)
{
  /* USER CODE BEGIN 5 */

//	shtpData[0] = 0xF9;
//	shtpData[1] = 0;
//
//	sendPacket(2, 2);
//	receivePacket();


	/* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

//			strcpy((char*)buf, "A OK\r\n");
//			HAL_UART_Transmit(&huart5, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

//		shtpData[0] = 0x02;
//		shtpData[1] = 0x00;
//		sendPacket(5, 2);
//		receivePacket();

//		uint8_t dataBuffer[20] = {0x06, 0x00, 0x02, 0x00, 0xF9, 0x00};
//		HAL_I2C_Master_Transmit(&hi2c1, IMU_ADDR, dataBuffer, 6, HAL_MAX_DELAY);
//
////		buf[0] = 0x01 << 1;
//		HAL_I2C_Master_Receive(&hi2c1, IMU_ADDR, recv, 4, HAL_MAX_DELAY);
//		HAL_UART_Transmit(&huart5, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
		osDelay(1000);
	}

  // In case we accidentally exit loop
  osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ThrottleADC */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ThrottleADC */
void ThrottleADC(void *argument)
{
  /* USER CODE BEGIN ThrottleADC */
	uint16_t raw;
	int32_t CH1_DC = 0;
//	char msg[10];
	int output_end = 4095;
	int output_start = 100;
	int input_end = 3120;
	int input_start = 1180;
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	  for(;;)
	  {

		  // Idle mode
		  if (mode == 0)
		  {
			  if(mode_changed == 1)
			  {
				  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			  }
			  mode_change_ack(mode_changed);
		  }
		  // Throttle mode
		  else if (mode == 1)
		  {
			  if(mode_changed == 1)
			  {
				  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			  }
			  mode_change_ack(mode_changed);
		  }

		  // Set pin high
//			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);

		  // Get ADC values
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		  raw = HAL_ADC_GetValue(&hadc1);

		  double slope = 1.0 * (output_end - output_start) / (input_end - input_start);
		  throttle = output_start + slope * (raw - input_start);
//		  throttle = raw;

		  CH1_DC = throttle;
		  // set pin low
//			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

		  // Set analog to output
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, CH1_DC);

		  // convert to string and print
//		  sprintf(msg, "%hu\r\n", raw);
//		  HAL_UART_Transmit(&huart5, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		  osDelay(500);
	  }

	  // In case we accidentally exit loop
	  osThreadTerminate(NULL);
  /* USER CODE END ThrottleADC */
}

/* USER CODE BEGIN Header_LED_Control */
/**
* @brief Function implementing the Task3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED_Control */
void LED_Control(void *argument)
{
  /* USER CODE BEGIN LED_Control */

	uint8_t PWM_red = 0;
	uint8_t PWM_green = 0;
	uint8_t PWM_blue = 0;

	//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	/* Infinite loop */
	for(;;)
	{
		PWM_red = convert_color(red);
		PWM_green = convert_color(green);
		PWM_blue = convert_color(blue);

//		TIM4->CCR4 = CH4_DC;
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, CH4_DC);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, PWM_red);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_green);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM_blue);

		osDelay(1000);
	}

	// In case we accidentally exit loop
	osThreadTerminate(NULL);
  /* USER CODE END LED_Control */
}

/* USER CODE BEGIN Header_WriteBluetooth */
/**
* @brief Function implementing the Task04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WriteBluetooth */
void WriteBluetooth(void *argument)
{
  /* USER CODE BEGIN WriteBluetooth */
	//	uint8_t Test[20] = "Hello World !!!\n";
	//	uint8_t Rece[20];
	uint8_t buffer[40];
//	uint16_t speed = 0;
	uint16_t batt = 50;
//	uint16_t raw;

	uint32_t test_speed = 814;

	  /* Infinite loop */
		  for(;;)
		  {
	//	    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	//	    osDelay(250);



	//	    HAL_UART_Transmit(&huart5, Test, sizeof(Test), 10)
	//	    if (HAL_UART_Transmit_IT(&huart5, Test, sizeof(Test)) == HAL_OK)
//			float wheel_speed = ((wheel_circum_miles)/6)/((float)speed*hrs);
//			float wheel_speed = (771.74302075876)/((float)speed);
//			float wheel_speed = (814.2)/((float)speed);
//			double wheel_speed = 12.11111111111;
//			sprintf(buffer, "speed:%u\n", speed);
//			sprintf(buffer, "speed:%02.2f\n", wheel_speed);
//			sprintf(buffer, "speed:%02.2f,mode:%hu\n", wheel_speed, mode);
//			sprintf(buffer, "mode:%hu,red:%hu,green:%hu,blue:%hu\n", mode, red, green, blue);
			sprintf(buffer, "mode:%hu,speed:%u\n", mode, speed);
//			sprintf(buffer, "speed:%f,battery:%hu,input:%hi\n", speed, batt, throttle);
//			sprintf(buffer, "speed:%hu,battery:%hu", speed, batt);
//			if (HAL_UART_Transmit(&huart5, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY) == HAL_OK)
			if (HAL_UART_Transmit(&huart5, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY) == HAL_OK)
			{
				osDelay(100);
				sprintf(buffer, "throttle:%hu\n", throttle);
				HAL_UART_Transmit(&huart5, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//				osDelay(100);
//				sprintf(buffer, "speed:%u\n", test_speed);
//				HAL_UART_Transmit(&huart5, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
			}
//		    if (HAL_UART_Transmit(&huart5, Test, sizeof(Test), HAL_MAX_DELAY) == HAL_OK)
//		    {
//		    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
//		    }
	//	    else if (HAL_UART_Transmit_IT(&huart5, Test, sizeof(Test)) == HAL_ERROR)
		    else
		    {
		    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		    }
		    osDelay(100);

		    if (UART4_rxBuffer[0] == '\n')
		    {
		    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		    	osDelay(5000);
	//	    	memset(UART4_rxBuffer, '\0', sizeof(UART4_rxBuffer));
		    }

	//	    while(HAL_UART_Receive(&huart5, Rece, sizeof(Rece), HAL_MAX_DELAY) == HAL_OK)
	//	    {
	//	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
	//	    	memcpy(&Test, &Rece, sizeof(Test));
	//	    }
//			osDelay(10);
		    batt++;
		    if(batt > 100)
		    	batt = 0;
		    test_speed -= 10;
		    if(test_speed < 10)
		    	test_speed = 814;
		  }

		  // In case we accidentally exit loop
		  osThreadTerminate(NULL);

  /* USER CODE END WriteBluetooth */
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
