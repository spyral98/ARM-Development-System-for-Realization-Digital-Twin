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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "data_logger.h"
#include "AT_commands.h"
#include "queue.h"
#include "thingspeak.h"
#include "TMP102.h"
#include "BME280.h"
#include "MPU6050.h"
#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define ADC_BUF_LEN 2048
#define MIC_BUF_LEN 1024
#define CUR_BUF_LEN 1024
#define IMU_BUF_LEN 64

typedef enum
{
	BUF_EMPTY,
	BUF_HALF_FULL,
	BUF_FULL
} BUF_STATE;

typedef struct
{
	float accel_x[IMU_BUF_LEN];
	float accel_y[IMU_BUF_LEN];
	float accel_z[IMU_BUF_LEN];
	float gyro_x[IMU_BUF_LEN];
	float gyro_y[IMU_BUF_LEN];
	float gyro_z[IMU_BUF_LEN];
} IMU_BUF;


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
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c3_rx;

TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* Definitions for usb_log_task */
osThreadId_t usb_log_taskHandle;
const osThreadAttr_t usb_log_task_attributes = {
  .name = "usb_log_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for motor_task */
osThreadId_t motor_taskHandle;
const osThreadAttr_t motor_task_attributes = {
  .name = "motor_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for wifi_task */
osThreadId_t wifi_taskHandle;
const osThreadAttr_t wifi_task_attributes = {
  .name = "wifi_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for proc_task */
osThreadId_t proc_taskHandle;
const osThreadAttr_t proc_task_attributes = {
  .name = "proc_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for heartbeat_timer */
osTimerId_t heartbeat_timerHandle;
const osTimerAttr_t heartbeat_timer_attributes = {
  .name = "heartbeat_timer"
};
/* USER CODE BEGIN PV */

uint8_t usb_rx_buffer[USB_RX_BUF_LEN];

uint16_t adc_raw_buf[ADC_BUF_LEN];
float mic_buf[MIC_BUF_LEN];
float cur_buf[CUR_BUF_LEN];

IMU_BUF imu_buf[2], imu_fft_buf;


float fft_mic_in_buf[MIC_BUF_LEN/2];
float fft_cur_in_buf[MIC_BUF_LEN/2];

float mic_fft_buf[MIC_BUF_LEN/2];
float cur_fft_buf[CUR_BUF_LEN/2];


IMU_BUF fft_in_imu;

float test_imu[6*IMU_BUF_LEN];


//QueueHandle_t logging_q;
BUF_STATE adc_buf_state = BUF_EMPTY;
BUF_STATE imu_buf_state = BUF_EMPTY;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C2_Init(void);
void usb_log_task_entry(void *argument);
void motor_task_entry(void *argument);
void wifi_task_entry(void *argument);
void proc_task_entry(void *argument);
void heartbeat_timer_callback(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM5_Init();
  MX_I2C3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  TMP102_init(&hi2c1);

  BME280_init(&hi2c3);



  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  	  Error_Handler();

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_raw_buf, ADC_BUF_LEN) != HAL_OK)
	  Error_Handler();

  if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
	  Error_Handler();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of heartbeat_timer */
  heartbeat_timerHandle = osTimerNew(heartbeat_timer_callback, osTimerPeriodic, NULL, &heartbeat_timer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  osTimerStart(heartbeat_timerHandle, 1000);

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  logging_q = xQueueCreate(64, sizeof(LOGGING_Q_ITEM));

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of usb_log_task */
  usb_log_taskHandle = osThreadNew(usb_log_task_entry, NULL, &usb_log_task_attributes);

  /* creation of motor_task */
  motor_taskHandle = osThreadNew(motor_task_entry, NULL, &motor_task_attributes);

  /* creation of wifi_task */
  wifi_taskHandle = osThreadNew(wifi_task_entry, NULL, &wifi_task_attributes);

  /* creation of proc_task */
  proc_taskHandle = osThreadNew(proc_task_entry, NULL, &proc_task_attributes);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
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
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x30A175AB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x307075B1;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart3.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart3.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_R_Pin|LED_G_Pin|LED_B_Pin|ESP_EN_Pin
                          |ESP_GPIO1_Pin|ESP_GPIO0_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MD_DIR_GPIO_Port, MD_DIR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MD_STEP_GPIO_Port, MD_STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, MD_MS2_Pin|MD_MS1_Pin|MD_NEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : MIC_TH_Pin */
  GPIO_InitStruct.Pin = MIC_TH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MIC_TH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_R_Pin LED_G_Pin LED_B_Pin ESP_EN_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_G_Pin|LED_B_Pin|ESP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ESP_GPIO1_Pin ESP_GPIO0_Pin */
  GPIO_InitStruct.Pin = ESP_GPIO1_Pin|ESP_GPIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MD_DIAG_Pin MD_INDEX_Pin */
  GPIO_InitStruct.Pin = MD_DIAG_Pin|MD_INDEX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : MD_DIR_Pin */
  GPIO_InitStruct.Pin = MD_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MD_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MD_STEP_Pin MD_MS2_Pin MD_MS1_Pin */
  GPIO_InitStruct.Pin = MD_STEP_Pin|MD_MS2_Pin|MD_MS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : MD_NEN_Pin */
  GPIO_InitStruct.Pin = MD_NEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MD_NEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C2_AD0_Pin */
  GPIO_InitStruct.Pin = I2C2_AD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(I2C2_AD0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C2_INT_Pin */
  GPIO_InitStruct.Pin = I2C2_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(I2C2_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C1_AD0_Pin I2C1_INT_Pin */
  GPIO_InitStruct.Pin = I2C1_AD0_Pin|I2C1_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	// Indicator that higher priority task has woken while we are in ISR
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	adc_buf_state = BUF_HALF_FULL;

	// Waking up task
	vTaskNotifyGiveFromISR(proc_taskHandle, &xHigherPriorityTaskWoken);
	// Run the scheduler upon exit of ISR
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	// Indicator that higher priority task has woken while we are in ISR
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	adc_buf_state = BUF_FULL;

	// Waking up task
	vTaskNotifyGiveFromISR(proc_taskHandle, &xHigherPriorityTaskWoken);
	// Run the scheduler upon exit of ISR
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
	// Indicator that higher priority task has woken while we are in ISR
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (hi2c == MPU6050.hi2c)
	{
		static uint32_t imu_index = 0;
		static uint8_t imu_buf_id = 0;

		MPU6050_convert_from_raw();

		imu_buf[imu_buf_id].accel_x[imu_index] = MPU6050.accel_x;
		imu_buf[imu_buf_id].accel_y[imu_index] = MPU6050.accel_y;
		imu_buf[imu_buf_id].accel_z[imu_index] = MPU6050.accel_z;

		imu_buf[imu_buf_id].gyro_x[imu_index] = MPU6050.gyro_x;
		imu_buf[imu_buf_id].gyro_y[imu_index] = MPU6050.gyro_y;
		imu_buf[imu_buf_id].gyro_z[imu_index] = MPU6050.gyro_z;

		imu_index++;
		if (imu_index == IMU_BUF_LEN)
		{
			if (imu_buf_id == 0)
			{
				imu_buf_state = BUF_HALF_FULL;
				imu_index = 0;
				imu_buf_id = 1;
			}
			else
			{
				imu_buf_state = BUF_FULL;
				imu_index = 0;
				imu_buf_id = 0;
			}

			vTaskNotifyGiveFromISR(proc_taskHandle, &xHigherPriorityTaskWoken);
		}
	}

	// Run the scheduler upon exit of ISR
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void compute_fft_mag(arm_rfft_fast_instance_f32* inst, float* buf_in, float* buf_out, uint32_t L)
{
	arm_rfft_fast_f32(inst, buf_in, buf_out, 0);
	arm_cmplx_mag_f32(buf_out, buf_out, L);

	for (int i=1; i<L/2; i++)
		buf_out[i] = (2.0f*buf_out[i]*buf_out[i]/L);

	buf_out[0] = (buf_out[0]*buf_out[0]/L);
}




/* USER CODE END 4 */

/* USER CODE BEGIN Header_usb_log_task_entry */
/**
  * @brief  Function implementing the usb_log_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_usb_log_task_entry */
void usb_log_task_entry(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */

  MPU6050_init(&hi2c2, &hdma_i2c2_rx);

  LOGGING_Q_ITEM current;
  static char id, signal = 0;

  /* Infinite loop */
  for(;;)
  {
	  if( xQueueReceive( logging_q, &current, portMAX_DELAY) == pdPASS )
	  {
		  signal = usb_rx_buffer[0];

		  if (signal == 'r')
		  {
			  switch (current.id)
			  {
			  case LOG_MIC:
				  id = 'm';
				  break;
			  case LOG_CUR:
				  id = 'c';
				  break;
			  case LOG_IMU:
				  id = 'i';
				  break;
			  case LOG_MIC_FFT:
				  id = 'M';
				  break;
			  case LOG_CUR_FFT:
				  id = 'C';
				  break;
			  case LOG_ACC_X_FFT:
				  id = '1';
				  break;
			  case LOG_ACC_Y_FFT:
				  id = '2';
				  break;
			  case LOG_ACC_Z_FFT:
				  id = '3';
				  break;
			  case LOG_GYR_X_FFT:
				  id = '4';
				  break;
			  case LOG_GYR_Y_FFT:
				  id = '5';
				  break;
			  case LOG_GYR_Z_FFT:
				  id = '6';
				  break;
			  }

			  while (CDC_Transmit_FS((uint8_t*)&id, 1) != USBD_OK)
				  osDelay(1);

			  while (CDC_Transmit_FS(current.buffer, current.size) != USBD_OK)
				  osDelay(1);

			  }

	  }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_motor_task_entry */
/**
* @brief Function implementing the motor_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor_task_entry */
void motor_task_entry(void *argument)
{
  /* USER CODE BEGIN motor_task_entry */

  /* Infinite loop */
	osDelay(3000);

	// Enabling the motor driver
	HAL_GPIO_WritePin(MD_NEN_GPIO_Port, MD_NEN_Pin, GPIO_PIN_RESET);
	osDelay(100);



  /* Infinite loop */
  for(;;)
  {
	 uint8_t dir;
	 dir = rand()%2;

	 // Setting a direction
	 if (dir == 0x00)
		 HAL_GPIO_WritePin(MD_DIR_GPIO_Port, MD_DIR_Pin, GPIO_PIN_RESET);
	 else
		 HAL_GPIO_WritePin(MD_DIR_GPIO_Port, MD_DIR_Pin, GPIO_PIN_SET);
	 osDelay(1);

	 int degrees;
	 degrees = rand()%200;

	 for (int i=0; i<degrees*16; i++)
	 {
	 	 HAL_GPIO_WritePin(MD_STEP_GPIO_Port, MD_STEP_Pin, GPIO_PIN_SET);
	     vTaskDelay((TickType_t)pdMS_TO_TICKS(1));
		 HAL_GPIO_WritePin(MD_STEP_GPIO_Port, MD_STEP_Pin, GPIO_PIN_RESET);
	     vTaskDelay((TickType_t)pdMS_TO_TICKS(1));
	 }


	  // Checking the diagnostic pin of the motor driver
	  if (HAL_GPIO_ReadPin(MD_DIAG_GPIO_Port, MD_DIAG_Pin) == GPIO_PIN_SET)
		  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
	  else
		  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);

	  osDelay(3000);
  }
  /* USER CODE END motor_task_entry */
}

/* USER CODE BEGIN Header_wifi_task_entry */
/**
* @brief Function implementing the wifi_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_wifi_task_entry */
void wifi_task_entry(void *argument)
{
  /* USER CODE BEGIN wifi_task_entry */
	AT_init(&huart3, &hdma_usart3_rx, wifi_taskHandle);

	TS_establish_connection("Alysandratos 2.4GHz", "a1b2c3d4e5");

	float buf[4] = {0};

  /* Infinite loop */
  for(;;)
  {
	  TMP102_read_temperature();
	  buf[0] = TMP102.current_temperature;

	  BME280_read_data();
	  buf[1] = BME280.press;
	  buf[2] = BME280.hum;
	  buf[3] = BME280.temp;

	  TS_send_data("13R16VK6AKMJNERT", buf, 4);
	  osDelay(15000);
  }
  /* USER CODE END wifi_task_entry */
}

/* USER CODE BEGIN Header_proc_task_entry */
/**
* @brief Function implementing the proc_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_proc_task_entry */
void proc_task_entry(void *argument)
{
  /* USER CODE BEGIN proc_task_entry */
	uint32_t L = MIC_BUF_LEN/2;
	arm_rfft_fast_instance_f32 adc_rfft_inst, imu_rfft_inst;

	arm_rfft_512_fast_init_f32(&adc_rfft_inst);
	arm_rfft_64_fast_init_f32(&imu_rfft_inst);


  /* Infinite loop */
  for(;;)
  {

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	if (adc_buf_state == BUF_HALF_FULL)
	{
		// Deinterleaving ADC data
		for (int i=0; i<MIC_BUF_LEN/2; i++)
		{
			mic_buf[i] = adc_raw_buf[2*i] * (2.5f / 4096);
			cur_buf[i] = adc_raw_buf[2*i+1] * (2.5f / 4096);
		}


		// Logging unprocessed
		log_via_USB(mic_buf, (MIC_BUF_LEN/2)*sizeof(float), LOG_MIC);
		log_via_USB(cur_buf, (CUR_BUF_LEN/2)*sizeof(float), LOG_CUR);

		// FFT
		memcpy(fft_mic_in_buf, mic_buf, L*sizeof(float));
		compute_fft_mag(&adc_rfft_inst, fft_mic_in_buf, mic_fft_buf, L);
		log_via_USB(mic_fft_buf, (L/2)*sizeof(float), LOG_MIC_FFT);


		memcpy(fft_cur_in_buf, cur_buf, L*sizeof(float));
		compute_fft_mag(&adc_rfft_inst, fft_cur_in_buf, cur_fft_buf, L);
		log_via_USB(cur_fft_buf, (L/2)*sizeof(float), LOG_CUR_FFT);


		adc_buf_state = BUF_EMPTY;
	}

	else if (adc_buf_state == BUF_FULL)
	{
		// Deinterleaving ADC data
		for (int i=MIC_BUF_LEN/2; i<MIC_BUF_LEN; i++)
		{
			mic_buf[i] = adc_raw_buf[2*i] * (2.5f / 4096);
			cur_buf[i] = adc_raw_buf[2*i+1] * (2.5f / 4096);
		}


		// Logging unprocessed
		log_via_USB(mic_buf + MIC_BUF_LEN/2, (MIC_BUF_LEN/2)*sizeof(float), LOG_MIC);
		log_via_USB(cur_buf + CUR_BUF_LEN/2, (CUR_BUF_LEN/2)*sizeof(float), LOG_CUR);

		// FFT
		memcpy(fft_mic_in_buf, mic_buf + MIC_BUF_LEN/2, L*sizeof(float));
		compute_fft_mag(&adc_rfft_inst, fft_mic_in_buf, mic_fft_buf, L);
		log_via_USB(mic_fft_buf, (L/2)*sizeof(float), LOG_MIC_FFT);


		memcpy(fft_cur_in_buf, cur_buf + CUR_BUF_LEN/2, L*sizeof(float));
		compute_fft_mag(&adc_rfft_inst, fft_cur_in_buf, cur_fft_buf, L);
		log_via_USB(cur_fft_buf, (L/2)*sizeof(float), LOG_CUR_FFT);

		adc_buf_state = BUF_EMPTY;
	}

	if (imu_buf_state == BUF_HALF_FULL)
	{

		log_via_USB(imu_buf[0].accel_x, sizeof(IMU_BUF), LOG_IMU);


		memcpy(fft_in_imu.accel_x, imu_buf[0].accel_x, sizeof(IMU_BUF));

		compute_fft_mag(&imu_rfft_inst, fft_in_imu.accel_x, imu_fft_buf.accel_x, IMU_BUF_LEN);
		log_via_USB(imu_fft_buf.accel_x, (IMU_BUF_LEN/2)*sizeof(float), LOG_ACC_X_FFT);

		compute_fft_mag(&imu_rfft_inst, fft_in_imu.accel_y, imu_fft_buf.accel_y, IMU_BUF_LEN);
		log_via_USB(imu_fft_buf.accel_y, (IMU_BUF_LEN/2)*sizeof(float), LOG_ACC_Y_FFT);

		compute_fft_mag(&imu_rfft_inst, fft_in_imu.accel_z, imu_fft_buf.accel_z, IMU_BUF_LEN);
		log_via_USB(imu_fft_buf.accel_z, (IMU_BUF_LEN/2)*sizeof(float), LOG_ACC_Z_FFT);

		compute_fft_mag(&imu_rfft_inst, fft_in_imu.gyro_x, imu_fft_buf.gyro_x, IMU_BUF_LEN);
		log_via_USB(imu_fft_buf.gyro_x, (IMU_BUF_LEN/2)*sizeof(float), LOG_GYR_X_FFT);

		compute_fft_mag(&imu_rfft_inst, fft_in_imu.gyro_y, imu_fft_buf.gyro_y, IMU_BUF_LEN);
		log_via_USB(imu_fft_buf.gyro_y, (IMU_BUF_LEN/2)*sizeof(float), LOG_GYR_Y_FFT);

		compute_fft_mag(&imu_rfft_inst, fft_in_imu.gyro_z, imu_fft_buf.gyro_z, IMU_BUF_LEN);
		log_via_USB(imu_fft_buf.gyro_z, (IMU_BUF_LEN/2)*sizeof(float), LOG_GYR_Z_FFT);


		imu_buf_state = BUF_EMPTY;
	}

	else if (imu_buf_state == BUF_FULL)
	{

		log_via_USB(imu_buf[1].accel_x, sizeof(IMU_BUF), LOG_IMU);


		memcpy(fft_in_imu.accel_x, imu_buf[1].accel_x, sizeof(IMU_BUF));

		compute_fft_mag(&imu_rfft_inst, fft_in_imu.accel_x, imu_fft_buf.accel_x, IMU_BUF_LEN);
		log_via_USB(imu_fft_buf.accel_x, (IMU_BUF_LEN/2)*sizeof(float), LOG_ACC_X_FFT);

		compute_fft_mag(&imu_rfft_inst, fft_in_imu.accel_y, imu_fft_buf.accel_y, IMU_BUF_LEN);
		log_via_USB(imu_fft_buf.accel_y, (IMU_BUF_LEN/2)*sizeof(float), LOG_ACC_Y_FFT);

		compute_fft_mag(&imu_rfft_inst, fft_in_imu.accel_z, imu_fft_buf.accel_z, IMU_BUF_LEN);
		log_via_USB(imu_fft_buf.accel_z, (IMU_BUF_LEN/2)*sizeof(float), LOG_ACC_Z_FFT);

		compute_fft_mag(&imu_rfft_inst, fft_in_imu.gyro_x, imu_fft_buf.gyro_x, IMU_BUF_LEN);
		log_via_USB(imu_fft_buf.gyro_x, (IMU_BUF_LEN/2)*sizeof(float), LOG_GYR_X_FFT);

		compute_fft_mag(&imu_rfft_inst, fft_in_imu.gyro_y, imu_fft_buf.gyro_y, IMU_BUF_LEN);
		log_via_USB(imu_fft_buf.gyro_y, (IMU_BUF_LEN/2)*sizeof(float), LOG_GYR_Y_FFT);

		compute_fft_mag(&imu_rfft_inst, fft_in_imu.gyro_z, imu_fft_buf.gyro_z, IMU_BUF_LEN);
		log_via_USB(imu_fft_buf.gyro_z, (IMU_BUF_LEN/2)*sizeof(float), LOG_GYR_Z_FFT);


		imu_buf_state = BUF_EMPTY;
	}


		/*
	process();
	log_processed();
	*/

  }
  /* USER CODE END proc_task_entry */
}

/* heartbeat_timer_callback function */
void heartbeat_timer_callback(void *argument)
{
  /* USER CODE BEGIN heartbeat_timer_callback */

	HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);

  /* USER CODE END heartbeat_timer_callback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM8 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM8) {
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
	  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
	  HAL_Delay(100);
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

