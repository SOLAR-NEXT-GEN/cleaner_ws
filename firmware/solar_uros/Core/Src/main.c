/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include "PWM.h"
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EEPROM_ADDR 0x40
#define EEPROM_ADDR2 0x48
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c3_rx;
DMA_HandleTypeDef hdma_i2c3_tx;

UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_tx;
DMA_HandleTypeDef hdma_lpuart1_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
rcl_subscription_t subscriber_servo;
std_msgs__msg__Int32 msg_servo;

rcl_subscription_t subscriber_Brush;
std_msgs__msg__Int32 msg_Brush;

rcl_subscription_t subscriber_Water;
std_msgs__msg__Int32 msg_Water;

rcl_subscription_t subscriber_encoder1;
std_msgs__msg__Int32 msg_encoder1;

rcl_subscription_t subscriber_encoder2;
std_msgs__msg__Int32 msg_encoder2;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg_pub;

rcl_publisher_t omron_publisher;
std_msgs__msg__Int32 msg_Omron;

rcl_publisher_t encoder1_publisher;
//std_msgs__msg__Int32 msg_encoder1;

rcl_publisher_t encoder2_publisher;
//std_msgs__msg__Int32 msg_encoder2;

rcl_timer_t timer;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_init_options_t init_options;
rclc_executor_t executor;

int Brush;
int Water;
int Servo_switch = 0;
int BrushUD_mode = 0;
int prev_Servo = 0;
uint64_t a = 0;


PWM WaterPump;
PWM BrushMTR;
PWM BrushUD;
PWM BrushUD2;
int Omron;


static uint32_t timestamp_servo  = 0;
static uint32_t timestamp_servo2  = 0;
static uint32_t timestamp_servo3  = 0;
static uint32_t timestamp_servo4  = 0;
static uint32_t timestamp_omron  = 0;

static uint64_t timestamp = 0;
uint8_t eepromExampleWriteFlag = 0;
uint8_t eepromExampleReadFlag = 0;
uint8_t eepromDataReadBack[2];
char check[17] = {};
uint64_t encoder = 0;


uint8_t eepromExampleWriteFlag2 = 0;
uint8_t eepromExampleReadFlag2 = 0;
uint8_t eepromDataReadBack2[2];
char check2[17] = {};
uint64_t encoder2 = 0;

uint64_t encoder1_order = 0;
uint64_t encoder2_order = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void BrusheMotorControlled();
void WaterPumpControlled();
void BrushUpDownMode_manual();
void BrushUpDownMode_auto();
void Omron_check();
void encoder_check();

void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void subscription_callback_servo(const void * msgin);
void subscription_callback_Brush(const void * msgin);
void subscription_callback_Water(const void * msgin);
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
  MX_LPUART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  PWM_init(&BrushMTR, &htim2, TIM_CHANNEL_1);
  PWM_init(&WaterPump, &htim2, TIM_CHANNEL_2);
  PWM_init(&BrushUD2, &htim3, TIM_CHANNEL_1);
  PWM_init(&BrushUD, &htim3, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x40B285C2;
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
  hi2c3.Init.Timing = 0x40B285C2;
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
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 169;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA7 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	if (timer != NULL) {
		BrusheMotorControlled();
		BrushUpDownMode_manual();
		BrushUpDownMode_auto();
		Omron_check();
		WaterPumpControlled();


		eepromExampleReadFlag = 1;
		eepromExampleReadFlag2 = 1;
	  EEPROMReadExample(eepromDataReadBack);
	  EEPROMReadExample2(eepromDataReadBack2);
		timestamp = HAL_GetTick();
	  for (int i = 0; i < 16; i++) {
		  // Select byte: 0 for i=0..7, 1 for i=8..15
		  uint8_t byteIndex = i / 8;
		  uint8_t bitIndex = 7 - (i % 8);  // MSB first

		  check[i] = ((eepromDataReadBack[byteIndex] >> bitIndex) & 0x01) ? '1' : '0';
		  check2[i] = ((eepromDataReadBack2[byteIndex] >> bitIndex) & 0x01) ? '1' : '0';
	  }
	  check[16] = '\0';  // Null-terminate
	  check2[16] = '\0';
	  encoder = ((check[11] - 48) * 1000 ) + ((check[12] - 48) * 800 ) + ((check[13] - 48) * 400 )
		+ ((check[14] - 48) * 200 ) + ((check[15] - 48) * 100 ) + ((check[0] - 48) * 80 ) +
		((check[1] - 48) * 40 ) + ((check[2] - 48) * 20 ) + ((check[3] - 48) * 10 ) +
		((check[4] - 48) * 8 ) + ((check[5] - 48) * 4 ) + ((check[6] - 48) * 2 )
	 + ((check[7] - 48) * 1 ) ;
	  encoder2 = ((check2[11] - 48) * 1000 ) + ((check2[12] - 48) * 800 ) + ((check2[13] - 48) * 400 )
				+ ((check2[14] - 48) * 200 ) + ((check2[15] - 48) * 100 ) + ((check2[0] - 48) * 80 ) +
				((check2[1] - 48) * 40 ) + ((check2[2] - 48) * 20 ) + ((check2[3] - 48) * 10 ) +
				((check2[4] - 48) * 8 ) + ((check2[5] - 48) * 4 ) + ((check2[6] - 48) * 2 )
			 + ((check2[7] - 48) * 1 ) ;

	  //		((check[11] - 48) * 1000 )
	  //		((check[12] - 48) * 800 )
	  //		((check[13] - 48) * 400 )
	  //		((check[14] - 48) * 200 )
	  //		((check[15] - 48) * 100 )
	  //		((check[0] - 48) * 80 )
	  //      ((check[1] - 48) * 40 )
	  //		((check[2] - 48) * 20 )
	  //		((check[3] - 48) * 10 )
	  //	    ((check[4] - 48) * 8 )
	  //	    ((check[5] - 48) * 4 )
	  //	    ((check[6] - 48) * 2 )
	  //	    ((check[7] - 48) * 1 )

	  encoder_check();


	}
	rcl_ret_t ret = rcl_publish(&publisher, &msg_pub, NULL);
	if (ret != RCL_RET_OK)
	{
		NVIC_SystemReset();
	}
}
void BrushUpDownMode_auto()
{
	if (encoder - encoder1_order > 0) // threshold implement
		{
			if (HAL_GetTick() < timestamp_servo + 200)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1); // 1 == up , -1 == down
				PWM_write_duty(&BrushUD, 998, 50);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
				PWM_write_duty(&BrushUD2, 998, 50);

			}
			else{

				timestamp_servo = HAL_GetTick();
			}
		}
		else if (encoder - encoder1_order < 0)
		{
	//		static uint32_t timestamp_servo2  = 0;

			if (HAL_GetTick() < timestamp_servo2 + 200)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
				PWM_write_duty(&BrushUD, 998, 50);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
				PWM_write_duty(&BrushUD2, 998, 50);

			}
			else{

				timestamp_servo2 = HAL_GetTick();
			}
		}
		else if (encoder2 - encoder2_order > 0)
		{
			if (HAL_GetTick() < timestamp_servo + 200)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1); // 1 == up , -1 == down
				PWM_write_duty(&BrushUD, 998, 50);
			}
			else{
				timestamp_servo = HAL_GetTick();
			}
		}
		else if (encoder2 - encoder2_order < 0)
		{
	//		static uint32_t timestamp_servo2  = 0;

			if (HAL_GetTick() < timestamp_servo2 + 200)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
				PWM_write_duty(&BrushUD, 998, 50);
			}
			else{

				timestamp_servo2 = HAL_GetTick();
			}
		}
		else{
			timestamp_servo = HAL_GetTick();
			timestamp_servo2 = HAL_GetTick();
			timestamp_servo3 = HAL_GetTick();
			timestamp_servo4 = HAL_GetTick();
			PWM_write_duty(&BrushUD2, 998, 0);
			PWM_write_duty(&BrushUD, 998, 0);
		}


}
void BrusheMotorControlled()
{
	if (Brush)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, RESET);
		PWM_write_duty(&BrushMTR, 2000, 100);
	}
	else
	{
		PWM_write_duty(&BrushMTR, 2000, 0);
	}
}
void WaterPumpControlled()
{
	if (Water)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, RESET);
		PWM_write_duty(&WaterPump, 2000, 100);
	}
	else
	{
		PWM_write_duty(&WaterPump, 2000, 0);
	}
}
void BrushUpDownMode_manual()
{
	// PA10 DIR1 , PA4 PWM1
	// PA7 DIR2 ,PA6 PWM2

	if (Servo_switch == 1)
	{
		if (HAL_GetTick() < timestamp_servo + 200)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1); // 1 == up , -1 == down
			PWM_write_duty(&BrushUD, 998, 50);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
			PWM_write_duty(&BrushUD2, 998, 50);

		}
		else{
			Servo_switch = 0;
			timestamp_servo = HAL_GetTick();
		}
	}
	else if (Servo_switch == -1)
	{
//		static uint32_t timestamp_servo2  = 0;

		if (HAL_GetTick() < timestamp_servo2 + 200)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
			PWM_write_duty(&BrushUD, 998, 50);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
			PWM_write_duty(&BrushUD2, 998, 50);

		}
		else{
			Servo_switch = 0;
			timestamp_servo2 = HAL_GetTick();
		}
	}
	else if (Servo_switch == 2)
	{
		if (HAL_GetTick() < timestamp_servo + 200)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1); // 1 == up , -1 == down
			PWM_write_duty(&BrushUD, 998, 50);
		}
		else{
			Servo_switch = 0;
			timestamp_servo = HAL_GetTick();
		}
	}
	else if (Servo_switch == -2)
	{
//		static uint32_t timestamp_servo2  = 0;

		if (HAL_GetTick() < timestamp_servo2 + 200)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
			PWM_write_duty(&BrushUD, 998, 50);
		}
		else{
			Servo_switch = 0;
			timestamp_servo2 = HAL_GetTick();
		}
	}
	else if (Servo_switch == 3)
	{
		if (HAL_GetTick() < timestamp_servo + 200)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0); // 1 == up , -1 == down
			PWM_write_duty(&BrushUD2, 998, 50);
		}
		else{
			Servo_switch = 0;
			timestamp_servo = HAL_GetTick();
		}
	}
	else if (Servo_switch == -3)
	{
//		static uint32_t timestamp_servo2  = 0;

		if (HAL_GetTick() < timestamp_servo2 + 200)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
			PWM_write_duty(&BrushUD2, 998, 50);
		}
		else{
			Servo_switch = 0;
			timestamp_servo2 = HAL_GetTick();
		}
	}
	else{
		timestamp_servo = HAL_GetTick();
		timestamp_servo2 = HAL_GetTick();
		timestamp_servo3 = HAL_GetTick();
		timestamp_servo4 = HAL_GetTick();
		PWM_write_duty(&BrushUD2, 998, 0);
		PWM_write_duty(&BrushUD, 998, 0);
	}

}

void Omron_check()
{
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 1)
	{
		if (HAL_GetTick() > timestamp_omron + 2000){
			std_msgs__msg__Int32 msg;

			// Set message value
			msg.data = 1;
			Omron = 1;
			// Publish message
			rcl_publish(&omron_publisher, &msg, NULL);
			timestamp_omron = HAL_GetTick();
		}
	}
	else
	{
		timestamp_omron  = HAL_GetTick();
		std_msgs__msg__Int32 msg;

		// Set message value
		msg.data = 0;
		Omron = 0;
		// Publish message
		rcl_publish(&omron_publisher, &msg, NULL);
	}
}

void encoder_check()
{
			std_msgs__msg__Int32 msg1;
			std_msgs__msg__Int32 msg2;

			// Set message value
			msg1.data = encoder;
			msg2.data = encoder2;
			// Publish message
			rcl_publish(&encoder1_publisher, &msg1, NULL);
			rcl_publish(&encoder2_publisher, &msg2, NULL);
}

void subscription_callback_servo(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	Servo_switch = msg->data;
	// 0 = do nothing
	// 1 and -1 , 1 == 1 up , -1 == 1 down
	// 2 and -2
}

void subscription_callback_Brush(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	Brush = msg->data;
}

void subscription_callback_Water(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	Water = msg->data;
}
void subscription_callback_encoder1(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	encoder1_order = msg->data;
}

void subscription_callback_encoder2(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	encoder2_order = msg->data;
}

void EEPROMReadExample(uint8_t *Rdata) {
	if (eepromExampleReadFlag && hi2c1.State == HAL_I2C_STATE_READY) {
//		HAL_I2C_Mem_Read_IT(&hi2c1, 0x40, 0x12, I2C_MEMADD_SIZE_8BIT, Rdata, 2);
		HAL_I2C_Mem_Read_IT(&hi2c1, EEPROM_ADDR, 0x12, I2C_MEMADD_SIZE_8BIT, Rdata, 2);
//		HAL_I2C_Mem_Read_IT(&hi2c1, EEPROM_ADDR, 0x12, I2C_MEMADD_SIZE_16BIT, Rdata, 2);
		eepromExampleReadFlag = 0;


	}
}
void EEPROMReadExample2(uint8_t *Rdata) {
	if (eepromExampleReadFlag2 && hi2c3.State == HAL_I2C_STATE_READY) {
//		HAL_I2C_Mem_Read_IT(&hi2c1, 0x40, 0x12, I2C_MEMADD_SIZE_8BIT, Rdata, 2);
		HAL_I2C_Mem_Read_IT(&hi2c3, EEPROM_ADDR2, 0x12, I2C_MEMADD_SIZE_8BIT, Rdata, 2);
//		HAL_I2C_Mem_Read_IT(&hi2c1, EEPROM_ADDR, 0x12, I2C_MEMADD_SIZE_16BIT, Rdata, 2);
		eepromExampleReadFlag2 = 0;


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
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
		rmw_uros_set_custom_transport(
		true,
		(void *) &hlpuart1,
		cubemx_transport_open,
		cubemx_transport_close,
		cubemx_transport_write,
		cubemx_transport_read);

		rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
		freeRTOS_allocator.allocate = microros_allocate;
		freeRTOS_allocator.deallocate = microros_deallocate;
		freeRTOS_allocator.reallocate = microros_reallocate;
		freeRTOS_allocator.zero_allocate = microros_zero_allocate;

		if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		  printf("Error on default allocators (line %d)\n", __LINE__);
		}

		// micro-ROS app
		allocator = rcl_get_default_allocator();

		init_options = rcl_get_zero_initialized_init_options();
		rcl_init_options_init(&init_options, allocator);
		rcl_init_options_set_domain_id(&init_options, 1);

		// Initialize rclc support object with custom options
		rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

		// Create node
		rclc_node_init_default(&node, "cubemx_node", "", &support);

		// Create timer
		rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10), timer_callback);

		// Create subscriber
		rclc_subscription_init_default(
			&subscriber_servo,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			"cubemx_publisher_servo");

		rclc_subscription_init_default(
			&subscriber_Brush,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			"cubemx_publisher_Brush");

		rclc_subscription_init_default(
			&subscriber_Water,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			"cubemx_publisher_Water");

		// create publisher
		rclc_publisher_init_default(
			&publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			"cubemx_publisher");

		rclc_publisher_init_default(
			&omron_publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			"cubemx_publisher_Omron");

		rclc_publisher_init_default(
			&encoder1_publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			"cubemx_publisher_encoder1");

		rclc_publisher_init_default(
			&encoder2_publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			"cubemx_publisher_encoder2");


		msg_pub.data = 0;

		// Initialize the executor
		executor = rclc_executor_get_zero_initialized_executor();
		rclc_executor_init(&executor, &support.context, 6, &allocator);
		rclc_executor_add_timer(&executor, &timer);
		rclc_executor_add_subscription(&executor, &subscriber_servo, &msg_servo, subscription_callback_servo, ON_NEW_DATA);
		rclc_executor_add_subscription(&executor, &subscriber_Brush, &msg_Brush, subscription_callback_Brush, ON_NEW_DATA);
		rclc_executor_add_subscription(&executor, &subscriber_Water, &msg_Water, subscription_callback_Water, ON_NEW_DATA);

		rclc_executor_add_subscription(&executor, &subscriber_encoder1, &msg_encoder1, subscription_callback_encoder1, ON_NEW_DATA);
		rclc_executor_add_subscription(&executor, &subscriber_encoder2, &msg_encoder2, subscription_callback_encoder2, ON_NEW_DATA);
		rclc_executor_spin(&executor);

		for(;;)
		{
			osDelay(10);
		}
  /* USER CODE END 5 */
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
