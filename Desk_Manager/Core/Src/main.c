/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Improved Desk Manager - Main program body
  * @description    : ??? ???, ?? ??, ?? ?? ??? ??? ?? ?? ???
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
#include "lcd_i2c.h"
#include "keypad.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "event_groups.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// ?? ?? ??
typedef enum {
    ERROR_NONE = 0,         // ?? ??
    ERROR_ADC_FAILED,       // ADC ?? ??
    ERROR_LCD_FAILED,       // LCD ?? ??
    ERROR_INVALID_PARAM     // ??? ?? ??
} error_code_t;

// ?????? ?? ??
typedef enum {
	APP_STATE_INIT,
    APP_STATE_STOPPED,
    APP_STATE_RUNNING,
    APP_STATE_PAUSED,
    APP_STATE_FINISHED,
	APP_STATE_BREAK_RUNNING,
	APP_STATE_BREAK_PAUSED
} AppState_t;

// ?? ?? ?? (State Machine)
typedef enum {
    BUZZER_OFF = 0,         // ?? ??
    BUZZER_BEEPING,         // ?? ??? ?? ?
    BUZZER_ALARMING         // ?? ??? ?? ?
} buzzer_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ?? ??
#define LIGHT_THRESHOLD         2000  // ?? ?? ? (???? ? ???? ??)
#define ADC_TIMEOUT_MS          100   // ADC ????
#define LIGHT_SAMPLE_COUNT      5     // ?? ???? ?? ?? ?

// ??? ??
#define DEFAULT_STUDY_TIME_S    10  // ?? ?? ?? (25?)
#define DEFAULT_BREAK_TIME_S    10
#define SECONDS_PER_MINUTE      60    // 1? = 60?
#define DEBOUNCE_TIME_MS        500   // ???? ?? ??

// ?? ??
#define BUZZER_BEEP_DURATION_MS   100   // ??? ?? ??
#define BUZZER_ALARM_ON_TIME_MS   200   // ?? ON ??
#define BUZZER_ALARM_OFF_TIME_MS  200   // ?? OFF ??

// LCD ??
#define LCD_UPDATE_INTERVAL_MS  1000  // LCD ???? ??
#define LCD_LINE_LENGTH         16    // LCD ?? ??
#define NOISE_ALERT_DURATION_MS 1000  // ?? ?? ??? ?? ??

// ??? ??
#define KEYPAD_DEBOUNCE_TIME_MS 200   // ??? ???? ?? ????

#define RED_PIN      GPIO_PIN_5
#define GREEN_PIN    GPIO_PIN_4
#define BLUE_PIN     GPIO_PIN_9

#define MAX_INPUT_DIGITS 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MINUTES_FROM_SECONDS(s) ((s) / SECONDS_PER_MINUTE)
#define SECONDS_REMAINDER(s)    ((s) % SECONDS_PER_MINUTE)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for KeypadTask */
osThreadId_t KeypadTaskHandle;
const osThreadAttr_t KeypadTask_attributes = {
  .name = "KeypadTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TimerTask */
osThreadId_t TimerTaskHandle;
const osThreadAttr_t TimerTask_attributes = {
  .name = "TimerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for BuzzerTask */
osThreadId_t BuzzerTaskHandle;
const osThreadAttr_t BuzzerTask_attributes = {
  .name = "BuzzerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lcdMutex */
osMutexId_t lcdMutexHandle;
const osMutexAttr_t lcdMutex_attributes = {
  .name = "lcdMutex"
};
/* USER CODE BEGIN PV */
// ?? ?? ??
// --- ?????? ?? ?? ---
static volatile AppState_t  g_app_state = APP_STATE_INIT;
static volatile uint32_t    g_countdown_seconds = DEFAULT_STUDY_TIME_S;
static volatile uint32_t    g_light_value = 0;
static volatile bool        g_noise_interrupt_flag = false;
static volatile error_code_t g_last_error = ERROR_NONE;

// --- ?? ?? ?? ---
static volatile buzzer_state_t g_buzzer_state = BUZZER_OFF;
static volatile uint32_t       g_buzzer_timer = 0;

// --- UI ???? ?? ---
static volatile uint32_t g_last_lcd_update_time = 0;
static volatile bool     g_force_lcd_update = true;
static volatile bool     g_showing_noise_alert = false;
static volatile uint32_t g_noise_alert_start_time = 0;
extern EventGroupHandle_t SoundAlertEventGate;
#define BIT_SOUND_ALERT (1 << 0)
static char time_input_buffer[MAX_INPUT_DIGITS + 1] = {0};
static uint8_t input_index = 0;
static bool is_setting_time = false;
uint32_t user_set_time_sec = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);

/* USER CODE BEGIN PFP */
uint32_t read_sound_level(void);
static void App_Init(void);
static void Handle_Display(void);

// --- ??? ?? ??? ---
static void Timer_Start(void);
static void Timer_Stop(void);
static void Timer_PauseResume(void);
static void Timer_Finish(void);

// --- ?? ?? ---
static void Buzzer_On(void);
static void Buzzer_Off(void);

// --- ?????(UI) ---
static void Display_TimerInfo(void);
static void Display_StatusInfo(void);
static void Clear_LcdLine(uint8_t line);
static uint32_t ParseTimeInput(const char* buffer);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  App_Init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of lcdMutex */
  lcdMutexHandle = osMutexNew(&lcdMutex_attributes);

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

  /* creation of KeypadTask */
  KeypadTaskHandle = osThreadNew(StartTask02, NULL, &KeypadTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartTask03, NULL, &SensorTask_attributes);

  /* creation of TimerTask */
  TimerTaskHandle = osThreadNew(StartTask04, NULL, &TimerTask_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(StartTask05, NULL, &DisplayTask_attributes);

  /* creation of BuzzerTask */
  BuzzerTaskHandle = osThreadNew(StartTask06, NULL, &BuzzerTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  MX_FREERTOS_Init();
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE BEGIN 3 */

    /* USER CODE END 3 */
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  sConfig.Channel = ADC_CHANNEL_0;
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

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, buzzer_Pin|Blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |Grenn_Pin|Red_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : buzzer_Pin Blue_Pin */
  GPIO_InitStruct.Pin = buzzer_Pin|Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15
                           Grenn_Pin Red_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |Grenn_Pin|Red_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void App_Init(void) {
    HAL_UART_Transmit(&huart2, (uint8_t*)"Starting App_Init...\r\n", 22, 1000);
    
    // ??? ???
    Keypad_Init();
    HAL_UART_Transmit(&huart2, (uint8_t*)"Keypad initialized\r\n", 20, 1000);
    
    // ?? ???
    Buzzer_Off();
    HAL_UART_Transmit(&huart2, (uint8_t*)"Buzzer initialized\r\n", 20, 1000);
    
    // LCD ??? (? ?? ??? ??!)
	if (LCD_Init() != HAL_OK) {
    g_last_error = ERROR_LCD_FAILED;
    Error_Handler();
}
    HAL_UART_Transmit(&huart2, (uint8_t*)"LCD Init called\r\n", 17, 1000);
    
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_Print("Desk Manager");
    LCD_SetCursor(1, 0);
    LCD_Print("Ready to Start!");
    HAL_UART_Transmit(&huart2, (uint8_t*)"LCD display set\r\n", 17, 1000);
    
    g_last_lcd_update_time = HAL_GetTick();
    HAL_UART_Transmit(&huart2, (uint8_t*)"System Initialized\r\n", 20, 1000);
}

//==============================================================================
// ??? ?? (Timer Logic)
//==============================================================================
static void Timer_Start(void) {
    if (g_app_state == APP_STATE_INIT ||
        g_app_state == APP_STATE_STOPPED ||
        g_app_state == APP_STATE_FINISHED ||
        g_app_state == APP_STATE_BREAK_RUNNING) {

        g_app_state = (g_app_state == APP_STATE_BREAK_RUNNING) ? APP_STATE_RUNNING : APP_STATE_RUNNING;
        g_countdown_seconds = user_set_time_sec;
        g_buzzer_state = BUZZER_OFF;
        Buzzer_Off();
        g_force_lcd_update = true;
    }
}

static void Timer_Stop(void) {
    g_app_state = APP_STATE_STOPPED;
    g_countdown_seconds = DEFAULT_STUDY_TIME_S;
    g_buzzer_state = BUZZER_OFF;
    Buzzer_Off();
}

static void Timer_PauseResume(void) {
	if (g_app_state == APP_STATE_RUNNING) {
	        g_app_state = APP_STATE_PAUSED;
	    }
	    else if (g_app_state == APP_STATE_PAUSED) {
	        g_app_state = APP_STATE_RUNNING;
	    }
	    else if (g_app_state == APP_STATE_BREAK_RUNNING) {   // ✅ Break도 일시정지
	        g_app_state = APP_STATE_BREAK_PAUSED;
	    }
	    else if (g_app_state == APP_STATE_BREAK_PAUSED) {    // ✅ Break 재개
	        g_app_state = APP_STATE_BREAK_RUNNING;
	    }
	}

static void Timer_Finish(void) {
	if (g_app_state == APP_STATE_RUNNING) {
	        g_app_state = APP_STATE_BREAK_RUNNING;
	        g_countdown_seconds = DEFAULT_BREAK_TIME_S;
	    }
	    else if (g_app_state == APP_STATE_BREAK_RUNNING) {
	        g_app_state = APP_STATE_RUNNING;
	        g_countdown_seconds = DEFAULT_STUDY_TIME_S;
	    }
	xEventGroupSetBits(SoundAlertEventGate, BIT_SOUND_ALERT);
    g_force_lcd_update = true;
}

//==============================================================================
// ?? ? ??? ?? (Sensor & Event Handler)
//==============================================================================
//==============================================================================
// ?? ? ??? ?? (Sensor & Event Handler)
//==============================================================================
static uint32_t Read_Light_Sensor_Average(void) {
    uint32_t total = 0;
    uint32_t valid_samples = 0;
	
   	ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_0;  // ?? ?? ??
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    
    for (int i = 0; i < LIGHT_SAMPLE_COUNT; i++) {
        if (HAL_ADC_Start(&hadc1) == HAL_OK) {
            if (HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT_MS) == HAL_OK) {
                total += HAL_ADC_GetValue(&hadc1);
                valid_samples++;
            }
            HAL_ADC_Stop(&hadc1);
        }
        HAL_Delay(5); // ?? ? ?? ??
    }
    
    return (valid_samples > 0) ? (total / valid_samples) : 0;
}


//==============================================================================
// ?? ?? (Buzzer Handler & Control)
//==============================================================================
static void Buzzer_On(void)  { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); }
static void Buzzer_Off(void) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); }

//==============================================================================
// ????? ?? (Display Handler & UI)
//==============================================================================
static void Handle_Display(void) {
    uint32_t current_time = HAL_GetTick();

    // ?? ?? ??? ?? ?? ? ?? ??
    if (g_showing_noise_alert && (current_time - g_noise_alert_start_time >= NOISE_ALERT_DURATION_MS)) {
        g_showing_noise_alert = false;
        g_force_lcd_update = true; // ?? ???? ??
    }
    
    // LCD ???? ?? ??
    if (!g_force_lcd_update && (current_time - g_last_lcd_update_time < LCD_UPDATE_INTERVAL_MS)) {
        return;
    }

    // ?? ????
    Display_TimerInfo(); // ? ?? ??? ??? ?? ??

    if (g_last_error == ERROR_ADC_FAILED) {
        LCD_SetCursor(1, 0);
        LCD_Print("Sensor Error!   ");
    } else if (g_showing_noise_alert) {
        LCD_SetCursor(1, 0);
        LCD_Print("Noise Alert!    ");
			  Buzzer_On();
        HAL_Delay(300);  // 0.3? ??
        Buzzer_Off();	
    } else {
        Display_StatusInfo(); // ? ?? ??? ?? ??
    }
    
    g_last_lcd_update_time = current_time;
    g_force_lcd_update = false;
}

static void Display_TimerInfo(void) {
    char buffer[LCD_LINE_LENGTH + 1];
    uint32_t min = MINUTES_FROM_SECONDS(g_countdown_seconds);
    uint32_t sec = SECONDS_REMAINDER(g_countdown_seconds);

    switch(g_app_state) {
        case APP_STATE_RUNNING:  
            snprintf(buffer, sizeof(buffer), "Study: %02lu:%02lu    ", (unsigned long)min, (unsigned long)sec);
            break;
        case APP_STATE_PAUSED:   
            snprintf(buffer, sizeof(buffer), "Pause: %02lu:%02lu    ", (unsigned long)min, (unsigned long)sec);
            break;
        case APP_STATE_FINISHED: 
            snprintf(buffer, sizeof(buffer), "Time's Up!      "); 
            break;
        case APP_STATE_BREAK_RUNNING:
                    snprintf(buffer, sizeof(buffer), "== Break Time!==  ");  // ✅ 상단 표시
                    break;
        case APP_STATE_BREAK_PAUSED:
        	        snprintf(buffer, sizeof(buffer), "== Break Time!==  ");
                    break;
        case APP_STATE_STOPPED:
        case APP_STATE_INIT:
        default:                 
            snprintf(buffer, sizeof(buffer), "Ready to study  "); 
            break;
    }
    LCD_SetCursor(0, 0);
    LCD_Print(buffer);
}

static void Display_StatusInfo(void) {
    char buffer[LCD_LINE_LENGTH + 1];

    if (is_setting_time) {
        LCD_SetCursor(0, 0);
        LCD_Print("Set Study Time");
        LCD_SetCursor(1, 0);
        char buffer[LCD_LINE_LENGTH + 1];
        snprintf(buffer, sizeof(buffer), "Set: %s", time_input_buffer);
        LCD_Print(buffer);
        return;  // 꼭 리턴!
    }
    if (g_app_state == APP_STATE_INIT) {
            LCD_SetCursor(0, 0);
            LCD_Print("Desk Manager     ");  // 라인 0
            LCD_SetCursor(1, 0);
            LCD_Print("Ready to Start!  ");  // 라인 1
            return;
        }

    if (g_app_state == APP_STATE_FINISHED) {
            snprintf(buffer, sizeof(buffer), "Press D to stop ");
        } else if (g_app_state == APP_STATE_STOPPED) {
            snprintf(buffer, sizeof(buffer), "Press A to start");
        } else if (g_app_state == APP_STATE_BREAK_RUNNING) {
            uint32_t min = MINUTES_FROM_SECONDS(g_countdown_seconds);
            uint32_t sec = SECONDS_REMAINDER(g_countdown_seconds);
            snprintf(buffer, sizeof(buffer), "Break: %02lu:%02lu    ", min, sec);
        } else if (g_app_state == APP_STATE_BREAK_PAUSED) {
            uint32_t min = MINUTES_FROM_SECONDS(g_countdown_seconds);
            uint32_t sec = SECONDS_REMAINDER(g_countdown_seconds);
            snprintf(buffer, sizeof(buffer), "BrkPause: %02lu:%02lu    ", min, sec);
        }
        else {
            if (g_light_value < LIGHT_THRESHOLD) {
                snprintf(buffer, sizeof(buffer), "Light: Too Dark!");
                RGB_SetColor(true, true, true);  // 예시: 조도 낮으면 RGB 모두 ON
            } else {
                snprintf(buffer, sizeof(buffer), "Light: OK (%4lu)", (g_light_value > 9999 ? 9999 : g_light_value));
                RGB_SetColor(false, false, false);  // 밝을 때는 꺼짐
            }
        }

    LCD_SetCursor(1, 0);
    LCD_Print(buffer);
}

static void Clear_LcdLine(uint8_t line) {
    LCD_SetCursor(line, 0);
    LCD_Print("                ");
}

//==============================================================================
// HAL ?? ?? (Callback Functions)
//==============================================================================



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	    if (GPIO_Pin == GPIO_PIN_1) { // 사운드센서 DO 핀
	        // 디바운스 처리 (선택적)
	    	static uint32_t last_interrupt_time = 0;
	    	        uint32_t current_time = HAL_GetTick();

	    	        if (current_time - last_interrupt_time < DEBOUNCE_TIME_MS) {
	    	            return;
	    	        }
	    	        last_interrupt_time = current_time;

	    	        g_showing_noise_alert = true;
	    	                g_noise_alert_start_time = current_time;
	    	                g_force_lcd_update = true;

	    	        // FreeRTOS 이벤트 그룹에 소리 감지 이벤트 전달
	    	        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) {
	    	                    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	    	                    xEventGroupSetBitsFromISR(SoundAlertEventGate, BIT_SOUND_ALERT, &xHigherPriorityTaskWoken);
	    	                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	    	                }
	    	    }
	    	}

uint32_t read_sound_level(void) {
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel = ADC_CHANNEL_1;  // PA1 = ADC1_IN1
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 50) == HAL_OK) {
    uint32_t sound_val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return sound_val;
  }
  HAL_ADC_Stop(&hadc1);
  return 0;
}
void RGB_SetColor(bool red, bool green, bool blue) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, red ? GPIO_PIN_SET : GPIO_PIN_RESET);  // Red
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, green ? GPIO_PIN_SET : GPIO_PIN_RESET); // Green
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, blue ? GPIO_PIN_SET : GPIO_PIN_RESET);  // Blue
}
static uint32_t ParseTimeInput(const char* buffer) {
	int input_time = atoi(buffer);  // MMSS 가정
	    int minutes = input_time / 100;
	    int seconds = input_time % 100;

    if (strlen(buffer) == 4 &&
        buffer[0] >= '0' && buffer[0] <= '9' &&
        buffer[1] >= '0' && buffer[1] <= '9' &&
        buffer[2] >= '0' && buffer[2] <= '9' &&
        buffer[3] >= '0' && buffer[3] <= '9') {

        minutes = (buffer[0] - '0') * 10 + (buffer[1] - '0');
        seconds = (buffer[2] - '0') * 10 + (buffer[3] - '0');

        if (seconds < 60) {
                user_set_time_sec = (minutes * 60) + seconds;
            } else {
                user_set_time_sec = DEFAULT_STUDY_TIME_S;
            }
    }

    return 0;  // 유효하지 않은 입력
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

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the KeypadTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	  char key = Keypad_Read();
	      if (key != 0) {

	        if (key >= '0' && key <= '9') {
	          if (input_index < MAX_INPUT_DIGITS) {
	            time_input_buffer[input_index++] = key;
	            time_input_buffer[input_index] = '\0';
	            is_setting_time = true;
	          }
	        }
	        else if (key == '#') {
	          input_index = 0;
	          time_input_buffer[0] = '\0';
	          is_setting_time = false;
	        }
	        else if (key == 'A') {
	            if (is_setting_time && input_index > 0) {
	                uint32_t parsed_time = ParseTimeInput(time_input_buffer);

	                if (user_set_time_sec > 0 && user_set_time_sec < 3600) {
	                    g_countdown_seconds = user_set_time_sec;
	                } else {
	                    g_countdown_seconds = DEFAULT_STUDY_TIME_S;
	                }

	                is_setting_time = false;
	                input_index = 0;
	                time_input_buffer[0] = '\0';
	            } else {
	                g_countdown_seconds = DEFAULT_STUDY_TIME_S;
	                user_set_time_sec = DEFAULT_STUDY_TIME_S;
	            }

	            Timer_Start();
	        }
	        else if (key == 'B') {
	          Timer_Stop();
	        }
	        else if (key == 'C') {
	          Timer_PauseResume();
	        }
	        else if (key == 'D') {
	          if (g_app_state == APP_STATE_FINISHED) Timer_Stop();
	        }

	        g_force_lcd_update = true;
	      }

	      osDelay(KEYPAD_DEBOUNCE_TIME_MS);
	    }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
	  uint32_t light = Read_Light_Sensor_Average();
	      g_light_value = light;

	      osDelay(200);

  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the TimerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {
	  if ((g_app_state == APP_STATE_RUNNING || g_app_state == APP_STATE_BREAK_RUNNING) && g_countdown_seconds > 0) {
	      g_countdown_seconds--;
	      if (g_countdown_seconds == 0) {
	          Timer_Finish();
	      }
	  }
	      osDelay(1000);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {
	  Handle_Display();  // 기존에 작성한 LCD 출력 함수 재활용
	      osDelay(LCD_UPDATE_INTERVAL_MS);
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the BuzzerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
  for(;;)
  {
	  // 1. 사운드 이벤트 수신 → 짧게 삑!
	      EventBits_t flags = xEventGroupWaitBits(
	          SoundAlertEventGate,
	          BIT_SOUND_ALERT,
	          pdTRUE,      // clear on exit
	          pdFALSE,     // wait for any bit
			  portMAX_DELAY);          // non-blocking check

	      if (flags & BIT_SOUND_ALERT) {
	        Buzzer_On();
	        osDelay(300);
	        Buzzer_Off();
	      }

	      // 2. 타이머 종료 상태 → 반복 알람
	      if (g_buzzer_state == BUZZER_ALARMING) {
	        Buzzer_On();
	        osDelay(BUZZER_ALARM_ON_TIME_MS);
	        Buzzer_Off();
	        osDelay(BUZZER_ALARM_OFF_TIME_MS);
	      }
	      else {
	        // 상태가 OFF이거나 BEEPING이면 짧은 대기 후 다음 루프
	        osDelay(50);
	      }
  }
  /* USER CODE END StartTask06 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* ?? ?? ? ?? ????? ?????? ?? ?? ?? */
  __disable_irq();
  while (1)
  {
    // ?? ??? LED ?? ?? ?? ?? ?? ?? ??
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* ??? ?? assert ?? ? ??? ??? ?? ?? ?? */
  /* printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
