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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_i2c.h"
#include "keypad.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
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
    APP_STATE_STOPPED,
    APP_STATE_RUNNING,
    APP_STATE_PAUSED,
    APP_STATE_FINISHED
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
#define LIGHT_THRESHOLD         1500  // ?? ?? ? (???? ? ???? ??)
#define ADC_TIMEOUT_MS          100   // ADC ????
#define LIGHT_SAMPLE_COUNT      5     // ?? ???? ?? ?? ?

// ??? ??
#define DEFAULT_STUDY_TIME_S    1500  // ?? ?? ?? (25?)
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

/* USER CODE BEGIN PV */
// ?? ?? ??
// --- ?????? ?? ?? ---
static volatile AppState_t  g_app_state = APP_STATE_STOPPED;
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
static void App_Init(void);
static void Handle_KeypadInput(void);
static void Handle_SensorsAndEvents(void);
static void Handle_Buzzer(void);
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
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE BEGIN 3 */

    Handle_KeypadInput();       // ??? ?? ??
    Handle_SensorsAndEvents();  // ?? ?? ? ??? ??
    Handle_Buzzer();            // ?? ?? ??
    Handle_Display();           // LCD ????? ????

    HAL_Delay(10); // CPU ?? ??
    /* USER CODE END 3 */
  }
  /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure. */
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

  /** Initializes the CPU, AHB and APB buses clocks */
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) */
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

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. */
  sConfig.Channel = ADC_CHANNEL_0;      // GL55 ???? ?? (PA0)
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;  // ??? ??? ?? ??
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
  hi2c1.Init.ClockSpeed = 100000;                           // ?? I2C ?? (100kHz)
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
  htim2.Init.Prescaler = 8399;          // 84MHz / (8399+1) = 10kHz
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;             // 10kHz / (9999+1) = 1Hz (1??? ????)
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  huart2.Init.BaudRate = 115200;                    // ??? ??? ?? ???
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // ?? ? ???

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin (?? ??) */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 (KY-038 ??? ??? ??? ?) */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;       // ?? ???? ????
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;             // ??? ?? ???
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 (?? ???) */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;       // ??? ??
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 (??? ? ??) */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;           // ?? ??
  GPIO_InitStruct.Pull = GPIO_PULLUP;               // ?? ?? ???
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 (??? ? ??) */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;       // ??? ??
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);          // ??? ??? ???? ??
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
    LCD_Init();
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
// ??? ?? (Keypad Handler)
//==============================================================================
static void Handle_KeypadInput(void) {
    char key = Keypad_Read();
    if (key == 0) return;

    switch (key) {
        case 'A': Timer_Start(); break;
        case 'B': Timer_Stop(); break;
        case 'C': Timer_PauseResume(); break;
        case 'D':
            if (g_app_state == APP_STATE_FINISHED) {
                Timer_Stop();
            }
            break;
        default: break;
    }
    g_force_lcd_update = true;
    HAL_Delay(KEYPAD_DEBOUNCE_TIME_MS); // ? ?? ??? ?? ?? ?? ??
}

//==============================================================================
// ??? ?? (Timer Logic)
//==============================================================================
static void Timer_Start(void) {
    if (g_app_state == APP_STATE_STOPPED || g_app_state == APP_STATE_FINISHED) {
        g_app_state = APP_STATE_RUNNING;
        g_countdown_seconds = DEFAULT_STUDY_TIME_S;
        g_buzzer_state = BUZZER_OFF;
        Buzzer_Off();
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
    } else if (g_app_state == APP_STATE_PAUSED) {
        g_app_state = APP_STATE_RUNNING;
    }
}

static void Timer_Finish(void) {
    g_app_state = APP_STATE_FINISHED;
    g_buzzer_state = BUZZER_ALARMING;
    g_buzzer_timer = HAL_GetTick();
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

static void Handle_SensorsAndEvents(void) {
    static uint32_t last_sensor_read = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 1. ?? ?? ? ?? (500ms?? ? ??? ??? ??? ??)
    if (current_time - last_sensor_read >= 500) {
        uint32_t new_light_value = Read_Light_Sensor_Average();
        
        if (new_light_value > 0) {
            g_light_value = new_light_value;
            if (g_last_error == ERROR_ADC_FAILED) g_last_error = ERROR_NONE;
        } else {
            g_last_error = ERROR_ADC_FAILED;
        }
        
        last_sensor_read = current_time;
    }

    // 2. ?? ?? ???? ??? ?? (???)
    if (g_noise_interrupt_flag) {
        g_noise_interrupt_flag = false;
        
        if (g_buzzer_state == BUZZER_OFF) { // ??? ??? ?? ?? ??
            g_buzzer_state = BUZZER_BEEPING;
            g_buzzer_timer = HAL_GetTick();
        }
        
        if (!g_showing_noise_alert) {
            g_showing_noise_alert = true;
            g_noise_alert_start_time = HAL_GetTick();
            g_force_lcd_update = true;
        }
    }
}

//==============================================================================
// ?? ?? (Buzzer Handler & Control)
//==============================================================================
static void Buzzer_On(void)  { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); }
static void Buzzer_Off(void) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); }

static void Handle_Buzzer(void) {
    uint32_t current_time = HAL_GetTick();

    switch (g_buzzer_state) {
        case BUZZER_BEEPING:
            Buzzer_On();
            if (current_time - g_buzzer_timer >= BUZZER_BEEP_DURATION_MS) {
                Buzzer_Off();
                g_buzzer_state = BUZZER_OFF;
            }
            break;

        case BUZZER_ALARMING:
            if ((current_time - g_buzzer_timer) % (BUZZER_ALARM_ON_TIME_MS + BUZZER_ALARM_OFF_TIME_MS) < BUZZER_ALARM_ON_TIME_MS) {
                Buzzer_On();
            } else {
                Buzzer_Off();
            }
            break;

        case BUZZER_OFF:
        default:
            // Buzzer_Off()? ?? ?? ? ????, ??? ?? ?? ??
            break;
    }
}

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
            snprintf(buffer, sizeof(buffer), "Study: %02lu:%02lu   ", min, sec); 
            break;
        case APP_STATE_PAUSED:   
            snprintf(buffer, sizeof(buffer), "Pause: %02lu:%02lu   ", min, sec); 
            break;
        case APP_STATE_FINISHED: 
            snprintf(buffer, sizeof(buffer), "Time's Up!      "); 
            break;
        case APP_STATE_STOPPED:
        default:                 
            snprintf(buffer, sizeof(buffer), "Ready to study  "); 
            break;
    }
    LCD_SetCursor(0, 0);
    LCD_Print(buffer);
}

static void Display_StatusInfo(void) {
    char buffer[LCD_LINE_LENGTH + 1];

    if (g_app_state == APP_STATE_FINISHED) {
        snprintf(buffer, sizeof(buffer), "Press D to stop ");
    } else if (g_app_state == APP_STATE_STOPPED) {
        snprintf(buffer, sizeof(buffer), "Press A to start");
    } else {
        if (g_light_value < LIGHT_THRESHOLD) {
            snprintf(buffer, sizeof(buffer), "Light: Too Dark!");
        } else {
            snprintf(buffer, sizeof(buffer), "Light: OK (%4lu)", g_light_value > 9999 ? 9999 : g_light_value);
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        if (g_app_state == APP_STATE_RUNNING) {
            if (g_countdown_seconds > 0) {
                g_countdown_seconds--;
            } else { // countdown_seconds? 0? ? ??
                Timer_Finish();
            }
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    static uint32_t last_interrupt_time = 0;
    uint32_t current_time = HAL_GetTick();

    if (current_time - last_interrupt_time < DEBOUNCE_TIME_MS) {
        return;
    }
    last_interrupt_time = current_time;

    if (GPIO_Pin == GPIO_PIN_1) { // PA1 (???? DO ?)
        g_noise_interrupt_flag = true;
    }
}

/* USER CODE END 4 */

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