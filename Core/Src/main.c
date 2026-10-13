/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* Button state */
#define BUTTON_RELEASED                    0U
#define BUTTON_PRESSED                     1U
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* -------------------------------------------------*/
/* -- Aufgabenteil B -- **/
/* -------------------------------------------------*/

/* -- Messung der Spannung -- */
static float cur_voltage = 0.0f; // variable for STM32Cube Live Watch
static float cur_voltage_avg = 0.0f; // variable for STM32Cube Live Watch
static uint32_t adc_reading_int = 0.0f; // variable for STM32Cube Live Watch
static float avg_adc_reading_float = 0.0f; // variable for STM32Cube Live Watch
static uint32_t avg_adc_reading_int = 0.0f; // variable for STM32Cube Live Watch

uint32_t read_adc(void) {
  uint32_t reading;

  // Start the ADC conversion from an analog to a digital signal
  HAL_ADC_Start(&hadc1);

  // Wait for the conversion to complete with a timeout of 100ms
  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
      // Get the raw 12-bit value (0 - 4095)
      reading = HAL_ADC_GetValue(&hadc1);

      // Stop the ADC
      HAL_ADC_Stop(&hadc1);

      return reading;

  } else {

    // Stop the ADC after timeout
    HAL_ADC_Stop(&hadc1);
    return 0;
  }
}

/* -- gleitender Mittelwert Filter -- **/
// Window size (N)
#define WINDOW 32
static uint32_t mov_avg_index = 0;

// Moving average buffer and sum for int
static uint32_t buffer_int[WINDOW] = {0};
static uint32_t sum_int = 0;

// Int-Filter
uint32_t read_adc_avg_int(void) {

  // Start the ADC conversion from an analog to a digital signal
  HAL_ADC_Start(&hadc1);

  // Wait for the conversion to complete with a timeout of 100ms
  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
  
    uint32_t raw_value = HAL_ADC_GetValue(&hadc1);

    // Calculate current moving average:
    sum_int -= buffer_int[mov_avg_index];           // subtract oldest value in window from the sum
    buffer_int[mov_avg_index] = raw_value;          // add new value to the window
    sum_int += raw_value;                           // add the new value to the sum
    mov_avg_index = (mov_avg_index + 1) % WINDOW;   // loop through the window with modolu

    HAL_ADC_Stop(&hadc1);

    return (sum_int >> 5); // because WINDOW = 32 = 2^5 --> right shift by 5
  
  } else {

    // Stop the ADC after timeout
    HAL_ADC_Stop(&hadc1);
    return (sum_int >> 5);
  }
}

// Moving average buffer and sum for float
static float buffer_float[WINDOW] = {0.0f};
static float sum_float = 0.0f;

// Float-Filter
float read_adc_avg_float(void) {

  // Start the ADC conversion from an analog to a digital signal
  HAL_ADC_Start(&hadc1);

  // Wait for the conversion to complete with a timeout of 100ms
  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
  
    float raw_value = HAL_ADC_GetValue(&hadc1);

    // Calculate current moving average:
    sum_float -= buffer_float[mov_avg_index];          // subtract oldest value in window from the sum
    buffer_float[mov_avg_index] = raw_value;           // add new value to the window
    sum_float += raw_value;                            // add the new value to the sum
    mov_avg_index = (mov_avg_index + 1) % WINDOW;      // loop through the window, by going back to 0 with modulo

    HAL_ADC_Stop(&hadc1);

    return (sum_float / WINDOW);
  
  } else {

    // Stop the ADC after timeout
    HAL_ADC_Stop(&hadc1);
    return (sum_float / WINDOW);
  }
}


/* -- Toggle der LED bei bestimmter Spannung -- */
void toggle_led(void) {

      // uint32_t value = read_adc();      // Take current adc reading
      uint32_t value = read_adc_avg_int();  // Take average adc reading
      
      // Observation Note: 
      // when the read value (single) are near the threshhold (2048) the LED will flicker, because the value fluctuates
      // when the avg is read, the LED will flicker less the greater the window
      
      // Check if value from ADC is above 2048, if true -> set pin, otherwise reset (LED on/off)
      if(value >= 2048) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
      }
}

float convert_to_voltage(uint32_t adc_reading) {
    // Convert value to voltage (3.3V reference voltage and 12-bit ADC resolution)
    return (float)adc_reading * (3.3f / 4095.0f);
}

/* ------------------------------------------ */
/* -- Aufgabenteil C -- */
/* -------------------------------------------*/

/* -- LED Dimmer / PWM -- */
#define ADC_MAX 4095u // max adc (here 12-Bit)
#define PWM_ARR 999u  // counter period (set in cubemx) -> 1000 duty steps

// Controlling brightness with average adc value
void led_brightness_pwm() {

  // Get current adc average
  uint32_t cur_adc_avg = read_adc_avg_int();

  // Calculate value to be compared
  uint32_t capture_compare_reg = (cur_adc_avg * (PWM_ARR + 1) ) / ADC_MAX; // i.e. (1450 * 1000) / 4095 = 354 -> 35,4 % pwm

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, capture_compare_reg);
}

/* ------------------------------------------ */
/* -- Aufgabenteil D -- */
/* -------------------------------------------*/

// Interrupt Callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* ----------------------------- */
  /* -- Aufgabenteil C -- */

  // Set to 10ms at runtime (1 µs * 10.000 = 10ms)
  //__HAL_TIM_SET_PRESCALER(&htim1, 83);        // PSC = 83 -> 84/84 = 1MHz = 1 µs
  //__HAL_TIM_SET_AUTORELOAD(&htim1, 9999);     // 10.000 ticks

  // Set to 100ms at runtime (10 µs * 10.000 = 10ms)
  //__HAL_TIM_SET_AUTORELOAD(&htim1, 9999);     // 10.000 ticks
  //__HAL_TIM_SET_PRESCALER(&htim1, 839);       // PSC = 839 -> 84/840 = 0,1 MHz = 10 µs

  // Start PWM Signal (Time 1, Channel 1)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  /* ----------------------------- */

  /* ----------------------------- */
  /* -- Aufgabenteil D -- */


  
  /* ----------------------------- */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /** -- Aufgabenteil A - Toggeln der LED -- */
    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    //HAL_Delay(1000);

    /* -- Aufgabenteil B - Messung der Spannung -- **/
    cur_voltage = convert_to_voltage(read_adc_avg_float());
    cur_voltage_avg = convert_to_voltage(read_adc_avg_int());
    avg_adc_reading_int = read_adc_avg_int();
    avg_adc_reading_float = read_adc_avg_float();

    //toggle_led(); // toggle led with current adc reading

    /* -- Aufgabenteil C - Dimmen der LED -- **/
    led_brightness_pwm(); // determine brightness for external led

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
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
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
