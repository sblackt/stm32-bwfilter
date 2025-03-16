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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ssd1306.h"
#include <stdio.h>
#include "ssd1306_fonts.h"
#include "arm_math.h"

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

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;


/* USER CODE BEGIN PV */
#define AUDIO_BUFFER_SIZE 1024
#define FFT_SIZE 1024  // Must be power of 2 for CMSIS-DSP FFT

uint16_t adc_buffer[AUDIO_BUFFER_SIZE];  // ADC samples
uint16_t pwm_buffer[AUDIO_BUFFER_SIZE];  // PWM output

float32_t fft_input[FFT_SIZE * 2];  // Complex input for FFT
float32_t fft_output[FFT_SIZE * 2]; // Complex output for FFT

arm_rfft_fast_instance_f32 fft_instance;  // CMSIS-DSP FFT instance

// Filter Parameters
float center_freq = 700.0f;  // Center frequency in Hz
float bandwidth = 150.0f;  // Bandwidth in Hz
float sample_rate = 16000.0f;  // Sample rate in Hz

volatile int32_t encoder_count = 0;  // Tracks encoder position
volatile uint8_t setting_mode = 0;  // 0 = Adjust Center Frequency, 1 = Adjust Bandwidth
volatile uint8_t last_encoder_state = 0;
volatile uint8_t button_pressed = 0;  // Detects button press

#define MIN_CENTER_FREQ 300.0f  // Lower limit for center frequency
#define MAX_CENTER_FREQ 3000.0f // Upper limit for center frequency
#define MIN_BANDWIDTH 50.0f     // Minimum bandwidth
#define MAX_BANDWIDTH 1000.0f   // Maximum bandwidth

// Global buffers (avoid stack overflow)
float32_t input_f32[AUDIO_BUFFER_SIZE];
float32_t output_f32[AUDIO_BUFFER_SIZE];
/* USER CODE END PV */



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void Update_Display(void);
void Init_Display(void);  //Function prototype
void Read_Encoder(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_TIM2_Init(void);
void MX_I2C1_Init(void);
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
  MX_TIM2_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);  // Wait for OLED power-up

  HAL_StatusTypeDef result;
  uint8_t address = 0x3C << 1;  // Default SSD1306 I2C address

  // Check if OLED is detected
  result = HAL_I2C_IsDeviceReady(&hi2c1, address, 3, HAL_MAX_DELAY);
  if (result != HAL_OK) {
      ssd1306_Init();
      ssd1306_Fill(Black);
      ssd1306_SetCursor(10, 10);
      ssd1306_WriteString("I2C FAIL", Font_7x10, White);
      ssd1306_UpdateScreen();
      while (1);
  }

  // ✅ OLED is detected, continue initialization
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(10, 10);
  ssd1306_WriteString("VA3WAV", Font_11x18, White);
  ssd1306_UpdateScreen();

  // ✅ Initialize FFT
  arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);

  Init_Display();
  Update_Display();  // Show filter parameters

  // ✅ Start ADC and PWM DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, AUDIO_BUFFER_SIZE);
  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, (uint32_t*)pwm_buffer, AUDIO_BUFFER_SIZE);

  /* USER CODE END 2 */



  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { Read_Encoder();  // Check encoder rotation
  Update_Display();  // Refresh OLED if value changed
  HAL_Delay(2);  // Avoid excessive updates
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  /* USER CODE END 3 */
  return 0;  // Ensure main() ends properly
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
	  // Reset I2C Bus: Set SCL and SDA High
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SCL High
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);  // SDA High
	    HAL_Delay(10);

	    // Force I2C1 Reset
	    __HAL_RCC_I2C1_FORCE_RESET();
	    HAL_Delay(10);
	    __HAL_RCC_I2C1_RELEASE_RESET();
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

	    hi2c1.Instance = I2C1;
	    hi2c1.Init.ClockSpeed = 100000;  // ✅ Standard I2C speed
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4095;
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
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */


  // ✅ Configure CLK (PB14) and DT (PB13) as input **without internal pull-up**
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;  // ✅ Disable internal pull-up
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // ✅ Configure SW (PB12) as input **without internal pull-up**
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;  // ✅ Disable internal pull-up
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Enable Interrupts for Button Press
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}
/* USER CODE BEGIN 4 */



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {  // Ensure this is from ADC1
        uint32_t i;
        float freq_resolution = sample_rate / FFT_SIZE;
        int low_bin = (center_freq - bandwidth / 2) / freq_resolution;
        int high_bin = (center_freq + bandwidth / 2) / freq_resolution;

        // Convert ADC values to floating point (normalize to -1 to 1)
        for (i = 0; i < FFT_SIZE; i++) {
            fft_input[2 * i] = ((float32_t)adc_buffer[i] / 4096.0f) * 2.0f - 1.0f;  // Real part
            fft_input[2 * i + 1] = 0.0f;  // Imaginary part = 0
        }

        // Perform FFT
        arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);

        // Apply Bandpass Filtering in Frequency Domain
        for (i = 0; i < FFT_SIZE / 2; i++) {
            if (i < low_bin || i > high_bin) {
                fft_output[2 * i] = 0.0f;  // Zero out real part
                fft_output[2 * i + 1] = 0.0f;  // Zero out imaginary part
            }
        }

        // Perform Inverse FFT (IFFT) to reconstruct the filtered signal
        arm_rfft_fast_f32(&fft_instance, fft_output, fft_input, 1);

        // Convert back to PWM values (re-normalize)
        for (i = 0; i < FFT_SIZE; i++) {
            pwm_buffer[i] = (uint16_t)(((fft_input[2 * i] + 1.0f) / 2.0f) * 4096.0f);
        }
    }
}


// OLED Initialization
void Init_Display(void) {
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_SetCursor(10, 10);
    ssd1306_WriteString("Audio DSP Ready", Font_7x10, White);
    ssd1306_UpdateScreen();
    HAL_Delay(1000);  // ✅ Delay so the user sees the message
    Update_Display();  // ✅ Automatically update to show filter settings
}

// Update Display with Current Filter Settings
void Update_Display(void) {
    static float last_bandwidth = -1;
    static float last_center_freq = -1;
    static uint8_t last_mode = 255;

    // ✅ Only update if a value has changed
    if (bandwidth == last_bandwidth && center_freq == last_center_freq && setting_mode == last_mode) {
        return;  // ✅ Skip screen update if values haven’t changed
    }

    char text[20];
    ssd1306_Fill(Black);  // ✅ Clear screen before updating

    // ✅ Show Bandwidth when setting_mode == 0
    if (setting_mode == 0) {
        sprintf(text, "BW: %.0fHz", bandwidth);
        ssd1306_SetCursor(10, 10);
        ssd1306_WriteString(text, Font_11x18, White);
    }
    // ✅ Show Center Frequency when setting_mode == 1
    else {
        sprintf(text, "CF: %.0fHz", center_freq);
        ssd1306_SetCursor(10, 10);
        ssd1306_WriteString(text, Font_11x18, White);
    }

    ssd1306_UpdateScreen();  // ✅ Refresh display after all updates

    // ✅ Store last values to track changes
    last_bandwidth = bandwidth;
    last_center_freq = center_freq;
    last_mode = setting_mode;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    static uint32_t press_start_time = 0;
    uint32_t current_time = HAL_GetTick();

    // ✅ Handle Encoder Rotation (PB13 or PB14)
    if (GPIO_Pin == GPIO_PIN_13 || GPIO_Pin == GPIO_PIN_14) {
        Read_Encoder();  // ✅ Call Read_Encoder() on every valid transition
        return;
    }

    // ✅ Handle Button Press (PB12)
    if (GPIO_Pin == GPIO_PIN_12) {
        // ✅ If the button is pressed (falling edge detected)
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) {
            press_start_time = current_time;  // ✅ Store time when press started
        }
        // ✅ If the button is released (rising edge detected)
        else {
            uint32_t press_duration = current_time - press_start_time;

            // ✅ Long Press Detected (1s)
            if (press_duration > 1000) {
                if (setting_mode == 0) {  // ✅ If in Bandwidth mode
                    if (bandwidth < (MAX_BANDWIDTH + MIN_BANDWIDTH) / 2) {
                        bandwidth = MAX_BANDWIDTH;  // ✅ Jump to max
                    } else {
                        bandwidth = MIN_BANDWIDTH;  // ✅ Jump to min
                    }
                    Update_Display();
                }
            }
            // ✅ Short Press Detected
            else if (press_duration > 150) {
                setting_mode = !setting_mode;  // ✅ Toggle between BW and CF pages
                Update_Display();  // ✅ Immediately update OLED
            }
        }
    }
}




void Read_Encoder(void) {
    static uint8_t last_clk = 1;
    static uint32_t last_step_time = 0;
    uint32_t current_time = HAL_GetTick();

    // ✅ Debounce movement (Ignore fast double steps)
    if (current_time - last_step_time < 5) {  // 5ms debounce
        return;
    }

    uint8_t clk = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
    uint8_t dt = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);

    if (clk == 0 && last_clk == 1) {  // ✅ Detect falling edge of CLK
        int step_size = 10;  // ✅ Normal step size

        if (setting_mode == 0) {  // ✅ Adjust Bandwidth
            if (dt == 1) {
                bandwidth += step_size;
                if (bandwidth > 3000.0f) bandwidth = 3000.0f;
            } else {
                bandwidth -= step_size;
                if (bandwidth < 50.0f) bandwidth = 50.0f;
            }
        }
        else {  // ✅ Adjust Center Frequency
            if (dt == 1) {
                center_freq += step_size;
                if (center_freq > 3000.0f) center_freq = 3000.0f;
            } else {
                center_freq -= step_size;
                if (center_freq < 50.0f) center_freq = 50.0f;
            }
        }

        Update_Display();  // ✅ Update OLED only when value actually changes
        last_step_time = current_time;  // ✅ Store last valid step time
    }

    last_clk = clk;  // ✅ Store last CLK state
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
