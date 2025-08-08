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
#include "stm32f1xx_hal.h"
#include "ssd1306.h"
#include "fonts.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
#define SET_RTC

#define DS3231_ADDR (0x68 << 1) // DS3231 I2C address (write/read shifted)
#define DS3231_REG_TIME 0x00    // seconds register
#define DS3231_REG_DATE 0x03    // day-of-week register
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Buffer and time variables
char line1[20], line2[24];
uint8_t hour, min, sec, day, month, dow;
uint16_t year;
const char* dow_str[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
// Convert BCD to decimal
static uint8_t bcd2dec(uint8_t val) {
	return (uint8_t)(((val >> 4) * 10) + (val & 0x0F));
}
// Convert decimal to BCD
#ifdef SET_RTC
static uint8_t dec2bcd(uint8_t val) {
  return (uint8_t)(((val / 10) << 4) | (val % 10));
}

// Write date & time int DS3231 (one-time initialization)
static void DS3231_SetTime(uint8_t h, uint8_t m, uint8_t s) {
	uint8_t t[3] = {
			dec2bcd(s),  // Seconds
			dec2bcd(m),  // Minutes
			dec2bcd(h),  // Hours
	};
	HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDR, DS3231_REG_TIME, I2C_MEMADD_SIZE_8BIT, t, 3, HAL_MAX_DELAY);
}

static void DS3231_SetDate(uint8_t W, uint8_t D, uint8_t M, uint8_t Y) {
	uint8_t d[4] = {
			dec2bcd(W),  // Day-of-week
			dec2bcd(D),  // Date
			dec2bcd(M),  // Month
			dec2bcd(Y)   // Year (00-99)
	};
	HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDR, DS3231_REG_DATE, I2C_MEMADD_SIZE_8BIT, d, 4, HAL_MAX_DELAY);
}
#endif

// Read time (hh:mm:ss) from DS3231
static void DS3231_GetTime(uint8_t* h, uint8_t* m, uint8_t* s) {
	uint8_t b[3];
	if (HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDR, DS3231_REG_TIME, I2C_MEMADD_SIZE_8BIT, b, 3, HAL_MAX_DELAY) == HAL_OK) {
		*s = bcd2dec(b[0] & 0x7F);
		*m = bcd2dec(b[1] & 0x7F);
		*h = bcd2dec(b[2] & 0x3F);
	} else {
		*h = *m = *s = 0;
	}
}
// Read date and day-of-week
static void DS3231_GetDate(uint8_t* D, uint8_t* M, uint16_t* Y, uint8_t* W) {
	uint8_t b[4];
	if (HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDR, DS3231_REG_DATE, I2C_MEMADD_SIZE_8BIT, b, 4, HAL_MAX_DELAY) == HAL_OK) {
		*W = bcd2dec(b[0]);         // Day-of-week
		*D = bcd2dec(b[1] & 0x3F);  // Date
		*M = bcd2dec(b[2] & 0x1F);  // Month
		*Y = 2000 + bcd2dec(b[3]);  // Year
	} else {
		*W = *D = *M = 0;
		*Y = 2000;
	}
}
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // Initialize OLED display
  SSD1306_Init();
  /* USER CODE END 2 */
  // To set RTC once, define SET_RTC and uncomment:
#ifdef SET_RTC
  // example: 2025-08-07 17:45:30, Thursday(4)
  DS3231_SetTime(17, 45, 30);
  DS3231_SetDate(5, 7, 8, 25);
  HAL_Delay(500);
#endif
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  // Read time and date from DS3231
	  DS3231_GetTime(&hour, &min, &sec);
	  DS3231_GetDate(&day, &month, &year, &dow);

	  // Format strings for display
	  sprintf(line1, "%02d:%02d:%02d", hour, min, sec);
	  sprintf(line2, "%02d-%02d-%04d %s", day, month, year, dow_str[(dow>=1 && dow<=7)?dow-1:0]);

	  // Update OLED screen
	  SSD1306_Fill(SSD1306_COLOR_BLACK);
	  SSD1306_GotoXY(0, 0);
	  SSD1306_Puts(line1, &Font_11x18, SSD1306_COLOR_WHITE);
	  SSD1306_GotoXY(0, 24);
	  SSD1306_Puts(line2, &Font_7x10, SSD1306_COLOR_WHITE);
	  SSD1306_UpdateScreen();

	  HAL_Delay(1000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
